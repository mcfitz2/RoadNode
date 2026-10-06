#include "mileage_tracker.h"

#include <string.h>

namespace roadnode {
namespace vehicle {

bool MileageTracker::begin(uint32_t now_ms) {
  storage::VehicleRecord rec;
  bool found = _storage.load(rec);
  if (found) {
    // A trip does not resume across a reboot: trip distance is kept as the
    // last trip's value until the next trip starts.
    _mileage.restore(rec.total_mm, rec.trip_mm);
    _last_trip_ts = rec.last_trip_timestamp;
    if (_vehicle_id[0] == 0) memcpy(_vehicle_id, rec.vehicle_id, sizeof(_vehicle_id));
  }
  _policy.markSaved(now_ms, _mileage.totalMm());
  return found;
}

void MileageTracker::setVehicleId(const char* id) {
  strncpy(_vehicle_id, id, sizeof(_vehicle_id) - 1);
  _vehicle_id[sizeof(_vehicle_id) - 1] = 0;
}

bool MileageTracker::save(uint32_t now_ms) {
  storage::VehicleRecord rec;
  rec.total_mm = _mileage.totalMm();
  rec.trip_mm = _mileage.tripMm();
  rec.last_trip_timestamp = _last_trip_ts;
  memcpy(rec.vehicle_id, _vehicle_id, sizeof(rec.vehicle_id));

  if (!_storage.save(rec)) {
    _save_failures++;
    return false;
  }
  _policy.markSaved(now_ms, rec.total_mm);
  _save_pending = false;
  return true;
}

TripEvent MileageTracker::update(uint32_t now_ms, const TripInput& in, bool speed_valid) {
  TripEvent ev = _trip.update(now_ms, in);
  if (ev == TripEvent::Started) _mileage.startTrip();

  if (speed_valid) {
    _mileage.update(now_ms, in.speed_kmh);
  } else {
    _mileage.breakContinuity();
  }

  if (ev == TripEvent::Ended) {
    _mileage.endTrip();
    if (_unix_time) _last_trip_ts = _unix_time;
    _save_pending = true;
  }

  // Required saves are retried on every sample until they succeed;
  // periodic saves simply stay due until one succeeds.
  if (_save_pending || _policy.due(now_ms, _mileage.totalMm())) save(now_ms);
  return ev;
}

bool MileageTracker::shutdown(uint32_t now_ms) {
  _save_pending = true;
  return save(now_ms);
}

}  // namespace vehicle
}  // namespace roadnode
