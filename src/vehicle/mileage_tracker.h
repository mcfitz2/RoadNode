#pragma once

#include <stdint.h>

#include "mileage.h"
#include "storage/checkpoint_policy.h"
#include "storage/vehicle_storage.h"
#include "trip.h"

// Ties the mileage engine, trip detection and persistence together
// (plan sections 11, 12, 22). Has no MeshCore dependency: mileage must keep
// working whether or not telemetry is available.
namespace roadnode {
namespace vehicle {

class MileageTracker {
public:
  MileageTracker(storage::SlotStore& store, const TripConfig& trip_cfg = TripConfig(),
                 const storage::CheckpointConfig& cp_cfg = storage::CheckpointConfig())
      : _storage(store), _trip(trip_cfg), _policy(cp_cfg) {}

  // Load the persisted record (if any) and restore the odometer.
  // Returns true if a record was found; false on a fresh device.
  bool begin(uint32_t now_ms);

  // Short identifier stored with the record (e.g. "RAV4").
  void setVehicleId(const char* id);

  // Optional unix time source for last_trip_timestamp (0 = unknown).
  void setTime(uint32_t unix_seconds) { _unix_time = unix_seconds; }

  // Call at every sample (about 1 Hz).
  //  in            ignition / engine / speed as currently known
  //  speed_valid   false if the speed reading is stale or the OBD link is down.
  //                Distance is not integrated across invalid samples.
  // Returns the trip event for this sample, so callers can send telemetry.
  TripEvent update(uint32_t now_ms, const TripInput& in, bool speed_valid);

  // Checkpoint now, e.g. before intentional shutdown or deep sleep.
  // Returns false if the write failed (it is retried on later update() calls).
  bool shutdown(uint32_t now_ms);

  uint64_t totalMm() const { return _mileage.totalMm(); }
  uint64_t tripMm() const { return _mileage.tripMm(); }
  bool tripActive() const { return _trip.active(); }
  const Mileage& mileage() const { return _mileage; }

  // True while a required checkpoint (trip end / shutdown) has not been written yet.
  bool savePending() const { return _save_pending; }
  uint32_t saveFailures() const { return _save_failures; }

private:
  bool save(uint32_t now_ms);

  storage::VehicleStorage _storage;
  Mileage _mileage;
  Trip _trip;
  storage::CheckpointPolicy _policy;
  char _vehicle_id[16] = {0};
  uint32_t _unix_time = 0;
  uint32_t _last_trip_ts = 0;
  bool _save_pending = false;
  uint32_t _save_failures = 0;
};

}  // namespace vehicle
}  // namespace roadnode
