#include "trip.h"

namespace roadnode {
namespace vehicle {

TripEvent Trip::update(uint32_t now_ms, const TripInput& in) {
  bool operating = in.vehicle_active && (in.engine_running || in.speed_kmh > 0);

  if (!_active) {
    if (!operating) return TripEvent::None;
    _active = true;
    _idle_timing = false;
    return TripEvent::Started;
  }

  if (operating) {
    _idle_timing = false;
    return TripEvent::None;
  }

  if (!_idle_timing) {
    _idle_timing = true;
    _idle_since_ms = now_ms;
    return TripEvent::None;
  }

  if (now_ms - _idle_since_ms >= _cfg.end_timeout_ms) {
    _active = false;
    _idle_timing = false;
    return TripEvent::Ended;
  }
  return TripEvent::None;
}

void Trip::reset() {
  _active = false;
  _idle_timing = false;
}

}  // namespace vehicle
}  // namespace roadnode
