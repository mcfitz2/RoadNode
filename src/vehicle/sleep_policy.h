#pragma once

#include <stdint.h>

// When the parked node may deep sleep (PROJ-596). MeshCore-free and host-tested;
// the node calls update() with the alert conditions and the carrier's CAR_ON line,
// and enters deep sleep (wake on CAR_ON high) when it returns true.
//
// RoadNode does not repeat, so nothing on the node needs to stay up while parked.
// The parked alert gets parked_grace_ms to go out first: a high-priority alert makes
// up to 4 attempts, 8 s apart, per opted-in client, and the channel post is immediate.
namespace roadnode {
namespace vehicle {

struct SleepConfig {
  uint32_t parked_grace_ms = 2u * 60u * 1000u;  // parked alert send window
  // Not operating and not parked (never driven since boot, or key on with the engine off).
  // CAR_ON wakes the node again as soon as the engine runs, so sleeping here loses nothing.
  uint32_t idle_ms = 10u * 60u * 1000u;
};

class SleepPolicy {
public:
  explicit SleepPolicy(const SleepConfig& cfg = SleepConfig()) : _cfg(cfg) {}

  // operating: trip active or engine running. parked: AlertConditions::parked.
  // car_on: the carrier's wake line; while it is high, ext0 wake would fire at once, so never sleep.
  bool update(bool operating, bool parked, bool car_on, uint32_t now_ms) {
    if (!_started) {
      _started = true;
      _idle_since = now_ms;
    }
    if (operating || car_on) {
      _idle_since = now_ms;
      _parked = false;
      return false;
    }
    if (parked) {
      if (!_parked) {
        _parked = true;
        _parked_since = now_ms;
      }
      return (uint32_t)(now_ms - _parked_since) >= _cfg.parked_grace_ms;
    }
    _parked = false;
    return (uint32_t)(now_ms - _idle_since) >= _cfg.idle_ms;
  }

private:
  SleepConfig _cfg;
  bool _started = false;
  bool _parked = false;
  uint32_t _idle_since = 0;
  uint32_t _parked_since = 0;
};

}  // namespace vehicle
}  // namespace roadnode
