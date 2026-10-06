#pragma once

#include <stdint.h>

// Decides when to checkpoint mileage to flash, so we never write per speed
// sample (plan section 12). Pure logic, no I/O.
namespace roadnode {
namespace storage {

struct CheckpointConfig {
  uint64_t distance_mm = 804672;     // 0.5 mile
  uint32_t interval_ms = 120000;     // 2 minutes
};

class CheckpointPolicy {
public:
  explicit CheckpointPolicy(const CheckpointConfig& cfg = CheckpointConfig()) : _cfg(cfg) {}

  // True when a periodic checkpoint is due: distance has changed since the last
  // save AND (at least distance_mm has been covered OR interval_ms has passed).
  // Nothing is due while parked (no distance change), which avoids pointless writes.
  bool due(uint32_t now_ms, uint64_t total_mm) const {
    if (total_mm == _saved_mm) return false;
    return (total_mm - _saved_mm) >= _cfg.distance_mm || (uint32_t)(now_ms - _saved_ms) >= _cfg.interval_ms;
  }

  // Call after every successful save (periodic, trip end, or before shutdown).
  void markSaved(uint32_t now_ms, uint64_t total_mm) {
    _saved_ms = now_ms;
    _saved_mm = total_mm;
  }

private:
  CheckpointConfig _cfg;
  uint32_t _saved_ms = 0;
  uint64_t _saved_mm = 0;
};

}  // namespace storage
}  // namespace roadnode
