#pragma once

#include <stdint.h>

#include "storage/kv_store.h"

// OBD vs GPS trip distance comparison (issue #43, plan section 17). Pure logic,
// MeshCore-free. The GPS distance is a lower bound by construction (chords cut
// curves, fixes lost for a while are not integrated), so a trip only yields a
// verdict when GPS covered most of it; otherwise the result is invalid, never zero.
namespace roadnode {
namespace gps {

struct TripCompareConfig {
  uint32_t min_trip_m = 1000;        // shorter trips are mostly quantisation noise
  uint16_t min_coverage_pm = 900;    // GPS fix time / trip time, per mille
  uint16_t warn_low_pm = 950;        // gps/obd outside [warn_low, warn_high] is suspect
  uint16_t warn_high_pm = 1050;
};

struct TripCompareResult {
  bool valid = false;
  bool suspect = false;
  uint32_t obd_m = 0;
  uint32_t gps_m = 0;
  int32_t diff_m = 0;              // obd - gps
  uint16_t ratio_pm = 0;           // gps / obd * 1000, capped 65535
  uint16_t coverage_pm = 0;        // 0-1000
};

TripCompareResult compareTrip(uint64_t obd_mm, uint64_t gps_mm, uint32_t fix_ms, uint32_t total_ms,
                              const TripCompareConfig& cfg = TripCompareConfig());

// Keeps the last trip's result across reboots (one small NVS string, written once per trip).
// The latest trip always replaces it, so an n/a trip does not leave an older valid ratio on display.
bool saveTripCompare(storage::KvStore& kv, const TripCompareResult& r);
// False (and out untouched) if nothing is stored or the stored value is malformed.
bool loadTripCompare(storage::KvStore& kv, TripCompareResult& out);

}  // namespace gps
}  // namespace roadnode
