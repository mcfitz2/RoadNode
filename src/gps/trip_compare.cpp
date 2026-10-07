#include "trip_compare.h"

namespace roadnode {
namespace gps {

TripCompareResult compareTrip(uint64_t obd_mm, uint64_t gps_mm, uint32_t fix_ms, uint32_t total_ms,
                              const TripCompareConfig& cfg) {
  TripCompareResult r;
  r.obd_m = (uint32_t)(obd_mm / 1000);
  r.gps_m = (uint32_t)(gps_mm / 1000);
  r.diff_m = (int32_t)r.obd_m - (int32_t)r.gps_m;
  if (total_ms) {
    uint64_t c = (uint64_t)fix_ms * 1000 / total_ms;
    r.coverage_pm = c > 1000 ? 1000 : (uint16_t)c;
  }
  if (r.obd_m < cfg.min_trip_m || r.obd_m == 0) return r;
  if (r.coverage_pm < cfg.min_coverage_pm) return r;
  uint64_t ratio = gps_mm * 1000 / obd_mm;
  r.ratio_pm = ratio > 65535 ? 65535 : (uint16_t)ratio;
  r.valid = true;
  r.suspect = r.ratio_pm < cfg.warn_low_pm || r.ratio_pm > cfg.warn_high_pm;
  return r;
}

}  // namespace gps
}  // namespace roadnode
