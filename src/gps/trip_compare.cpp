#include "trip_compare.h"

#include <stdio.h>
#include <stdlib.h>

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

static const char* KEY_LAST_TRIP = "trip_cmp";

bool saveTripCompare(storage::KvStore& kv, const TripCompareResult& r) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%lu,%lu,%u,%u,%u", (unsigned long)r.obd_m, (unsigned long)r.gps_m, (unsigned)r.coverage_pm,
           (unsigned)r.ratio_pm, (unsigned)((r.valid ? 1 : 0) | (r.suspect ? 2 : 0)));
  return kv.putString(KEY_LAST_TRIP, buf);
}

bool loadTripCompare(storage::KvStore& kv, TripCompareResult& out) {
  char buf[48];
  if (!kv.getString(KEY_LAST_TRIP, buf, sizeof(buf))) return false;
  unsigned long v[5];
  const char* p = buf;
  for (int i = 0; i < 5; i++) {
    if (*p < '0' || *p > '9') return false;
    char* end;
    v[i] = strtoul(p, &end, 10);
    p = end;
    if (i < 4) {
      if (*p != ',') return false;
      p++;
    }
  }
  if (*p || v[0] > 0xFFFFFFFFul || v[1] > 0xFFFFFFFFul || v[2] > 1000 || v[3] > 65535 || v[4] > 3) return false;
  TripCompareResult r;
  r.obd_m = (uint32_t)v[0];
  r.gps_m = (uint32_t)v[1];
  r.diff_m = (int32_t)r.obd_m - (int32_t)r.gps_m;
  r.coverage_pm = (uint16_t)v[2];
  r.ratio_pm = (uint16_t)v[3];
  r.valid = v[4] & 1;
  r.suspect = v[4] & 2;
  out = r;
  return true;
}

}  // namespace gps
}  // namespace roadnode
