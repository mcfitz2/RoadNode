#include "odometer_commands.h"

#include "mileage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace roadnode {
namespace vehicle {

// 1 mile = 1,609,344 mm, so one tenth of a mile is 160,934.4 mm.
uint64_t milesTenthsToMm(uint32_t tenths) { return ((uint64_t)tenths * 1609344u + 5) / 10; }
uint32_t mmToMilesTenths(uint64_t mm) { return (uint32_t)((mm * 10 + 804672u) / 1609344u); }

static bool parseMilesTenths(const char* s, uint32_t& out) {
  uint64_t whole = 0;
  const char* p = s;
  if (*p < '0' || *p > '9') return false;
  for (; *p >= '0' && *p <= '9'; p++) {
    whole = whole * 10 + (uint64_t)(*p - '0');
    if (whole > ODOMETER_MILES_MAX) return false;
  }
  uint32_t frac = 0;
  if (*p == '.') {
    p++;
    if (*p < '0' || *p > '9') return false;
    frac = (uint32_t)(*p++ - '0');
  }
  if (*p) return false;
  uint64_t tenths = whole * 10 + frac;
  if (tenths > (uint64_t)ODOMETER_MILES_MAX * 10) return false;
  out = (uint32_t)tenths;
  return true;
}

static void fmtTenths(char* out, size_t n, uint32_t tenths) {
  snprintf(out, n, "%lu.%lu", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
}

static const char* SCALE_KEY = "odo_scale";

bool parsePercentBp(const char* s, uint32_t& bp) {
  uint32_t whole = 0, digits = 0;
  const char* p = s;
  if (*p < '0' || *p > '9') return false;
  for (; *p >= '0' && *p <= '9'; p++) {
    whole = whole * 10 + (uint32_t)(*p - '0');
    if (++digits > 3) return false;
  }
  uint32_t frac = 0;
  if (*p == '.') {
    p++;
    uint32_t n = 0;
    for (; *p >= '0' && *p <= '9'; p++, n++) {
      if (n == 2) return false;
      frac = frac * 10 + (uint32_t)(*p - '0');
    }
    if (n == 0) return false;
    if (n == 1) frac *= 10;
  }
  if (*p) return false;
  uint32_t v = whole * 100 + frac;
  if (v < Mileage::SCALE_MIN_BP || v > Mileage::SCALE_MAX_BP) return false;
  bp = v;
  return true;
}

bool loadOdoScale(storage::KvStore& kv, uint32_t& bp) {
  char buf[16];
  if (!kv.getString(SCALE_KEY, buf, sizeof(buf))) return false;
  char* end;
  unsigned long v = strtoul(buf, &end, 10);
  if (end == buf || *end || v < Mileage::SCALE_MIN_BP || v > Mileage::SCALE_MAX_BP) return false;
  bp = (uint32_t)v;
  return true;
}

bool saveOdoScale(storage::KvStore& kv, uint32_t bp) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)bp);
  return kv.putString(SCALE_KEY, buf);
}

static void fmtPercent(char* out, size_t n, uint32_t bp) {
  snprintf(out, n, "%lu.%02lu", (unsigned long)(bp / 100), (unsigned long)(bp % 100));
}

bool handleOdometerCommand(OdometerControl& odo, const char* command, char* reply) {
  if (strcmp(command, "odo") == 0) {
    uint64_t total, gps;
    if (!odo.read(total, gps)) {
      strcpy(reply, "Err: busy, retry");
      return true;
    }
    char a[16], b[16];
    fmtTenths(a, sizeof(a), mmToMilesTenths(total));
    fmtTenths(b, sizeof(b), mmToMilesTenths(gps));
    sprintf(reply, "odo %s mi (gps filled %s mi since boot)", a, b);
    return true;
  }
  if (strcmp(command, "odo scale") == 0) {
    uint32_t bp;
    if (!odo.scaleBp(bp)) {
      strcpy(reply, "Err: busy, retry");
      return true;
    }
    char a[16];
    fmtPercent(a, sizeof(a), bp);
    sprintf(reply, "odo scale %s%%", a);
    return true;
  }
  if (strncmp(command, "odo scale ", 10) == 0) {
    uint32_t bp, before;
    if (!parsePercentBp(command + 10, bp)) {
      strcpy(reply, "Err: percent 80-120, two decimals");
      return true;
    }
    if (!odo.scaleBp(before)) {
      strcpy(reply, "Err: busy, retry");
      return true;
    }
    if (!odo.setScaleBp(bp)) {
      strcpy(reply, "Err: not saved, scale unchanged");
      return true;
    }
    char a[16], b[16];
    fmtPercent(a, sizeof(a), bp);
    fmtPercent(b, sizeof(b), before);
    sprintf(reply, "OK odo scale %s%% (was %s%%)", a, b);
    return true;
  }
  if (strncmp(command, "odo set ", 8) == 0) {
    uint32_t tenths;
    if (!parseMilesTenths(command + 8, tenths)) {
      strcpy(reply, "Err: miles 0-1000000, one decimal");
      return true;
    }
    uint64_t before, gps;
    if (!odo.read(before, gps)) {
      strcpy(reply, "Err: busy, retry");
      return true;
    }
    if (!odo.setTotalMm(milesTenthsToMm(tenths))) {
      strcpy(reply, "Err: not saved, odo unchanged");
      return true;
    }
    int32_t delta = (int32_t)tenths - (int32_t)mmToMilesTenths(before);
    char a[16], b[16], d[16];
    fmtTenths(a, sizeof(a), tenths);
    fmtTenths(b, sizeof(b), mmToMilesTenths(before));
    fmtTenths(d, sizeof(d), (uint32_t)(delta < 0 ? -delta : delta));
    sprintf(reply, "OK odo %s mi (was %s, %c%s)", a, b, delta < 0 ? '-' : '+', d);
    return true;
  }
  return false;
}

}  // namespace vehicle
}  // namespace roadnode
