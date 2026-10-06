#include "gps_track.h"

#include <math.h>

namespace roadnode {
namespace gps {

static const double DEG2RAD = 3.14159265358979323846 / 180.0;
static const double EARTH_M = 6371000.0;

float GpsTrack::distanceM(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  double p1 = lat1 * 1e-6 * DEG2RAD, p2 = lat2 * 1e-6 * DEG2RAD;
  double dp = p2 - p1, dl = (lon2 - lon1) * 1e-6 * DEG2RAD;
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  return (float)(2 * EARTH_M * asin(sqrt(a)));
}

float GpsTrack::bearingDeg(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  double p1 = lat1 * 1e-6 * DEG2RAD, p2 = lat2 * 1e-6 * DEG2RAD;
  double dl = (lon2 - lon1) * 1e-6 * DEG2RAD;
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double deg = atan2(y, x) / DEG2RAD;
  if (deg < 0) deg += 360.0;
  return (float)deg;
}

void GpsTrack::update(uint32_t now_ms, bool valid, int32_t lat, int32_t lon) {
  if (!valid) return;
  bool gap = _ever_fix && (uint32_t)(now_ms - _last_fix_ms) > _cfg.max_gap_ms;
  _last_fix_ms = now_ms;
  _ever_fix = true;

  if (!_have_anchor || gap) {  // first fix, or back after a gap: start over, add nothing
    _have_anchor = true;
    _lat = lat;
    _lon = lon;
    _anchor_ms = now_ms;
    _last_step_ms = now_ms;
    _has_speed = false;
    _speed_kmh = 0;
    return;
  }

  float d = distanceM(_lat, _lon, lat, lon);
  if (d < _cfg.min_step_m) return;  // within wander of the anchor

  uint32_t dt = now_ms - _anchor_ms;
  float kmh = dt ? d / (dt / 1000.0f) * 3.6f : 0;
  if (kmh > _cfg.max_speed_kmh) {  // jump: re-anchor without counting it
    _lat = lat;
    _lon = lon;
    _anchor_ms = now_ms;
    return;
  }

  _heading = (uint16_t)(bearingDeg(_lat, _lon, lat, lon) + 0.5f) % 360;
  _has_heading = true;
  _speed_kmh = kmh;
  _has_speed = true;
  _last_step_ms = now_ms;
  _frac_mm += (double)d * 1000.0;
  uint64_t whole = (uint64_t)_frac_mm;
  _trip_mm += whole;
  _frac_mm -= (double)whole;
  _lat = lat;
  _lon = lon;
  _anchor_ms = now_ms;
}

GpsTrackState GpsTrack::state(uint32_t now_ms) const {
  GpsTrackState s;
  s.trip_mm = _trip_mm;
  s.has_fix = _ever_fix && (uint32_t)(now_ms - _last_fix_ms) <= _cfg.max_gap_ms;
  if (!s.has_fix) return s;
  s.has_heading = _has_heading;
  s.heading_deg = _heading;
  if (_have_anchor && _has_speed) {
    bool stale = (uint32_t)(now_ms - _last_step_ms) > _cfg.stopped_after_ms;
    s.has_motion = true;
    s.speed_kmh = stale ? 0 : _speed_kmh;
  } else if (_have_anchor && (uint32_t)(now_ms - _last_step_ms) > _cfg.stopped_after_ms) {
    s.has_motion = true;  // fix held steady: stopped
    s.speed_kmh = 0;
  }
  return s;
}

}  // namespace gps
}  // namespace roadnode
