#pragma once

#include <stdint.h>

// GPS-derived motion (issue #42): distance, speed and heading computed from
// successive fixes. Kept apart from the OBD odometer and never a replacement for
// OBD speed. MeshCore-free: the telemetry layer feeds it fixes read from MeshCore's
// location provider (which already does the NMEA parsing and RTC time sync, #41).
namespace roadnode {
namespace gps {

struct GpsTrackConfig {
  // A fix counts as movement only once it is this far from the last accepted one.
  // Stationary GNSS wander is a few metres, so this keeps it out of the distance
  // (and means speeds under about 30 km/h are resolved in steps of several seconds).
  float min_step_m = 8.0f;
  uint32_t max_gap_ms = 10000;     // no fix for this long: the next fix starts a new anchor, gap not integrated
  uint32_t stopped_after_ms = 5000;  // no accepted step for this long reads as speed 0
  float max_speed_kmh = 300.0f;    // a step implying more than this is a jump, not travel
};

struct GpsTrackState {
  bool has_fix = false;      // a fix arrived recently
  bool has_motion = false;   // speed known (moving or confirmed stopped)
  float speed_kmh = 0;
  bool has_heading = false;  // set after the first accepted step; kept while stopped
  uint16_t heading_deg = 0;  // 0-359, true north
  uint64_t trip_mm = 0;      // since resetTrip() / boot
};

class GpsTrack {
public:
  explicit GpsTrack(const GpsTrackConfig& cfg = GpsTrackConfig()) : _cfg(cfg) {}

  // One call per sample (about 1 Hz). valid=false means no current fix.
  // lat/lon in degrees * 1e6, the unit MeshCore's LocationProvider uses.
  void update(uint32_t now_ms, bool valid, int32_t lat_e6, int32_t lon_e6);

  void resetTrip() { _trip_mm = 0; }
  GpsTrackState state(uint32_t now_ms) const;

  static float distanceM(int32_t lat1_e6, int32_t lon1_e6, int32_t lat2_e6, int32_t lon2_e6);
  static float bearingDeg(int32_t lat1_e6, int32_t lon1_e6, int32_t lat2_e6, int32_t lon2_e6);

private:
  GpsTrackConfig _cfg;
  bool _have_anchor = false;
  int32_t _lat = 0, _lon = 0;
  uint32_t _anchor_ms = 0;
  uint32_t _last_fix_ms = 0;
  bool _ever_fix = false;
  uint32_t _last_step_ms = 0;
  float _speed_kmh = 0;
  bool _has_speed = false;
  bool _has_heading = false;
  uint16_t _heading = 0;
  uint64_t _trip_mm = 0;
  double _frac_mm = 0;
};

}  // namespace gps
}  // namespace roadnode
