#pragma once

#include <stdint.h>

// Distance integration from OBD vehicle speed. Integer math only: distance is
// kept in millimeters, with the sub-millimeter remainder carried between
// samples, so nothing accumulates rounding error (plan section 10).
namespace roadnode {
namespace vehicle {

class Mileage {
public:
  // Samples further apart than this are not integrated (CAN dropout, sleep).
  static constexpr uint32_t DEFAULT_MAX_GAP_MS = 5000;

  explicit Mileage(uint32_t max_gap_ms = DEFAULT_MAX_GAP_MS) : _max_gap_ms(max_gap_ms) {}

  // Restore persisted state (see storage). Clears the sample history.
  void restore(uint64_t total_mm, uint64_t trip_mm);

  // Feed one speed sample (km/h, as reported by OBD PID 0D). Uses the trapezoid
  // of the previous and current speed over the measured elapsed time.
  // now_ms is a free-running millisecond clock; wraparound is handled.
  void update(uint32_t now_ms, uint8_t speed_kmh);

  // Forget the previous sample, e.g. after losing the OBD link. The next
  // update() only records a baseline and adds no distance.
  void breakContinuity();

  // Trip distance accumulates only while a trip is active. It keeps its value
  // after endTrip() (the last trip) until the next startTrip().
  void startTrip();
  void endTrip();
  bool tripActive() const { return _trip_active; }

  // Distance measured by another source (GPS) while OBD speed was unavailable. Goes into the
  // total, and the trip while one is active; leaves the OBD sample history alone.
  void addExternal(uint64_t mm);
  // Odometer resync (admin command). Replaces the total, drops the sub-mm remainder; trip untouched.
  void setTotal(uint64_t total_mm);

  uint64_t externalMm() const { return _external_mm; }  // added by addExternal() since boot
  uint64_t totalMm() const { return _total_mm; }
  uint64_t tripMm() const { return _trip_mm; }
  uint32_t gapsSkipped() const { return _gaps_skipped; }

private:
  void add(uint64_t mm);

  uint32_t _max_gap_ms;
  uint64_t _total_mm = 0;
  uint64_t _trip_mm = 0;
  uint64_t _external_mm = 0;
  uint32_t _remainder = 0;  // in units of 1/36 mm
  uint32_t _gaps_skipped = 0;
  bool _trip_active = false;
  bool _have_prev = false;
  uint32_t _prev_ms = 0;
  uint8_t _prev_speed = 0;
};

}  // namespace vehicle
}  // namespace roadnode
