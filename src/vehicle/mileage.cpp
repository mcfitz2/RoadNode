#include "mileage.h"

namespace roadnode {
namespace vehicle {

// 1 km/h = 5/18 mm per ms. With the trapezoid average (v0 + v1) / 2:
//   mm = (v0 + v1) * dt_ms * 5 / 36
// The numerator is accumulated, multiplied by the calibration (basis points) and divided by
// 36 * 10000, carrying the remainder.
static constexpr uint32_t DIVISOR = 36 * Mileage::SCALE_UNITY_BP;

void Mileage::restore(uint64_t total_mm, uint64_t trip_mm) {
  _total_mm = total_mm;
  _trip_mm = trip_mm;
  _remainder = 0;
  _trip_active = false;
  _have_prev = false;
}

void Mileage::breakContinuity() { _have_prev = false; }

void Mileage::startTrip() {
  _trip_mm = 0;
  _trip_active = true;
}

void Mileage::endTrip() { _trip_active = false; }

void Mileage::add(uint64_t mm) {
  _total_mm += mm;
  if (_trip_active) _trip_mm += mm;
}

void Mileage::addExternal(uint64_t mm) {
  mm = (mm * _scale_bp + SCALE_UNITY_BP / 2) / SCALE_UNITY_BP;
  add(mm);
  _external_mm += mm;
}

bool Mileage::setScaleBp(uint32_t bp) {
  if (bp < SCALE_MIN_BP || bp > SCALE_MAX_BP) return false;
  _scale_bp = bp;
  _remainder = 0;
  return true;
}

void Mileage::setTotal(uint64_t total_mm) {
  _total_mm = total_mm;
  _remainder = 0;
}

void Mileage::update(uint32_t now_ms, uint8_t speed_kmh) {
  if (_have_prev) {
    uint32_t dt = now_ms - _prev_ms;  // wrap-safe
    if (dt > _max_gap_ms) {
      _gaps_skipped++;
    } else {
      uint64_t acc = (uint64_t)_remainder + (uint64_t)(_prev_speed + speed_kmh) * dt * 5 * _scale_bp;
      add(acc / DIVISOR);
      _remainder = (uint32_t)(acc % DIVISOR);
    }
  }
  _have_prev = true;
  _prev_ms = now_ms;
  _prev_speed = speed_kmh;
}

}  // namespace vehicle
}  // namespace roadnode
