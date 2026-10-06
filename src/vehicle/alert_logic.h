#pragma once

#include <stdint.h>

#include "vehicle_state.h"

// Conditions for mesh alerts (issue #30/#51). MeshCore-free: the sensor node
// feeds these to SensorMesh::alertIf, which fires once per false->true edge.
namespace roadnode {
namespace vehicle {

struct AlertConditions {
  bool started = false;   // engine running
  bool parked = false;    // was driven since last start, now engine off and CAN bus quiet
  bool periodic = false;  // "still moving" report is due (held true for a window, see below)
  bool dtc_new = false;   // a code not seen before appeared (held true for dtc_hold_ms)
};

struct AlertConfig {
  uint32_t periodic_interval_ms = 5u * 60u * 1000u;  // between "still moving" reports
  // alertIf cancels a queued alert as soon as its condition goes false, so the
  // periodic condition must stay true long enough for the send (one attempt per
  // opted-in client, 8 s each). Clamped to half the interval so it always releases
  // and re-arms before the next report is due.
  uint32_t periodic_hold_ms = 2u * 60u * 1000u;
  float moving_kmh = 5.0f;  // speed that counts as moving
  uint32_t dtc_hold_ms = 2u * 60u * 1000u;  // same reason as periodic_hold_ms
};

class AlertLogic {
public:
  explicit AlertLogic(const AlertConfig& cfg = AlertConfig()) : _cfg(cfg) {
    if (_cfg.periodic_hold_ms > _cfg.periodic_interval_ms / 2) _cfg.periodic_hold_ms = _cfg.periodic_interval_ms / 2;
  }

  AlertConditions update(const VehicleSnapshot& s, uint32_t now_ms) {
    AlertConditions c;
    if (s.engine_running) _driven = true;
    c.started = s.engine_running;
    // Not parked at boot: needs an engine-running observation first. The poller
    // debounces vehicle_active with its CAN-silence timeout.
    c.parked = _driven && !s.engine_running && !s.vehicle_active;

    c.dtc_new = updateDtc(s, now_ms);

    if (!s.engine_running) {
      _running = false;
      _pulse = false;
    } else {
      if (!_running) {  // engine just started: first report one interval from now
        _running = true;
        _last_ms = now_ms;
        _pulse = false;
      }
      if (_pulse && (int32_t)(now_ms - _pulse_end_ms) >= 0) _pulse = false;
      bool moving = s.has_speed && s.speed_kmh >= _cfg.moving_kmh;
      // Due but stationary: stays due and fires on the first moving check.
      if (!_pulse && moving && (uint32_t)(now_ms - _last_ms) >= _cfg.periodic_interval_ms) {
        _pulse = true;
        _last_ms = now_ms;
        _pulse_end_ms = now_ms + _cfg.periodic_hold_ms;
      }
      c.periodic = _pulse;
    }
    return c;
  }

private:
  // New-code detection. The first successful read after boot is the baseline and
  // does not alert, so a code that was already stored does not re-alert on every
  // ignition cycle. A code that disappears and comes back alerts again.
  bool updateDtc(const VehicleSnapshot& s, uint32_t now_ms) {
    if (_dtc_pulse && (int32_t)(now_ms - _dtc_pulse_end_ms) >= 0) _dtc_pulse = false;
    if (!s.has_dtcs) return _dtc_pulse;
    bool fresh = false;
    for (uint8_t i = 0; i < s.dtc_count && i < VehicleSnapshot::MAX_DTCS; i++) {
      bool known = false;
      for (uint8_t k = 0; k < _known_n; k++)
        if (_known_raw[k] == s.dtc_raw[i] && _known_kind[k] == s.dtc_kind[i]) known = true;
      if (!known) fresh = true;
    }
    _known_n = s.dtc_count;
    for (uint8_t i = 0; i < _known_n; i++) {
      _known_raw[i] = s.dtc_raw[i];
      _known_kind[i] = s.dtc_kind[i];
    }
    if (!_dtc_baseline) {
      _dtc_baseline = true;
    } else if (fresh) {
      _dtc_pulse = true;
      _dtc_pulse_end_ms = now_ms + _cfg.dtc_hold_ms;
    }
    return _dtc_pulse;
  }

  AlertConfig _cfg;
  bool _dtc_baseline = false;
  bool _dtc_pulse = false;
  uint32_t _dtc_pulse_end_ms = 0;
  uint8_t _known_n = 0;
  uint16_t _known_raw[VehicleSnapshot::MAX_DTCS] = {0};
  uint8_t _known_kind[VehicleSnapshot::MAX_DTCS] = {0};
  bool _driven = false;
  bool _running = false;
  bool _pulse = false;
  uint32_t _last_ms = 0;
  uint32_t _pulse_end_ms = 0;
};

}  // namespace vehicle
}  // namespace roadnode
