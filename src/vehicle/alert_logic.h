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
};

struct AlertConfig {
  uint32_t periodic_interval_ms = 5u * 60u * 1000u;  // between "still moving" reports
  // alertIf cancels a queued alert as soon as its condition goes false, so the
  // periodic condition must stay true long enough for the send (one attempt per
  // opted-in client, 8 s each). Clamped to half the interval so it always releases
  // and re-arms before the next report is due.
  uint32_t periodic_hold_ms = 2u * 60u * 1000u;
  float moving_kmh = 5.0f;  // speed that counts as moving
};

class AlertLogic {
public:
  explicit AlertLogic(const AlertConfig& cfg = AlertConfig()) : _cfg(cfg), _hold_wanted_ms(cfg.periodic_hold_ms) { clampHold(); }

  // Runtime change of the "still moving" interval; 0 turns periodic reports off.
  // Takes effect from the next report; an interval already running is not cut short.
  void setPeriodicIntervalMs(uint32_t ms) {
    _cfg.periodic_interval_ms = ms;
    clampHold();
  }
  uint32_t periodicIntervalMs() const { return _cfg.periodic_interval_ms; }

  AlertConditions update(const VehicleSnapshot& s, uint32_t now_ms) {
    AlertConditions c;
    // On a hybrid the engine stops and restarts throughout a normal drive, so engine_running
    // alone is not "the vehicle is operating": keying `started` to it re-fires the alert on
    // every restart, and the periodic countdown below restarts with it and may never elapse.
    // Trip state already debounces exactly this (trip.h: active and engine-or-moving, with an
    // end timeout), so a trip in progress holds `operating` true across the gaps.
    const bool operating = s.trip_active || s.engine_running;
    if (operating) _driven = true;
    c.started = operating;
    // Not parked at boot: needs an operating observation first.
    //
    // Keyed to obd_connected, not vehicle_active. vehicle_active means "a frame arrived", and
    // the poller never stops polling, so a gateway that answers our requests with negative
    // responses after shutdown keeps it true forever and parking is never reported (measured on
    // a 2025 Maverick: fully powered off, vehicle_active stuck true, no valid PID at all).
    // obd_connected means the ECU is actually serving data, which is what a shutdown ends.
    // !trip_active guards the other direction: a transient CAN failure mid-drive drops
    // obd_connected for a few polls, and the trip stays active across that.
    c.parked = _driven && !s.engine_running && !s.trip_active && !s.obd_connected;

    if (!operating) {
      _running = false;
      _pulse = false;
    } else {
      if (!_running) {  // just started operating: first report one interval from now
        _running = true;
        _last_ms = now_ms;
        _pulse = false;
      }
      if (_pulse && (int32_t)(now_ms - _pulse_end_ms) >= 0) _pulse = false;
      bool moving = s.has_speed && s.speed_kmh >= _cfg.moving_kmh;
      // Due but stationary: stays due and fires on the first moving check.
      if (!_pulse && moving && _cfg.periodic_interval_ms && (uint32_t)(now_ms - _last_ms) >= _cfg.periodic_interval_ms) {
        _pulse = true;
        _last_ms = now_ms;
        _pulse_end_ms = now_ms + _cfg.periodic_hold_ms;
      }
      c.periodic = _pulse;
    }
    return c;
  }

private:
  void clampHold() {
    _cfg.periodic_hold_ms = _hold_wanted_ms;
    if (_cfg.periodic_hold_ms > _cfg.periodic_interval_ms / 2) _cfg.periodic_hold_ms = _cfg.periodic_interval_ms / 2;
  }

  AlertConfig _cfg;
  uint32_t _hold_wanted_ms;
  bool _driven = false;
  bool _running = false;
  bool _pulse = false;
  uint32_t _last_ms = 0;
  uint32_t _pulse_end_ms = 0;
};

}  // namespace vehicle
}  // namespace roadnode
