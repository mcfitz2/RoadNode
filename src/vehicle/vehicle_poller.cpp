#include "vehicle_poller.h"

#include <string.h>

namespace roadnode {
namespace vehicle {

using obd::Status;

// Counts one request outcome. An ECU that answers with a negative response is
// alive; only silence, garbage and bus faults count towards "connection lost".
void VehiclePoller::account(Status st) {
  switch (st) {
    case Status::Ok:
      _consec_fails = 0;
      break;
    case Status::NegativeResponse:
      _stats.negative++;
      _consec_fails = 0;
      break;
    case Status::Unsupported:
      _stats.unsupported_skips++;
      break;
    case Status::Timeout:
      _stats.timeouts++;
      _consec_fails++;
      break;
    case Status::Malformed:
      _stats.malformed++;
      _consec_fails++;
      break;
    case Status::BusError:
      _stats.bus_errors++;
      _consec_fails++;
      break;
    case Status::Disabled:
    case Status::Forbidden:
      break;
  }
}

bool VehiclePoller::pollValue(uint8_t pid, float& v, uint32_t now_ms) {
  (void)now_ms;
  Status st = _obd.readPid(pid, v);
  account(st);
  return st == Status::Ok;
}

// Reads stored, pending and permanent DTCs (read-only) and merges them into the
// snapshot. Stored (03) must succeed for the result to count; 07/0A are optional
// and a silent or unsupporting ECU just skips them. Silence on these modes does
// not count against the OBD connection: many ECUs never answer 0A.
void VehiclePoller::pollDtcs() {
  static const obd::DtcMode modes[3] = {obd::DtcMode::Stored, obd::DtcMode::Pending, obd::DtcMode::Permanent};
  static const uint8_t kinds[3] = {DTC_STORED, DTC_PENDING, DTC_PERMANENT};

  VehicleSnapshot next = _s;
  next.dtc_count = 0;
  next.dtc_total = 0;
  bool stored_ok = false;
  for (int m = 0; m < 3; m++) {
    if (_dtc_mode_unsupported[m]) continue;
    obd::DtcList list;
    Status st = _obd.readDtcs(modes[m], list);
    if (st == Status::Unsupported) {
      _dtc_mode_unsupported[m] = true;
      continue;
    }
    if (st == Status::Ok || st == Status::NegativeResponse || st == Status::BusError) account(st);
    if (st != Status::Ok) continue;
    if (m == 0) stored_ok = true;
    next.dtc_total += (uint16_t)list.count;
    for (size_t i = 0; i < list.count && next.dtc_count < VehicleSnapshot::MAX_DTCS; i++) {
      next.dtc_raw[next.dtc_count] = list.codes[i].raw;
      next.dtc_kind[next.dtc_count] = kinds[m];
      next.dtc_count++;
    }
  }
  _stats.dtc_reads++;
  if (!stored_ok) return;  // keep the previous result rather than report a partial one
  _s.has_dtcs = true;
  _s.dtc_total = next.dtc_total;
  _s.dtc_count = next.dtc_count;
  memcpy(_s.dtc_raw, next.dtc_raw, sizeof(_s.dtc_raw));
  memcpy(_s.dtc_kind, next.dtc_kind, sizeof(_s.dtc_kind));
}

void VehiclePoller::step(uint32_t now_ms) {
  _stats.polls++;

  // Bus-off: nothing can be sent. Try to recover at a bounded rate.
  if (_bus.busOff()) {
    if (!_in_bus_off) {
      _in_bus_off = true;
      _stats.bus_off_events++;
      _recovery_started = false;
    }
    if (!_recovery_started || now_ms - _last_recovery >= _cfg.bus_recovery_interval_ms) {
      _bus.recover();
      _last_recovery = now_ms;
      _recovery_started = true;
      _stats.recoveries++;
    }
    _s.obd_connected = false;
    TripInput idle;
    idle.vehicle_active = false;
    idle.engine_running = false;
    idle.speed_kmh = 0;
    _mileage.update(now_ms, idle, false);
    _s.vehicle_active = false;
    _s.engine_running = false;
    _s.has_speed = _s.has_rpm = false;
    strncpy(_s.vehicle_id, _mileage.vehicleId(), sizeof(_s.vehicle_id) - 1);
    _s.total_mm = _mileage.totalMm();
    _s.trip_mm = _mileage.tripMm();
    _s.trip_active = _mileage.tripActive();
    _out.publish(_s);
    return;
  }
  if (_in_bus_off) {
    // Left bus-off. The controller may need an explicit restart after recovery.
    _bus.recover();
    _in_bus_off = false;
    _discovered = false;  // re-learn capabilities: the ECU may have rebooted
  }

  // Capability discovery, retried until an ECU answers.
  if (!_discovered && (!_tried_discovery || now_ms - _last_discovery_try >= _cfg.discovery_retry_ms)) {
    _tried_discovery = true;
    _last_discovery_try = now_ms;
    Status st = _obd.discover();
    account(st);
    if (st == Status::Ok) {
      _discovered = true;
      _stats.discoveries++;
    }
  }

  float speed = 0, rpm = 0, batt = 0;
  bool got_speed = false;
  if (_discovered || _obd.anyFrameSeen() || !_tried_discovery) {
    got_speed = pollValue(obd::PID_SPEED, speed, now_ms);
    if (got_speed) {
      _s.speed_kmh = speed;
      _speed_ms = now_ms;
    }
    if (pollValue(obd::PID_RPM, rpm, now_ms)) {
      _s.rpm = rpm;
      _rpm_ms = now_ms;
    }
    if (pollValue(obd::PID_MODULE_VOLTAGE, batt, now_ms)) {
      _s.battery_v = batt;
      _batt_ms = now_ms;
    }
  }

  // DTCs change rarely: poll at a slow rate, only while the ECU is answering.
  // Deferred while moving so the speed samples keep their 1 Hz spacing; an unreadable
  // speed does not defer (distance is not integrating then anyway).
  bool moving = got_speed && speed >= _cfg.dtc_max_speed_kmh;
  if (!_dtc_clock_set) {  // deferral is measured from the first poll until the first read
    _dtc_clock_set = true;
    _dtc_last_ms = now_ms;
  }
  uint32_t since_dtc = now_ms - _dtc_last_ms;
  bool dtc_due = !_dtc_tried || since_dtc >= _cfg.dtc_poll_interval_ms;  // first read as soon as allowed
  bool dtc_forced = _cfg.dtc_max_defer_ms && since_dtc >= _cfg.dtc_max_defer_ms;
  if (_cfg.dtc_poll_interval_ms && (got_speed || _s.obd_connected) && dtc_due && (!moving || dtc_forced)) {
    _dtc_tried = true;
    _dtc_last_ms = now_ms;
    pollDtcs();
  }

  // Timestamps can be slightly ahead of now_ms (taken during the polls above), so compare signed.
  auto age = [&](uint32_t t) -> int32_t {
    int32_t a = (int32_t)(now_ms - t);
    return a < 0 ? 0 : a;
  };
  auto fresh = [&](uint32_t t) { return t != 0 && (uint32_t)age(t) <= _cfg.value_stale_ms; };
  _s.has_speed = fresh(_speed_ms);
  _s.has_rpm = fresh(_rpm_ms);
  _s.has_battery = fresh(_batt_ms);

  if (_obd.anyFrameSeen()) {
    _s.ever_can_activity = true;
    _s.last_can_activity_ms = _obd.lastFrameMs();
  }
  if (_s.has_speed || _s.has_rpm || _s.has_battery) {
    _s.ever_obd_response = true;
    _s.last_obd_response_ms = now_ms;
  }

  _s.vehicle_active = _s.ever_can_activity && (uint32_t)age(_s.last_can_activity_ms) <= _cfg.can_active_timeout_ms;
  if (_consec_fails >= _cfg.obd_lost_after_fails) _s.obd_connected = false;
  else if (_s.has_speed || _s.has_rpm || _s.has_battery) _s.obd_connected = true;
  _s.engine_running = _s.has_rpm ? _s.rpm > 0 : (_s.has_speed && _s.speed_kmh > 0);

  // Distance integrates only over fresh speed samples from a connected ECU.
  TripInput in;
  in.vehicle_active = _s.vehicle_active;
  in.engine_running = _s.engine_running;
  in.speed_kmh = _s.has_speed ? (uint8_t)(_s.speed_kmh > 255 ? 255 : _s.speed_kmh) : 0;
  _mileage.update(now_ms, in, got_speed);

  strncpy(_s.vehicle_id, _mileage.vehicleId(), sizeof(_s.vehicle_id) - 1);
  _s.total_mm = _mileage.totalMm();
  _s.trip_mm = _mileage.tripMm();
  _s.trip_active = _mileage.tripActive();
  _out.publish(_s);
}

}  // namespace vehicle
}  // namespace roadnode
