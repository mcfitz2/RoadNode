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

// One VIN read per boot, deferred while moving. The result goes to
// VehicleIdentity, which persists it; the snapshot copies it from there.
void VehiclePoller::pollVin(uint32_t now_ms) {
  _vin_last_ms = now_ms;
  _vin_attempts++;
  _stats.vin_reads++;
  char vin[18];
  Status st = _obd.readVin(vin);
  if (st == Status::Ok) {
    VehicleIdentity::VinResult r = _identity->onVinRead(vin);
    if (r != VehicleIdentity::VinResult::WriteFailed && r != VehicleIdentity::VinResult::Invalid) _vin_done = true;
    return;
  }
  if (st == Status::Unsupported || (st == Status::NegativeResponse)) _vin_done = true;  // ECU says no: do not nag it
  if (st == Status::Timeout || st == Status::Malformed || st == Status::BusError) account(st);
}

// The configured ID (VehicleIdentity) wins over the one stored in the mileage record.
// The VIN is only known through VehicleIdentity.
void VehiclePoller::copyVehicleId() {
  if (_identity) {
    _identity->copyId(_s.vehicle_id);
    memcpy(_s.vin, _identity->vin(), sizeof(_s.vin));
  } else {
    strncpy(_s.vehicle_id, _mileage.vehicleId(), sizeof(_s.vehicle_id) - 1);
  }
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
    copyVehicleId();
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

  bool moving = got_speed && speed >= _cfg.vin_max_speed_kmh;
  if (_identity && !_vin_done && _vin_attempts < _cfg.vin_max_attempts && _obd.transmitEnabled() &&
      (_discovered || _obd.anyFrameSeen()) && !moving &&
      (_vin_attempts == 0 || now_ms - _vin_last_ms >= _cfg.vin_retry_ms))
    pollVin(now_ms);

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
  // Connected means the ECU is serving data. A negative response keeps _consec_fails at 0 (the ECU
  // is alive), so a gateway refusing every request after shutdown must still read as disconnected
  // once no value is fresh (PROJ-686).
  _s.obd_connected = _consec_fails < _cfg.obd_lost_after_fails && (_s.has_speed || _s.has_rpm || _s.has_battery);
  _s.engine_running = _s.has_rpm ? _s.rpm > 0 : (_s.has_speed && _s.speed_kmh > 0);

  // Distance integrates only over fresh speed samples from a connected ECU.
  TripInput in;
  in.vehicle_active = _s.vehicle_active;
  in.engine_running = _s.engine_running;
  in.speed_kmh = _s.has_speed ? (uint8_t)(_s.speed_kmh > 255 ? 255 : _s.speed_kmh) : 0;
  _mileage.update(now_ms, in, got_speed);

  copyVehicleId();
  _s.total_mm = _mileage.totalMm();
  _s.trip_mm = _mileage.tripMm();
  _s.trip_active = _mileage.tripActive();
  _out.publish(_s);
}

}  // namespace vehicle
}  // namespace roadnode
