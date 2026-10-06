#pragma once

#include <stdint.h>

#include "mileage_tracker.h"
#include "obd/obd_manager.h"
#include "vehicle_state.h"

// Drives the OBD link, feeds mileage and publishes VehicleTelemetry.
// Failure handling (plan section 22): every failure degrades to "unknown"
// values and a stopped odometer, never to invented distance or a crash.
namespace roadnode {
namespace vehicle {

struct PollerConfig {
  uint32_t can_active_timeout_ms = 10000;   // no frames for this long = vehicle inactive
  uint32_t obd_lost_after_fails = 3;        // consecutive failed polls before obd_connected clears
  uint32_t discovery_retry_ms = 5000;
  uint32_t bus_recovery_interval_ms = 1000;
  uint32_t value_stale_ms = 3000;           // live values older than this read as unknown
  uint32_t dtc_poll_interval_ms = 30000;    // modes 03/07/0A; 0 disables DTC polling
};

struct PollerStats {
  uint32_t polls = 0;
  uint32_t timeouts = 0;
  uint32_t malformed = 0;
  uint32_t negative = 0;
  uint32_t bus_errors = 0;
  uint32_t unsupported_skips = 0;
  uint32_t dtc_reads = 0;
  uint32_t discoveries = 0;
  uint32_t bus_off_events = 0;
  uint32_t recoveries = 0;
};

class VehiclePoller {
public:
  VehiclePoller(obd::ObdManager& obd, obd::CanBus& bus, MileageTracker& mileage, VehicleTelemetry& out,
                const PollerConfig& cfg = PollerConfig())
      : _obd(obd), _bus(bus), _mileage(mileage), _out(out), _cfg(cfg) {}

  // Run one poll cycle (blocks for at most a few request timeouts).
  // Call at the profile's poll interval.
  void step(uint32_t now_ms);

  // Checkpoint mileage before an intentional shutdown.
  bool shutdown(uint32_t now_ms) { return _mileage.shutdown(now_ms); }

  const PollerStats& stats() const { return _stats; }
  bool discovered() const { return _discovered; }

private:
  void account(obd::Status st);
  bool pollValue(uint8_t pid, float& v, uint32_t now_ms);
  void pollDtcs();

  obd::ObdManager& _obd;
  obd::CanBus& _bus;
  MileageTracker& _mileage;
  VehicleTelemetry& _out;
  PollerConfig _cfg;
  PollerStats _stats;
  VehicleSnapshot _s;

  bool _discovered = false;
  uint32_t _last_discovery_try = 0;
  bool _tried_discovery = false;
  uint32_t _consec_fails = 0;
  bool _in_bus_off = false;
  uint32_t _last_recovery = 0;
  bool _recovery_started = false;
  uint32_t _speed_ms = 0, _rpm_ms = 0, _batt_ms = 0;
  bool _dtc_tried = false;
  uint32_t _dtc_last_ms = 0;
  bool _dtc_mode_unsupported[3] = {false, false, false};  // 03, 07, 0A
};

}  // namespace vehicle
}  // namespace roadnode
