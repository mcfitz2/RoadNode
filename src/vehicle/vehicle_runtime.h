#pragma once

#ifdef ARDUINO

#include "obd/obd_pids.h"
#include "odometer_commands.h"
#include "vehicle_identity.h"
#include "vehicle_state.h"

// Owns the whole on-device vehicle stack (CAN, OBD, mileage, NVS) and runs it
// in its own FreeRTOS task on core 0. No MeshCore dependency: telemetry
// consumers only read VehicleTelemetry, so mileage keeps running whether or
// not the mesh side starts, fails or is absent.
namespace roadnode {
namespace vehicle {

class VehicleRuntime {
public:
  // Starts CAN (listen-only unless `transmit`), restores the odometer and
  // starts the poll task. Safe to call once; later calls return the first result.
  static bool begin(const char* vehicle_id, bool transmit);

  // Latest published state. Valid (all unknown) even if begin() was never called.
  static const VehicleTelemetry& telemetry();

  // Checkpoint the odometer now (before intentional shutdown/sleep). Safe from any task. False
  // if begin() has not run, the poll task did not yield in 2 s, or the write failed.
  static bool shutdown();

  // shutdown() plus stop CAN and float the TX pin, so the transceiver's supply (VEXT) can be
  // switched off without back-powering it through its D input. Deep sleep or power off must
  // follow: the poll task stays blocked. CAN is left running if the lock is not taken in 2 s.
  static bool suspend();

  // Unix time for the odometer record's last-trip timestamp. Call only with a trustworthy clock
  // (a stock MeshCore RTC with no sync holds an arbitrary value). No-op before begin().
  static void setTime(uint32_t unix_seconds);

  // Odometer read/resync for the admin command. Takes the poll task's lock.
  static OdometerControl& odometer();

  // Reported by begin(): false if the CAN driver failed to start.
  static bool canStarted();

  // Mode 01 PIDs the vehicle declared supported, as discovered by the poller. Empty until
  // discovery succeeds. Diagnostic only: lets a car test see what a vehicle actually offers
  // instead of inferring it from which values stay unread.
  static const obd::CapabilityTable& capabilities();
  static VehicleIdentity& identity();
};

}  // namespace vehicle
}  // namespace roadnode

#endif  // ARDUINO
