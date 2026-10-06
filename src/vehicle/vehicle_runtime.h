#pragma once

#ifdef ARDUINO

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

  // Checkpoint the odometer now (before intentional shutdown/sleep).
  static bool shutdown();

  // Reported by begin(): false if the CAN driver failed to start.
  static bool canStarted();
  static VehicleIdentity& identity();
};

}  // namespace vehicle
}  // namespace roadnode

#endif  // ARDUINO
