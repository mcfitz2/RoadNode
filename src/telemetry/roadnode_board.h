#pragma once

#include <HeltecV4Board.h>

#include "vehicle/vehicle_runtime.h"

namespace roadnode {

// Heltec V4 board that checkpoints the odometer before an intentional reboot or power off
// (admin `reboot`, `poweroff`, `shutdown`, `clkreboot`), which MeshCore performs through these
// virtual methods. Not covered: OTA (restarts inside the update handler) and power loss.
class RoadNodeBoard : public HeltecV4Board {
public:
  void reboot() override {
    vehicle::VehicleRuntime::shutdown();
    HeltecV4Board::reboot();
  }
  void powerOff() override {
    vehicle::VehicleRuntime::shutdown();
    HeltecV4Board::powerOff();
  }
};

}  // namespace roadnode
