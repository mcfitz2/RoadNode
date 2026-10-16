#pragma once

#include <HeltecV4Board.h>
#include <driver/gpio.h>

#include "vehicle/vehicle_runtime.h"

namespace roadnode {

// Heltec V4 board that checkpoints the odometer before an intentional reboot or power off
// (admin `reboot`, `poweroff`, `shutdown`, `clkreboot`), which MeshCore performs through these
// virtual methods. Not covered: OTA (restarts inside the update handler) and power loss.
//
// The carrier powers the CAN transceiver from VEXT (PROJ-685), which HeltecV4Board::begin()
// switches off. begin() claims it for the whole time the node is awake; sleep() releases it.
class RoadNodeBoard : public HeltecV4Board {
public:
  void begin() {
    gpio_hold_dis((gpio_num_t)PIN_VEXT_EN);  // held off through deep sleep by sleep()
    HeltecV4Board::begin();
    periph_power.claim();  // before sensors.begin() starts TWAI
  }

  // Checkpoint, stop CAN and switch VEXT off, held off through deep sleep (GPIO36 is not an
  // RTC pin, so it would float otherwise). Returns whether the odometer was saved.
  bool sleep() {
    bool saved = vehicle::VehicleRuntime::suspend();
    periph_power.release();
    gpio_hold_en((gpio_num_t)PIN_VEXT_EN);
    gpio_deep_sleep_hold_en();
    return saved;
  }

  void reboot() override {
    vehicle::VehicleRuntime::shutdown();
    HeltecV4Board::reboot();
  }
  void powerOff() override {
    sleep();
    HeltecV4Board::powerOff();
  }
};

}  // namespace roadnode
