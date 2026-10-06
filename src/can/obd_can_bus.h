#pragma once

#include <Arduino.h>

#include "can_manager.h"
#include "obd/obd_manager.h"

namespace roadnode {
namespace can {

// Connects ObdManager to the TWAI driver. The driver must be started in
// Mode::Normal; in ListenOnly mode every send() fails.
class ObdCanBus : public obd::CanBus {
public:
  bool send(const obd::CanFrame& f) override {
    Frame out;
    out.id = f.id;
    out.dlc = f.dlc;
    memcpy(out.data, f.data, 8);
    return transmit(out, 20) == Result::Ok;
  }
  bool receive(obd::CanFrame& f, uint32_t timeout_ms) override {
    Frame in;
    if (can::receive(in, timeout_ms) != Result::Ok || in.extended || in.rtr) return false;
    f.id = in.id;
    f.dlc = in.dlc;
    memcpy(f.data, in.data, 8);
    return true;
  }
  uint32_t nowMs() override { return millis(); }
  bool busOff() override { return can::busOff(); }
  void recover() override { can::recover(); }
};

}  // namespace can
}  // namespace roadnode
