#pragma once

#include <deque>
#include <functional>
#include <vector>

#include "obd/obd_manager.h"

// Scripted CAN bus: records sent frames, replays queued frames on receive.
// Time advances only through receive() timeouts and explicit advance().
class FakeBus : public roadnode::obd::CanBus {
public:
  std::vector<roadnode::obd::CanFrame> sent;
  std::deque<roadnode::obd::CanFrame> inbox;
  bool fail_send = false;
  uint32_t now = 0;
  // Called after each send so tests can enqueue a reply (e.g. flow-control driven).
  std::function<void(const roadnode::obd::CanFrame&)> on_send;

  bool send(const roadnode::obd::CanFrame& f) override {
    if (fail_send) return false;
    sent.push_back(f);
    if (on_send) on_send(f);
    return true;
  }
  bool receive(roadnode::obd::CanFrame& f, uint32_t timeout_ms) override {
    if (inbox.empty()) {
      now += timeout_ms;
      return false;
    }
    f = inbox.front();
    inbox.pop_front();
    now += 1;
    return true;
  }
  uint32_t nowMs() override { return now; }
  bool bus_off = false;
  bool recovers = false;  // recover() clears bus_off
  int recover_calls = 0;
  bool busOff() override { return bus_off; }
  void recover() override {
    recover_calls++;
    if (recovers) bus_off = false;
  }

  void push(uint32_t id, std::initializer_list<uint8_t> bytes) {
    roadnode::obd::CanFrame f;
    f.id = id;
    f.dlc = 8;
    size_t i = 0;
    for (uint8_t b : bytes) f.data[i++] = b;
    inbox.push_back(f);
  }
};
