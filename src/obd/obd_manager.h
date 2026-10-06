#pragma once

#include <stddef.h>
#include <stdint.h>

#include "dtc.h"
#include "obd_pids.h"
#include "vehicle_profiles.h"

// OBD-II request/response over CAN with ISO-TP (plan sections 8, 16).
// Talks to the bus through CanBus, so it has no ESP32 or MeshCore dependency
// and is unit-tested on the host.
namespace roadnode {
namespace obd {

struct CanFrame {
  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {0};
};

class CanBus {
public:
  virtual ~CanBus() {}
  virtual bool send(const CanFrame& f) = 0;
  // Waits up to timeout_ms; false if nothing arrived.
  virtual bool receive(CanFrame& f, uint32_t timeout_ms) = 0;
  virtual uint32_t nowMs() = 0;
  // Controller bus-off state and recovery trigger. Defaults suit buses without either.
  virtual bool busOff() { return false; }
  virtual void recover() {}
};

enum class Status : uint8_t {
  Ok,
  Disabled,          // transmit safety gate closed
  Unsupported,       // capability table says the PID is not supported
  Timeout,           // no matching response
  NegativeResponse,  // see Response::nrc
  Malformed,         // bad ISO-TP or response too large
  BusError,          // send failed
};

const char* statusName(Status s);

struct Response {
  static constexpr size_t MAX = 64;
  uint8_t data[MAX];  // payload after the "4x PID" header
  size_t len = 0;
  uint32_t source_id = 0;
  uint8_t nrc = 0;
};

class ObdManager {
public:
  ObdManager(CanBus& bus, const VehicleProfile& profile) : _bus(bus), _profile(&profile) {}

  // Safety gate: nothing is ever transmitted until this is enabled.
  void enableTransmit(bool on) { _enabled = on; }
  bool transmitEnabled() const { return _enabled; }

  void setProfile(const VehicleProfile& p) { _profile = &p; }
  const VehicleProfile& profile() const { return *_profile; }

  // Generic request: mode (01, 09, ...) and PID. Handles single and multi-frame replies.
  Status request(uint8_t mode, uint8_t pid, Response& out);

  // Request with no PID byte (modes 03, 07, 0A). out.data is the payload after the mode byte.
  Status requestMode(uint8_t mode, Response& out);

  // Mode 01 request that skips PIDs known unsupported, and decodes the value.
  Status readPid(uint8_t pid, float& value);

  // Queries PID 00/20/40... and fills the table. Returns Ok if at least PID 00 answered.
  Status discover();
  const CapabilityTable& capabilities() const { return _caps; }
  void clearCapabilities() { _caps.clear(); }

  // Any frame at all from a responder ID, whether or not it answered us.
  // Used to tell a live vehicle bus from a silent one.
  bool anyFrameSeen() const { return _frames_seen != 0; }
  uint32_t lastFrameMs() const { return _last_frame_ms; }

  // Mode 03 / 07 / 0A. Read only. Unsupported modes (negative response 0x11/0x12)
  // return Unsupported; a silent ECU returns Timeout (common for 0A on older cars).
  // The first responding ECU answers; more than DtcList::MAX codes is Malformed.
  Status readDtcs(DtcMode mode, DtcList& out);

  // Mode 09 PID 02. Writes a NUL-terminated 17-character VIN.
  Status readVin(char vin[18]);

private:
  Status receiveMessage(uint8_t mode, int pid, uint8_t* msg, size_t& len, uint8_t& nrc,
                        uint32_t& source);
  bool accepts(uint32_t id) const;

  CanBus& _bus;
  const VehicleProfile* _profile;
  CapabilityTable _caps;
  bool _enabled = false;
  uint32_t _frames_seen = 0;
  uint32_t _last_frame_ms = 0;
};

// 17 characters, no I/O/Q, and a valid North American check digit (position 9).
bool vinValid(const char* vin);
// Same checks except the check digit, which non-North-American VINs may not satisfy.
bool vinWellFormed(const char* vin);

}  // namespace obd
}  // namespace roadnode
