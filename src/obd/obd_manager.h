#pragma once

#include <stddef.h>
#include <stdint.h>

#include "obd_pids.h"

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
  Forbidden,         // mode not on the read-only allowlist (never transmitted)
};

const char* statusName(Status s);

struct Response {
  static constexpr size_t MAX = 64;
  uint8_t data[MAX];  // payload after the "4x PID" header
  size_t len = 0;
  uint32_t source_id = 0;
  uint8_t nrc = 0;
};

// Standard OBD-II on 11-bit CAN at 500 kbit/s. Every vehicle gets the same
// settings; there are no per-vehicle profiles.
struct ObdConfig {
  uint32_t request_id = 0x7DF;  // functional broadcast
  uint32_t response_id = 0;     // 0 = accept any 0x7E8-0x7EF
  uint32_t timeout_ms = 200;    // per request
};

// Time between poller steps.
constexpr uint32_t POLL_INTERVAL_MS = 1000;

class ObdManager {
public:
  explicit ObdManager(CanBus& bus, const ObdConfig& config = ObdConfig()) : _bus(bus), _config(config) {}

  // Safety gate: nothing is ever transmitted until this is enabled.
  void enableTransmit(bool on) { _enabled = on; }
  bool transmitEnabled() const { return _enabled; }

  const ObdConfig& config() const { return _config; }

  // Generic request: mode (01, 09, ...) and PID. Handles single and multi-frame replies.
  Status request(uint8_t mode, uint8_t pid, Response& out);

  // Mode 01 request that skips PIDs known unsupported, and decodes the value.
  Status readPid(uint8_t pid, float& value);

  // Queries PID 00/20/40... and fills the table. Returns Ok if at least PID 00 answered.
  Status discover();
  const CapabilityTable& capabilities() const { return _caps; }
  void clearCapabilities() { _caps.clear(); }

  // Any frame received while waiting for a reply, from any ID: counted before the responder-ID
  // filter, so ordinary bus traffic counts too.
  // Used to tell a live vehicle bus from a silent one.
  bool anyFrameSeen() const { return _frames_seen != 0; }
  uint32_t lastFrameMs() const { return _last_frame_ms; }

  // Mode 09 PID 02. Writes a NUL-terminated 17-character VIN.
  Status readVin(char vin[18]);

private:
  Status receiveMessage(uint8_t mode, int pid, uint8_t* msg, size_t& len, uint8_t& nrc,
                        uint32_t& source);
  bool accepts(uint32_t id) const;

  CanBus& _bus;
  ObdConfig _config;
  CapabilityTable _caps;
  bool _enabled = false;
  uint32_t _frames_seen = 0;
  uint32_t _last_frame_ms = 0;
};

// RoadNode is read-only: only these modes can ever be transmitted. Anything that
// could clear codes or change ECU state (04, 14, 2x, 3x, ...) is refused in
// request() before a frame is built.
bool modeAllowed(uint8_t mode);

// 17 characters, no I/O/Q, and a valid North American check digit (position 9).
bool vinValid(const char* vin);
// Same checks except the check digit, which non-North-American VINs may not satisfy.
bool vinWellFormed(const char* vin);

}  // namespace obd
}  // namespace roadnode
