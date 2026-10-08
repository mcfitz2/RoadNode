#pragma once

#include <stdint.h>

// CAN types shared by every backend (ESP32 TWAI, MCP2515) and by host tests. No hardware includes.
namespace roadnode {
namespace can {

enum class Mode : uint8_t {
  Loopback,    // no-ack self-test mode: frames sent with self-reception are received back
  ListenOnly,  // receive only; never transmits or acknowledges
  Normal,      // transmit and receive (needed for OBD requests)
};

enum class Result : uint8_t {
  Ok,
  InvalidBitrate,
  InvalidState,  // begin() while running, or end() while stopped
  DriverError,   // see lastError()
  Timeout,       // no frame received / queue full within the timeout
};

struct Frame {
  uint32_t id = 0;
  uint8_t dlc = 0;
  bool extended = false;  // 29-bit identifier
  bool rtr = false;
  bool self = false;      // transmit: request self-reception (Loopback mode)
  uint8_t data[8] = {0};
};

struct Status {
  uint32_t rx_queued;
  uint32_t tx_queued;
  uint32_t tx_errors;      // TEC
  uint32_t rx_errors;      // REC
  uint32_t tx_failed;
  uint32_t rx_missed;      // dropped: RX queue full
  uint32_t rx_overrun;     // dropped: hardware FIFO overrun
  uint32_t arb_lost;
  uint32_t bus_errors;
  const char* state;       // stopped / running / bus-off / recovering
};

inline const char* resultName(Result r) {
  switch (r) {
    case Result::Ok: return "ok";
    case Result::InvalidBitrate: return "invalid bitrate";
    case Result::InvalidState: return "invalid state";
    case Result::Timeout: return "timeout";
    default: return "driver error";
  }
}

}  // namespace can
}  // namespace roadnode
