#pragma once

#include <stddef.h>
#include <stdint.h>

#include "can_types.h"

// Microchip MCP2515 (and the pin-compatible MCP25625) driver over an abstract SPI bus.
// Hardware independent so it runs against a fake chip in host tests. No MeshCore dependencies.
// Not thread safe: the caller serialises access.
namespace roadnode {
namespace can {

class SpiBus {
public:
  virtual ~SpiBus() = default;
  // One chip-select framed transaction: clocks out tx[0..n) while capturing rx[0..n). rx may be null.
  virtual void transfer(const uint8_t* tx, uint8_t* rx, size_t n) = 0;
  virtual void delayMs(uint32_t ms) = 0;
};

class Mcp2515 {
public:
  explicit Mcp2515(SpiBus& spi) : _spi(spi) {}

  // Resets the chip, programs the bit timing for osc_hz and bitrate_bps, opens every RX filter and
  // enters the requested mode. False if the chip does not answer or the bitrate is unreachable.
  bool begin(uint32_t bitrate_bps, uint32_t osc_hz, Mode mode);
  bool validBitrate(uint32_t bitrate_bps, uint32_t osc_hz) const;

  // Loads a free transmit buffer and requests transmission. False when all three buffers are busy
  // (a buffer stuck without an ACK is aborted so the next call can succeed) or in listen-only mode.
  bool send(const Frame& f);

  // Reads one received frame. False when none is pending.
  bool receive(Frame& f);

  // Reads error state and folds the chip's sticky error flags into the running counters.
  void poll();
  bool busOff();
  uint8_t tec();
  uint8_t rec();

  uint32_t rxOverruns() const { return _rx_overruns; }
  uint32_t busErrors() const { return _bus_errors; }
  uint32_t txAborted() const { return _tx_aborted; }

  // Register helpers (public for tests and diagnostics).
  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t value);
  void bitModify(uint8_t addr, uint8_t mask, uint8_t value);

  static bool timing(uint32_t bitrate_bps, uint32_t osc_hz, uint8_t& cnf1, uint8_t& cnf2, uint8_t& cnf3);

private:
  bool setMode(uint8_t reqop);
  int freeTxBuffer();
  SpiBus& _spi;
  Mode _mode = Mode::ListenOnly;
  uint32_t _rx_overruns = 0;
  uint32_t _bus_errors = 0;
  uint32_t _tx_aborted = 0;
};

}  // namespace can
}  // namespace roadnode
