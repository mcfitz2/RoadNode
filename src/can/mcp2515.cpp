#include "mcp2515.h"

#include <string.h>

namespace roadnode {
namespace can {

namespace {

// SPI instructions.
constexpr uint8_t CMD_RESET = 0xC0;
constexpr uint8_t CMD_READ = 0x03;
constexpr uint8_t CMD_WRITE = 0x02;
constexpr uint8_t CMD_BITMOD = 0x05;
constexpr uint8_t CMD_READ_RX = 0x90;  // | 0x04 for buffer 1; CS going high clears the RXnIF flag
constexpr uint8_t CMD_LOAD_TX = 0x40;  // | (n << 1); starts at TXBnSIDH
constexpr uint8_t CMD_RTS = 0x80;      // | (1 << n)
constexpr uint8_t CMD_READ_STATUS = 0xA0;

// Registers.
constexpr uint8_t REG_TEC = 0x1C;
constexpr uint8_t REG_REC = 0x1D;
constexpr uint8_t REG_CANSTAT = 0x0E;
constexpr uint8_t REG_CANCTRL = 0x0F;
constexpr uint8_t REG_CNF3 = 0x28;
constexpr uint8_t REG_CNF2 = 0x29;
constexpr uint8_t REG_CNF1 = 0x2A;
constexpr uint8_t REG_CANINTE = 0x2B;
constexpr uint8_t REG_CANINTF = 0x2C;
constexpr uint8_t REG_EFLG = 0x2D;
constexpr uint8_t REG_TXB0CTRL = 0x30;  // buffers are 0x10 apart
constexpr uint8_t REG_RXB0CTRL = 0x60;
constexpr uint8_t REG_RXB1CTRL = 0x70;

constexpr uint8_t MODE_NORMAL = 0x00;
constexpr uint8_t MODE_LOOPBACK = 0x40;
constexpr uint8_t MODE_LISTEN = 0x60;
constexpr uint8_t MODE_CONFIG = 0x80;
constexpr uint8_t MODE_MASK = 0xE0;

constexpr uint8_t CANCTRL_ABAT = 0x10;
constexpr uint8_t CANINTF_ERRIF = 0x20;
constexpr uint8_t EFLG_TXBO = 0x20;
constexpr uint8_t EFLG_RX1OVR = 0x80;
constexpr uint8_t EFLG_RX0OVR = 0x40;
constexpr uint8_t TXB_TXREQ = 0x08;
constexpr uint8_t SIDL_IDE = 0x08;
constexpr uint8_t SIDL_SRR = 0x10;
constexpr uint8_t DLC_RTR = 0x40;

uint8_t modeBits(Mode m) {
  switch (m) {
    case Mode::Loopback: return MODE_LOOPBACK;
    case Mode::ListenOnly: return MODE_LISTEN;
    default: return MODE_NORMAL;
  }
}

}  // namespace

// 16 time quanta per bit, sample point at 9/16 (same values Adafruit's MCP2515 library uses).
// One quantum is 2 * (BRP + 1) / osc, so BRP = osc / (32 * bitrate) - 1 and must divide exactly.
bool Mcp2515::timing(uint32_t bitrate_bps, uint32_t osc_hz, uint8_t& cnf1, uint8_t& cnf2, uint8_t& cnf3) {
  if (!bitrate_bps || !osc_hz) return false;
  uint64_t div = (uint64_t)32 * bitrate_bps;
  if (osc_hz % div) return false;
  uint64_t brp = osc_hz / div;
  if (brp < 1 || brp > 64) return false;
  cnf1 = (uint8_t)(brp - 1);  // SJW = 1 TQ
  cnf2 = 0xF0;                // BTLMODE, triple sample, PS1 = 7 TQ, PropSeg = 1 TQ
  cnf3 = 0x86;                // SOF, PS2 = 7 TQ
  return true;
}

bool Mcp2515::validBitrate(uint32_t bitrate_bps, uint32_t osc_hz) const {
  uint8_t a, b, c;
  return timing(bitrate_bps, osc_hz, a, b, c);
}

uint8_t Mcp2515::readReg(uint8_t addr) {
  uint8_t tx[3] = {CMD_READ, addr, 0}, rx[3] = {0, 0, 0};
  _spi.transfer(tx, rx, 3);
  return rx[2];
}

void Mcp2515::writeReg(uint8_t addr, uint8_t value) {
  uint8_t tx[3] = {CMD_WRITE, addr, value};
  _spi.transfer(tx, nullptr, 3);
}

void Mcp2515::bitModify(uint8_t addr, uint8_t mask, uint8_t value) {
  uint8_t tx[4] = {CMD_BITMOD, addr, mask, value};
  _spi.transfer(tx, nullptr, 4);
}

bool Mcp2515::setMode(uint8_t reqop) {
  bitModify(REG_CANCTRL, MODE_MASK, reqop);
  for (int i = 0; i < 20; i++) {
    if ((readReg(REG_CANSTAT) & MODE_MASK) == reqop) return true;
    _spi.delayMs(1);
  }
  return false;
}

bool Mcp2515::begin(uint32_t bitrate_bps, uint32_t osc_hz, Mode mode) {
  uint8_t cnf1, cnf2, cnf3;
  if (!timing(bitrate_bps, osc_hz, cnf1, cnf2, cnf3)) return false;

  uint8_t reset = CMD_RESET;
  _spi.transfer(&reset, nullptr, 1);
  _spi.delayMs(10);
  if ((readReg(REG_CANSTAT) & MODE_MASK) != MODE_CONFIG) return false;  // no chip, or not in reset

  writeReg(REG_CNF1, cnf1);
  writeReg(REG_CNF2, cnf2);
  writeReg(REG_CNF3, cnf3);
  if (readReg(REG_CNF1) != cnf1 || readReg(REG_CNF2) != cnf2) return false;  // SPI wiring check

  for (uint8_t n = 0; n < 3; n++) writeReg(REG_TXB0CTRL + n * 0x10, 0x00);
  writeReg(REG_RXB0CTRL, 0x64);  // accept every frame, roll over into buffer 1
  writeReg(REG_RXB1CTRL, 0x60);
  writeReg(REG_CANINTF, 0x00);
  writeReg(REG_CANINTE, 0x03);  // INT pin asserts on a frame in either RX buffer

  _mode = mode;
  _rx_overruns = _bus_errors = _tx_aborted = 0;
  return setMode(modeBits(mode));
}

int Mcp2515::freeTxBuffer() {
  for (int n = 0; n < 3; n++) {
    if (!(readReg(REG_TXB0CTRL + n * 0x10) & TXB_TXREQ)) return n;
  }
  return -1;
}

bool Mcp2515::send(const Frame& f) {
  if (_mode == Mode::ListenOnly || f.dlc > 8) return false;

  int n = freeTxBuffer();
  if (n < 0) {
    // Every buffer is still waiting for an ACK. Abort them so the bus is not wedged by a dead frame.
    bitModify(REG_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT);
    _spi.delayMs(1);
    bitModify(REG_CANCTRL, CANCTRL_ABAT, 0);
    _tx_aborted++;
    n = freeTxBuffer();
    if (n < 0) return false;
  }

  uint8_t buf[1 + 5 + 8];
  buf[0] = CMD_LOAD_TX | (uint8_t)(n << 1);
  if (f.extended) {
    buf[1] = (uint8_t)(f.id >> 21);
    buf[2] = (uint8_t)(((f.id >> 18) & 0x07) << 5) | SIDL_IDE | (uint8_t)((f.id >> 16) & 0x03);
    buf[3] = (uint8_t)(f.id >> 8);
    buf[4] = (uint8_t)f.id;
  } else {
    buf[1] = (uint8_t)(f.id >> 3);
    buf[2] = (uint8_t)((f.id & 0x07) << 5);
    buf[3] = buf[4] = 0;
  }
  buf[5] = (uint8_t)(f.dlc | (f.rtr ? DLC_RTR : 0));
  memcpy(&buf[6], f.data, f.dlc);
  _spi.transfer(buf, nullptr, 6 + f.dlc);

  uint8_t rts = CMD_RTS | (uint8_t)(1u << n);
  _spi.transfer(&rts, nullptr, 1);
  return true;
}

bool Mcp2515::receive(Frame& f) {
  uint8_t st_tx[2] = {CMD_READ_STATUS, 0}, st_rx[2] = {0, 0};
  _spi.transfer(st_tx, st_rx, 2);
  uint8_t n;
  if (st_rx[1] & 0x01) n = 0;
  else if (st_rx[1] & 0x02) n = 1;
  else return false;

  uint8_t tx[14] = {0}, rx[14] = {0};
  tx[0] = CMD_READ_RX | (uint8_t)(n << 2);
  _spi.transfer(tx, rx, 14);
  const uint8_t sidh = rx[1], sidl = rx[2], eid8 = rx[3], eid0 = rx[4], dlc = rx[5];

  f = Frame();
  f.extended = (sidl & SIDL_IDE) != 0;
  uint32_t base = ((uint32_t)sidh << 3) | (sidl >> 5);
  if (f.extended) {
    f.id = (base << 18) | ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    f.rtr = (dlc & DLC_RTR) != 0;
  } else {
    f.id = base;
    f.rtr = (sidl & SIDL_SRR) != 0;
  }
  f.dlc = (dlc & 0x0F) > 8 ? 8 : (dlc & 0x0F);
  if (!f.rtr) memcpy(f.data, &rx[6], f.dlc);
  return true;
}

void Mcp2515::poll() {
  uint8_t eflg = readReg(REG_EFLG);
  if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
    _rx_overruns++;
    bitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
  }
  if (readReg(REG_CANINTF) & CANINTF_ERRIF) {
    _bus_errors++;
    bitModify(REG_CANINTF, CANINTF_ERRIF, 0);
  }
}

bool Mcp2515::busOff() { return (readReg(REG_EFLG) & EFLG_TXBO) != 0; }
uint8_t Mcp2515::tec() { return readReg(REG_TEC); }
uint8_t Mcp2515::rec() { return readReg(REG_REC); }

}  // namespace can
}  // namespace roadnode
