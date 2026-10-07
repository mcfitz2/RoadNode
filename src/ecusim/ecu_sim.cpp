#include "ecu_sim.h"

#include <initializer_list>
#include <string.h>

namespace roadnode {
namespace ecusim {

using obd::CanFrame;

float EcuSim::driveSpeedKmh(uint32_t cycle_ms) {
  uint32_t t = cycle_ms % CYCLE_MS;
  if (t < 10000) return 0;
  if (t < 30000) return 60.0f * (t - 10000) / 20000.0f;
  if (t < 90000) return 60.0f;
  if (t < 110000) return 60.0f * (110000 - t) / 20000.0f;
  return 0;
}

void EcuSim::update(uint32_t now_ms) {
  uint32_t dt = _have_last ? now_ms - _last_ms : 0;
  _last_ms = now_ms;
  _have_last = true;
  if (!_sc.drive_cycle) {
    _cycle_ms = 0;
    return;
  }
  _cycle_ms += dt;
  _sc.speed_kmh = driveSpeedKmh(_cycle_ms);
  _sc.rpm = 800 + _sc.speed_kmh * 30;
  _distance_m += (double)_sc.speed_kmh / 3.6 * dt / 1000.0;
}

void EcuSim::push(const CanFrame& f) {
  if (_qn >= QUEUE) return;  // never happens with one request in flight; drop rather than overwrite
  _q[(_qh + _qn) % QUEUE] = f;
  _qn++;
}

bool EcuSim::nextFrame(CanFrame& out) {
  if (_qn == 0) return false;
  out = _q[_qh];
  _qh = (_qh + 1) % QUEUE;
  _qn--;
  return true;
}

void EcuSim::onFrame(const CanFrame& f) {
  if (f.dlc < 2) return;
  uint8_t type = f.data[0] >> 4;
  if (f.id == PHYSICAL_REQUEST_ID && type == 3) {  // flow control: clear to send
    if (_awaiting_fc && (f.data[0] & 0x0F) == 0) sendConsecutive();
    return;
  }
  if (f.id != FUNCTIONAL_REQUEST_ID && f.id != PHYSICAL_REQUEST_ID) return;
  if (type != 0) return;  // requests are single frames
  uint8_t n = f.data[0] & 0x0F;
  if (n < 1 || n > 7 || n + 1 > f.dlc) return;
  _requests++;
  _awaiting_fc = false;  // a new request abandons any unfinished response
  handleRequest(f.data + 1, n);
}

void EcuSim::handleRequest(const uint8_t* d, uint8_t n) {
  uint8_t mode = d[0];
  bool known = mode == 0x01 || mode == 0x03 || mode == 0x07 || mode == 0x09 || mode == 0x0A;
  if (!known) {
    _forbidden++;
    _last_forbidden = mode;
  }
  if (_sc.silent) return;
  if (mode < 32 && (_sc.ignore_modes & (1u << mode))) return;
  if (!known) return negative(mode, 0x11);  // service not supported
  switch (mode) {
    case 0x01:
      if (n < 2) return;
      return mode01(d[1]);
    case 0x03: return dtcMode(mode, _sc.stored);
    case 0x07: return dtcMode(mode, _sc.pending);
    case 0x0A: return dtcMode(mode, _sc.permanent);
    case 0x09:
      if (n < 2) return;
      return mode09(d[1]);
  }
}

void EcuSim::negative(uint8_t mode, uint8_t nrc) {
  const uint8_t p[3] = {0x7F, mode, nrc};
  respond(p, 3);
}

// Bit for `pid` in the range answered by `base`: bit 31 is base+1.
static void setBit(uint8_t mask[4], uint8_t base, uint8_t pid) {
  uint8_t i = pid - base - 1;
  mask[i / 8] |= 0x80 >> (i % 8);
}

static uint8_t tempByte(float c) { return (uint8_t)(c + 40.5f); }
static uint8_t pctByte(float p) { return (uint8_t)(p * 255.0f / 100.0f + 0.5f); }

void EcuSim::mode01(uint8_t pid) {
  uint8_t p[8];
  p[0] = 0x41;
  p[1] = pid;
  switch (pid) {
    case 0x00:
    case 0x20:
    case 0x40: {
      uint8_t m[4] = {0, 0, 0, 0};
      if (pid == 0x00) {
        for (uint8_t q : {0x04, 0x05, 0x0C, 0x0D, 0x0F, 0x20}) setBit(m, 0x00, q);
      } else if (pid == 0x20) {
        for (uint8_t q : {0x2F, 0x40}) setBit(m, 0x20, q);
      } else {
        setBit(m, 0x40, 0x42);  // no 0x60: discovery stops here
      }
      memcpy(p + 2, m, 4);
      return respond(p, 6);
    }
    case 0x04: p[2] = pctByte(_sc.load_pct); return respond(p, 3);
    case 0x05: p[2] = tempByte(_sc.coolant_c); return respond(p, 3);
    case 0x0C: {
      uint16_t r = (uint16_t)(_sc.rpm * 4.0f);
      p[2] = r >> 8;
      p[3] = r & 0xFF;
      return respond(p, 4);
    }
    case 0x0D: p[2] = (uint8_t)(_sc.speed_kmh + 0.5f); return respond(p, 3);
    case 0x0F: p[2] = tempByte(_sc.intake_c); return respond(p, 3);
    case 0x2F: p[2] = pctByte(_sc.fuel_pct); return respond(p, 3);
    case 0x42: {
      uint16_t mv = (uint16_t)(_sc.module_volts * 1000.0f);
      p[2] = mv >> 8;
      p[3] = mv & 0xFF;
      return respond(p, 4);
    }
    default: return negative(0x01, 0x12);  // sub-function not supported
  }
}

void EcuSim::dtcMode(uint8_t mode, const DtcSet& s) {
  uint8_t p[2 + 2 * MAX_DTCS];
  p[0] = mode + 0x40;
  p[1] = (uint8_t)s.count;
  for (size_t i = 0; i < s.count; i++) {
    p[2 + 2 * i] = s.raw[i] >> 8;
    p[3 + 2 * i] = s.raw[i] & 0xFF;
  }
  respond(p, 2 + 2 * s.count);
}

void EcuSim::mode09(uint8_t pid) {
  if (pid == 0x00) {
    uint8_t p[6] = {0x49, 0x00, 0, 0, 0, 0};
    if (_sc.vin_supported) setBit(p + 2, 0x00, 0x02);
    return respond(p, 6);
  }
  if (pid == 0x02 && _sc.vin_supported) {
    uint8_t p[20] = {0x49, 0x02, 0x01};
    memcpy(p + 3, _sc.vin, 17);
    return respond(p, 20);
  }
  negative(0x09, 0x12);
}

void EcuSim::respond(const uint8_t* payload, size_t len) {
  CanFrame f;
  f.id = RESPONSE_ID;
  f.dlc = 8;
  memset(f.data, 0x55, 8);
  if (len <= 7) {
    f.data[0] = (uint8_t)len;
    memcpy(f.data + 1, payload, len);
    return push(f);
  }
  if (len > MAX_PAYLOAD) return;
  f.data[0] = 0x10 | (uint8_t)(len >> 8);
  f.data[1] = (uint8_t)len;
  memcpy(f.data + 2, payload, 6);
  push(f);
  memcpy(_pend, payload, len);
  _pend_len = len;
  _pend_sent = 6;
  _pend_seq = 1;
  _awaiting_fc = true;
}

void EcuSim::sendConsecutive() {
  while (_pend_sent < _pend_len) {
    CanFrame f;
    f.id = RESPONSE_ID;
    f.dlc = 8;
    memset(f.data, 0x55, 8);
    f.data[0] = 0x20 | (_pend_seq & 0x0F);
    size_t n = _pend_len - _pend_sent;
    if (n > 7) n = 7;
    memcpy(f.data + 1, _pend + _pend_sent, n);
    push(f);
    _pend_sent += n;
    _pend_seq++;
  }
  _awaiting_fc = false;
}

}  // namespace ecusim
}  // namespace roadnode
