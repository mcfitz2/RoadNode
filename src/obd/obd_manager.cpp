#include "obd_manager.h"

#include <string.h>

namespace roadnode {
namespace obd {

const char* statusName(Status s) {
  switch (s) {
    case Status::Ok: return "ok";
    case Status::Disabled: return "disabled";
    case Status::Unsupported: return "unsupported";
    case Status::Timeout: return "timeout";
    case Status::NegativeResponse: return "negative-response";
    case Status::Malformed: return "malformed";
    case Status::BusError: return "bus-error";
    case Status::Forbidden: return "forbidden";
  }
  return "?";
}

bool modeAllowed(uint8_t mode) {
  return mode == 0x01 || mode == 0x03 || mode == 0x07 || mode == 0x09 || mode == 0x0A;
}

bool ObdManager::accepts(uint32_t id) const {
  if (_profile->response_id) return id == _profile->response_id;
  return id >= 0x7E8 && id <= 0x7EF;
}

// Receives one complete ISO-TP message (single or multi-frame) that answers
// (mode, pid; pid < 0 means the mode has no PID byte) or is a negative response to mode. Total time bounded by timeout_ms.
Status ObdManager::receiveMessage(uint8_t mode, int pid, uint8_t* msg, size_t& len,
                                  uint8_t& nrc, uint32_t& source) {
  uint32_t start = _bus.nowMs();
  auto remaining = [&]() -> uint32_t {
    uint32_t el = _bus.nowMs() - start;
    return el >= _profile->timeout_ms ? 0 : _profile->timeout_ms - el;
  };

  size_t total = 0, got = 0;
  uint8_t next_seq = 1;
  bool assembling = false;
  uint32_t src = 0;

  for (;;) {
    uint32_t rem = remaining();
    if (rem == 0) return Status::Timeout;
    CanFrame f;
    if (!_bus.receive(f, rem)) return Status::Timeout;
    _frames_seen++;
    _last_frame_ms = _bus.nowMs();
    if (!accepts(f.id) || f.dlc < 2) continue;
    if (assembling && f.id != src) continue;

    uint8_t type = f.data[0] >> 4;
    if (!assembling) {
      if (type == 0) {  // single frame
        size_t n = f.data[0] & 0x0F;
        if (n < 2 || n > 7 || n + 1 > f.dlc) return Status::Malformed;
        // negative response: 7F mode NRC
        if (f.data[1] == 0x7F && n >= 3 && f.data[2] == mode) {
          if (f.data[3] == 0x78) continue;  // response pending
          nrc = f.data[3];
          source = f.id;
          return Status::NegativeResponse;
        }
        if (f.data[1] != mode + 0x40 || (pid >= 0 && n >= 3 && f.data[2] != pid)) continue;
        memcpy(msg, f.data + 1, n);
        len = n;
        source = f.id;
        return Status::Ok;
      }
      if (type == 1) {  // first frame
        total = ((size_t)(f.data[0] & 0x0F) << 8) | f.data[1];
        if (total < 3 || total > Response::MAX + 2) return Status::Malformed;
        if (f.data[2] != mode + 0x40 || (pid >= 0 && f.data[3] != pid)) continue;
        src = f.id;
        memcpy(msg, f.data + 2, 6);
        got = 6;
        assembling = true;
        // flow control to the physical request ID: ECU response IDs are request + 8
        CanFrame fc;
        fc.id = src - 8;
        fc.dlc = 8;
        memset(fc.data, 0x55, 8);
        fc.data[0] = 0x30;  // continue to send
        fc.data[1] = 0;     // no block limit
        fc.data[2] = 0;     // no separation time
        if (!_bus.send(fc)) return Status::BusError;
        continue;
      }
      continue;  // stray consecutive/flow frame
    }

    if (type != 2) continue;
    if ((f.data[0] & 0x0F) != (next_seq & 0x0F)) return Status::Malformed;
    next_seq++;
    size_t n = total - got;
    if (n > 7) n = 7;
    if (n + 1 > f.dlc) return Status::Malformed;
    memcpy(msg + got, f.data + 1, n);
    got += n;
    if (got >= total) {
      len = total;
      source = src;
      return Status::Ok;
    }
  }
}

Status ObdManager::request(uint8_t mode, uint8_t pid, Response& out) {
  out.len = 0;
  if (!modeAllowed(mode)) return Status::Forbidden;
  if (!_enabled) return Status::Disabled;

  CanFrame req;
  req.id = _profile->request_id;
  req.dlc = 8;
  memset(req.data, 0x55, 8);
  req.data[0] = 2;
  req.data[1] = mode;
  req.data[2] = pid;
  if (!_bus.send(req)) return Status::BusError;

  uint8_t msg[Response::MAX + 2];
  size_t len = 0;
  Status st = receiveMessage(mode, pid, msg, len, out.nrc, out.source_id);
  if (st != Status::Ok) return st;
  if (len < 2) return Status::Malformed;
  // msg = [mode+0x40, pid, data...]
  out.len = len - 2;
  memcpy(out.data, msg + 2, out.len);
  return Status::Ok;
}

Status ObdManager::requestMode(uint8_t mode, Response& out) {
  out.len = 0;
  if (!modeAllowed(mode)) return Status::Forbidden;
  if (!_enabled) return Status::Disabled;

  CanFrame req;
  req.id = _profile->request_id;
  req.dlc = 8;
  memset(req.data, 0x55, 8);
  req.data[0] = 1;
  req.data[1] = mode;
  if (!_bus.send(req)) return Status::BusError;

  uint8_t msg[Response::MAX + 2];
  size_t len = 0;
  Status st = receiveMessage(mode, -1, msg, len, out.nrc, out.source_id);
  if (st != Status::Ok) return st;
  if (len < 1) return Status::Malformed;
  // msg = [mode+0x40, data...]
  out.len = len - 1;
  if (out.len > Response::MAX) return Status::Malformed;
  memcpy(out.data, msg + 1, out.len);
  return Status::Ok;
}

Status ObdManager::readDtcs(DtcMode mode, DtcList& out) {
  out.count = 0;
  Response r;
  Status st = requestMode((uint8_t)mode, r);
  if (st == Status::NegativeResponse && (r.nrc == 0x11 || r.nrc == 0x12)) return Status::Unsupported;
  if (st != Status::Ok) return st;
  return decodeDtcs(r.data, r.len, out) ? Status::Ok : Status::Malformed;
}

Status ObdManager::readPid(uint8_t pid, float& value) {
  if (_caps.rangeKnown(pid) && !_caps.supported(pid)) return Status::Unsupported;
  Response r;
  Status st = request(0x01, pid, r);
  if (st != Status::Ok) return st;
  return decodePid(pid, r.data, (uint8_t)r.len, value) ? Status::Ok : Status::Malformed;
}

Status ObdManager::discover() {
  _caps.clear();
  bool any = false;
  for (uint8_t base = 0x00; base <= 0xC0; base += 0x20) {
    Response r;
    Status st = request(0x01, base, r);
    if (st != Status::Ok) return any ? Status::Ok : st;
    if (!_caps.load(base, r.data, (uint8_t)r.len)) return any ? Status::Ok : Status::Malformed;
    any = true;
    if (!_caps.hasNextRange(base)) {
      _caps.markRemainingUnsupported(base);
      break;
    }
  }
  return Status::Ok;
}

Status ObdManager::readVin(char vin[18]) {
  Response r;
  Status st = request(0x09, 0x02, r);
  if (st != Status::Ok) return st;
  // Payload is [count] + 17 ASCII, possibly with the count byte missing.
  if (r.len < 17) return Status::Malformed;
  const uint8_t* p = r.data + (r.len - 17);
  for (int i = 0; i < 17; i++) {
    if (p[i] < 0x20 || p[i] > 0x7E) return Status::Malformed;
    vin[i] = (char)p[i];
  }
  vin[17] = 0;
  return Status::Ok;
}

static int vinValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  static const int vals[] = {1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 0, 7, 0, 9, 2, 3, 4, 5, 6, 7, 8, 9};
  if (c >= 'A' && c <= 'Z') return vals[c - 'A'];
  return -1;
}

bool vinWellFormed(const char* vin) {
  if (!vin || strlen(vin) != 17) return false;
  for (int i = 0; i < 17; i++) {
    char c = vin[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
    if (!ok || c == 'I' || c == 'O' || c == 'Q') return false;
  }
  return true;
}

bool vinValid(const char* vin) {
  if (!vinWellFormed(vin)) return false;
  static const int w[17] = {8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2};
  int sum = 0;
  for (int i = 0; i < 17; i++) sum += vinValue(vin[i]) * w[i];
  char expect = (sum % 11 == 10) ? 'X' : (char)('0' + sum % 11);
  return vin[8] == expect;
}

}  // namespace obd
}  // namespace roadnode
