#include "trip_log.h"

#include <string.h>

namespace roadnode {
namespace storage {

uint16_t logCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void put32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

TripLog::TripLog(LogFs& fs, const TripLogConfig& cfg) : _fs(fs), _cfg(cfg) {
  if (_cfg.max_segments > LOG_MAX_SEGMENTS) _cfg.max_segments = LOG_MAX_SEGMENTS;
  if (_cfg.max_segments < 2) _cfg.max_segments = 2;
  if (_cfg.segment_bytes < LOG_HEADER_BYTES + LOG_FRAME_OVERHEAD + LOG_MAX_PAYLOAD)
    _cfg.segment_bytes = LOG_HEADER_BYTES + LOG_FRAME_OVERHEAD + LOG_MAX_PAYLOAD;
}

size_t TripLog::listSegments(uint32_t* ids) {
  size_t n = 0;
  if (!_fs.list(ids, LOG_MAX_SEGMENTS, n)) return 0;
  for (size_t i = 1; i < n; i++) {  // ascending
    uint32_t v = ids[i];
    size_t j = i;
    while (j > 0 && ids[j - 1] > v) { ids[j] = ids[j - 1]; j--; }
    ids[j] = v;
  }
  return n;
}

bool TripLog::readHeader(uint32_t id, uint32_t& first_seq, size_t& bytes) {
  if (!_fs.size(id, bytes) || bytes < LOG_HEADER_BYTES) return false;
  uint8_t h[LOG_HEADER_BYTES];
  size_t got = 0;
  if (!_fs.read(id, 0, h, sizeof(h), got) || got != sizeof(h)) return false;
  if (h[0] != 'R' || h[1] != 'L' || h[2] != LOG_VERSION) return false;
  first_seq = get32(h + 4);
  return true;
}

template <class F>
bool TripLog::scanSegment(uint32_t id, size_t bytes, F&& visit) {
  size_t pos = LOG_HEADER_BYTES;
  uint32_t last_seq = 0;
  bool any = false;
  while (pos + 1 <= bytes) {
    uint8_t len_b;
    size_t got = 0;
    if (!_fs.read(id, pos, &len_b, 1, got) || got != 1) break;
    size_t len = len_b;
    // Implausible length or a frame running past the end: truncated tail or garbage.
    if (len < LOG_FRAME_OVERHEAD - 1 || len > LOG_FRAME_OVERHEAD - 1 + LOG_MAX_PAYLOAD || pos + 1 + len > bytes) break;
    uint8_t f[LOG_FRAME_OVERHEAD + LOG_MAX_PAYLOAD];
    if (!_fs.read(id, pos + 1, f, len, got) || got != len) break;
    pos += 1 + len;
    uint16_t want = (uint16_t)(f[len - 2] | (f[len - 1] << 8));
    if (logCrc16(f, len - 2) != want) { _stats.corrupt_skipped++; continue; }
    LogRecord r;
    r.type = f[0] & 0x7F;
    r.time_unix = (f[0] & LOG_TIME_UNIX) != 0;
    r.seq = get32(f + 1);
    r.time = get32(f + 5);
    r.len = (uint8_t)(len - (LOG_FRAME_OVERHEAD - 1));
    memcpy(r.payload, f + 9, r.len);
    if (any && r.seq <= last_seq) { _stats.corrupt_skipped++; continue; }  // non-monotonic: not trustworthy
    last_seq = r.seq;
    any = true;
    if (!visit(r)) return false;
  }
  return true;
}

bool TripLog::begin() {
  _have_cur = false;
  _next_seq = 1;
  _uploaded = 0;
  _next_id = 1;

  uint32_t ids[LOG_MAX_SEGMENTS];
  size_t n = listSegments(ids);
  if (n == 0) return true;
  _next_id = ids[n - 1] + 1;

  // Newest valid segment decides the next sequence number.
  bool seq_found = false;
  for (size_t i = n; i-- > 0 && !seq_found;) {
    uint32_t first;
    size_t bytes;
    if (!readHeader(ids[i], first, bytes)) {
      if (i == n - 1 && bytes < LOG_HEADER_BYTES) _fs.remove(ids[i]);  // crashed before the header landed
      continue;
    }
    uint32_t last = first - 1;
    scanSegment(ids[i], bytes, [&](const LogRecord& r) { last = r.seq; return true; });
    _next_seq = last + 1;
    seq_found = true;
  }

  // Upload cursor: newest LOG_UPLOADED record wins; search newest segment first.
  for (size_t i = n; i-- > 0;) {
    uint32_t first;
    size_t bytes;
    if (!readHeader(ids[i], first, bytes)) continue;
    uint32_t best = 0;
    scanSegment(ids[i], bytes, [&](const LogRecord& r) {
      if (r.type == LOG_UPLOADED && r.len >= 4) best = get32(r.payload);
      return true;
    });
    if (best) { _uploaded = best; break; }
  }
  return true;
}

bool TripLog::makeRoom() {
  uint32_t ids[LOG_MAX_SEGMENTS];
  size_t n = listSegments(ids);
  size_t i = 0;
  // Cap by count, then by free space on the shared filesystem. Oldest first.
  while (i < n && (n - i >= _cfg.max_segments || _fs.freeBytes() < _cfg.min_free_bytes + _cfg.segment_bytes)) {
    if (!_fs.remove(ids[i])) return false;
    _stats.segments_removed++;
    i++;
  }
  return _fs.freeBytes() >= _cfg.min_free_bytes + _cfg.segment_bytes;
}

bool TripLog::openNewSegment(uint32_t first_seq) {
  _have_cur = false;
  if (!makeRoom()) return false;
  uint8_t h[LOG_HEADER_BYTES] = {'R', 'L', LOG_VERSION, 0};
  put32(h + 4, first_seq);
  uint32_t id = _next_id;
  if (!_fs.append(id, h, sizeof(h))) return false;
  _next_id++;
  _cur_id = id;
  _cur_bytes = sizeof(h);
  _have_cur = true;
  return true;
}

bool TripLog::append(uint8_t type, bool time_unix, uint32_t time, const uint8_t* payload, size_t len) {
  if (len > LOG_MAX_PAYLOAD) { _stats.dropped++; return false; }
  uint8_t f[1 + LOG_FRAME_OVERHEAD + LOG_MAX_PAYLOAD];
  size_t flen = LOG_FRAME_OVERHEAD - 1 + len;  // bytes after the length byte
  f[0] = (uint8_t)flen;
  f[1] = (uint8_t)((type & 0x7F) | (time_unix ? LOG_TIME_UNIX : 0));
  put32(f + 2, _next_seq);
  put32(f + 6, time);
  if (len) memcpy(f + 10, payload, len);
  uint16_t crc = logCrc16(f + 1, flen - 2);
  f[1 + flen - 2] = (uint8_t)crc;
  f[1 + flen - 1] = (uint8_t)(crc >> 8);
  size_t total = 1 + flen;

  if (!_have_cur || _cur_bytes + total > _cfg.segment_bytes) {
    if (!openNewSegment(_next_seq)) { _stats.dropped++; return false; }
  }
  if (!_fs.append(_cur_id, f, total)) {
    _have_cur = false;  // tail may be damaged: next record starts a fresh segment
    _stats.dropped++;
    return false;
  }
  _cur_bytes += total;
  _next_seq++;
  _stats.appended++;
  return true;
}

size_t TripLog::read(uint32_t after_seq, LogRecord* out, size_t max) {
  uint32_t ids[LOG_MAX_SEGMENTS];
  size_t n = listSegments(ids);
  size_t count = 0;
  for (size_t i = 0; i < n && count < max; i++) {
    uint32_t first;
    size_t bytes;
    if (!readHeader(ids[i], first, bytes)) continue;
    // Whole segment is older than the cursor if the next one starts at or before after_seq+1.
    if (i + 1 < n) {
      uint32_t next_first;
      size_t nb;
      if (readHeader(ids[i + 1], next_first, nb) && next_first <= after_seq + 1) continue;
    }
    scanSegment(ids[i], bytes, [&](const LogRecord& r) {
      if (r.seq <= after_seq || r.type == LOG_UPLOADED) return true;
      out[count++] = r;
      return count < max;
    });
  }
  return count;
}

bool TripLog::markUploaded(uint32_t seq, bool time_unix, uint32_t time) {
  if (seq <= _uploaded) return true;
  uint8_t p[4];
  put32(p, seq);
  if (!append(LOG_UPLOADED, time_unix, time, p, sizeof(p))) return false;
  _uploaded = seq;
  return true;
}

}  // namespace storage
}  // namespace roadnode
