#pragma once

#include <stddef.h>
#include <stdint.h>

// Local trip/data log (issue #52). A ring of small segment files with
// CRC-protected, versioned records. MeshCore-free; the filesystem sits behind
// LogFs so the whole format is host-tested, including power loss.
//
// Segment file:  "RL" ver flags first_seq(u32 LE)                    8 bytes
// Record:        len type seq(u32) time(u32) payload crc16           len = bytes after len
//   crc16 (CCITT) covers type..payload. type bit 7 set = time is unix seconds,
//   clear = seconds since boot (the BOOT record anchors those).
//
// Power loss: a record is one append, so an interrupted write leaves a short or
// bad tail. begin() never appends to an existing segment; it starts a new one, and
// readers stop at the first unparseable length and skip CRC-bad records.
// Overwrite policy: when full, the oldest segment is deleted first, uploaded or not.
namespace roadnode {
namespace storage {

constexpr uint8_t LOG_VERSION = 1;
constexpr size_t LOG_HEADER_BYTES = 8;
constexpr size_t LOG_FRAME_OVERHEAD = 12;  // len, type, seq, time, crc16
constexpr size_t LOG_MAX_PAYLOAD = 40;
constexpr size_t LOG_MAX_SEGMENTS = 128;

enum LogType : uint8_t {
  LOG_BOOT = 1,
  LOG_SAMPLE = 2,
  LOG_TRIP_END = 3,
  LOG_DTC = 4,
  LOG_UPLOADED = 5,  // payload: seq u32, everything up to it is confirmed received
};
constexpr uint8_t LOG_TIME_UNIX = 0x80;

struct LogRecord {
  uint8_t type = 0;
  bool time_unix = false;
  uint32_t seq = 0;
  uint32_t time = 0;
  uint8_t len = 0;
  uint8_t payload[LOG_MAX_PAYLOAD] = {0};
};

// Segment-file backend. Implementations: SpiffsLogFs (device), an in-memory fake in tests.
class LogFs {
public:
  virtual ~LogFs() {}
  // Fills ids (unordered) with segment ids present; returns false on failure.
  virtual bool list(uint32_t* ids, size_t max, size_t& n) = 0;
  virtual bool size(uint32_t id, size_t& bytes) = 0;
  // Appends to the segment, creating it if missing. All-or-error: on false,
  // part of the data may still have been written.
  virtual bool append(uint32_t id, const uint8_t* data, size_t len) = 0;
  virtual bool read(uint32_t id, size_t off, uint8_t* buf, size_t len, size_t& got) = 0;
  virtual bool remove(uint32_t id) = 0;
  // Free bytes on the shared filesystem (MeshCore lives there too).
  virtual size_t freeBytes() = 0;
};

struct TripLogConfig {
  size_t segment_bytes = 16 * 1024;
  size_t max_segments = 64;          // 64 x 16 KiB = 1 MiB budget
  size_t min_free_bytes = 256 * 1024;  // never starve MeshCore: shrink the log instead
};

struct TripLogStats {
  uint32_t appended = 0;
  uint32_t dropped = 0;          // append failed or no space
  uint32_t corrupt_skipped = 0;  // CRC-bad records skipped while reading
  uint32_t segments_removed = 0;
};

uint16_t logCrc16(const uint8_t* data, size_t len);

class TripLog {
public:
  TripLog(LogFs& fs, const TripLogConfig& cfg = TripLogConfig());

  // Scans existing segments: next sequence number and upload cursor. Safe on an empty or damaged log.
  bool begin();

  // Appends one record, assigning the next sequence number. False if it could not be stored.
  bool append(uint8_t type, bool time_unix, uint32_t time, const uint8_t* payload, size_t len);

  // Reads up to max records with seq > after_seq in ascending order, skipping
  // LOG_UPLOADED bookkeeping records. Returns the number read.
  size_t read(uint32_t after_seq, LogRecord* out, size_t max);

  // Records that everything up to seq has been received. Appends a LOG_UPLOADED record.
  bool markUploaded(uint32_t seq, bool time_unix, uint32_t time);

  uint32_t uploadedSeq() const { return _uploaded; }
  uint32_t lastSeq() const { return _next_seq - 1; }
  uint32_t nextSeq() const { return _next_seq; }
  const TripLogStats& stats() const { return _stats; }

private:
  struct SegInfo {
    uint32_t id = 0;
    uint32_t first_seq = 0;
    size_t bytes = 0;
  };
  size_t listSegments(uint32_t* ids);
  bool readHeader(uint32_t id, uint32_t& first_seq, size_t& bytes);
  // Visits valid records in order; visitor returns false to stop.
  template <class F>
  bool scanSegment(uint32_t id, size_t bytes, F&& visit);
  bool openNewSegment(uint32_t first_seq);
  bool makeRoom();

  LogFs& _fs;
  TripLogConfig _cfg;
  TripLogStats _stats;
  uint32_t _next_seq = 1;
  uint32_t _uploaded = 0;
  uint32_t _next_id = 1;
  bool _have_cur = false;
  uint32_t _cur_id = 0;
  size_t _cur_bytes = 0;
};

}  // namespace storage
}  // namespace roadnode
