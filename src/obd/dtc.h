#pragma once

#include <stddef.h>
#include <stdint.h>

// Diagnostic trouble codes (plan section 18). Read-only: Mode 04 (clear) is
// deliberately not implemented anywhere in RoadNode.
namespace roadnode {
namespace obd {

enum class DtcMode : uint8_t {
  Stored = 0x03,     // confirmed
  Pending = 0x07,    // seen this or last drive cycle, not yet confirmed
  Permanent = 0x0A,  // cannot be cleared by a scan tool
};

struct Dtc {
  char code[6] = {0};  // e.g. "P0301", NUL terminated
};

struct DtcList {
  static constexpr size_t MAX = 31;  // what fits in one Response (63 bytes after the mode byte)
  Dtc codes[MAX];
  size_t count = 0;
};

// Formats one 2-byte DTC: P/C/B/U, then four hex digits (e.g. 0x03 0x01 -> P0301).
void formatDtc(uint8_t hi, uint8_t lo, char out[6]);

// Decodes a response payload (bytes after the 43/47/4A mode byte). CAN responses
// start with a count byte, which is detected by odd length; an even length is
// treated as bare code pairs. 0000 padding entries are skipped.
// Returns false if the length is inconsistent with the count byte.
bool decodeDtcs(const uint8_t* data, size_t len, DtcList& out);

}  // namespace obd
}  // namespace roadnode
