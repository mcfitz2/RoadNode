#pragma once

#include <stddef.h>
#include <stdint.h>

// Persistent vehicle record with power-loss-safe A/B slots (plan section 12).
//
// Each save writes a CRC-protected record to the slot that does NOT hold the
// newest valid record, so an interrupted write can only damage the older copy.
// Load picks the valid record with the highest sequence number.
namespace roadnode {
namespace storage {

struct VehicleRecord {
  uint64_t total_mm = 0;
  uint64_t trip_mm = 0;
  uint32_t last_trip_timestamp = 0;  // unix seconds; 0 if unknown
  char vehicle_id[16] = {0};         // short ID (e.g. "RAV4"), NUL-terminated
};

// Two fixed storage slots. Implementations: NvsSlotStore (device), fakes in tests.
class SlotStore {
public:
  virtual ~SlotStore() {}
  // Returns true only if exactly len bytes were read.
  virtual bool read(uint8_t slot, uint8_t* buf, size_t len) = 0;
  virtual bool write(uint8_t slot, const uint8_t* buf, size_t len) = 0;
};

class VehicleStorage {
public:
  static constexpr size_t ENCODED_SIZE = 52;

  explicit VehicleStorage(SlotStore& store) : _store(store) {}

  // Returns false if neither slot holds a valid record (fresh device); rec is untouched.
  bool load(VehicleRecord& rec);

  // Writes rec to the older slot. Returns false if the write failed; the
  // previous record is still intact in that case.
  bool save(const VehicleRecord& rec);

  uint32_t sequence() const { return _seq; }

  // Exposed for tests.
  static void encode(const VehicleRecord& rec, uint32_t seq, uint8_t out[ENCODED_SIZE]);
  static bool decode(const uint8_t in[ENCODED_SIZE], VehicleRecord& rec, uint32_t& seq);
  static uint32_t crc32(const uint8_t* data, size_t len);

private:
  bool scan(VehicleRecord* rec);

  SlotStore& _store;
  bool _scanned = false;
  uint32_t _seq = 0;     // sequence of the newest valid record (0 if none)
  uint8_t _next_slot = 0;  // slot the next save() writes to
};

}  // namespace storage
}  // namespace roadnode
