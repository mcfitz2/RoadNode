#include "vehicle_storage.h"

#include <string.h>

namespace roadnode {
namespace storage {

static constexpr uint32_t MAGIC = 0x444F4E52;  // "RNOD" little-endian
static constexpr uint16_t VERSION = 1;

static void put16(uint8_t* p, uint16_t v) {
  p[0] = v;
  p[1] = v >> 8;
}
static void put32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = v >> (8 * i);
}
static void put64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; i++) p[i] = v >> (8 * i);
}
static uint16_t get16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static uint32_t get32(const uint8_t* p) {
  uint32_t v = 0;
  for (int i = 0; i < 4; i++) v |= (uint32_t)p[i] << (8 * i);
  return v;
}
static uint64_t get64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v |= (uint64_t)p[i] << (8 * i);
  return v;
}

uint32_t VehicleStorage::crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320 & (0 - (crc & 1)));
  }
  return ~crc;
}

// Layout (little-endian): magic[4] version[2] reserved[2] seq[4] total[8] trip[8]
//                         last_trip_ts[4] vehicle_id[16] crc32[4]
void VehicleStorage::encode(const VehicleRecord& rec, uint32_t seq, uint8_t out[ENCODED_SIZE]) {
  memset(out, 0, ENCODED_SIZE);
  put32(out + 0, MAGIC);
  put16(out + 4, VERSION);
  put32(out + 8, seq);
  put64(out + 12, rec.total_mm);
  put64(out + 20, rec.trip_mm);
  put32(out + 28, rec.last_trip_timestamp);
  memcpy(out + 32, rec.vehicle_id, sizeof(rec.vehicle_id));
  out[32 + sizeof(rec.vehicle_id) - 1] = 0;
  put32(out + 48, crc32(out, 48));
}

bool VehicleStorage::decode(const uint8_t in[ENCODED_SIZE], VehicleRecord& rec, uint32_t& seq) {
  if (get32(in + 0) != MAGIC || get16(in + 4) != VERSION) return false;
  if (get32(in + 48) != crc32(in, 48)) return false;
  seq = get32(in + 8);
  rec.total_mm = get64(in + 12);
  rec.trip_mm = get64(in + 20);
  rec.last_trip_timestamp = get32(in + 28);
  memcpy(rec.vehicle_id, in + 32, sizeof(rec.vehicle_id));
  rec.vehicle_id[sizeof(rec.vehicle_id) - 1] = 0;
  return true;
}

bool VehicleStorage::scan(VehicleRecord* out) {
  bool have = false;
  uint32_t best_seq = 0;
  uint8_t best_slot = 0;
  VehicleRecord best;

  for (uint8_t slot = 0; slot < 2; slot++) {
    uint8_t buf[ENCODED_SIZE];
    VehicleRecord r;
    uint32_t s;
    if (!_store.read(slot, buf, sizeof(buf)) || !decode(buf, r, s)) continue;
    // Wrap-safe "newer than".
    if (!have || (int32_t)(s - best_seq) > 0) {
      have = true;
      best = r;
      best_seq = s;
      best_slot = slot;
    }
  }

  _scanned = true;
  _seq = have ? best_seq : 0;
  _next_slot = have ? (uint8_t)(1 - best_slot) : 0;
  if (have && out) *out = best;
  return have;
}

bool VehicleStorage::load(VehicleRecord& rec) { return scan(&rec); }

bool VehicleStorage::save(const VehicleRecord& rec) {
  if (!_scanned) scan(nullptr);

  uint8_t buf[ENCODED_SIZE];
  encode(rec, _seq + 1, buf);
  if (!_store.write(_next_slot, buf, sizeof(buf))) return false;

  _seq++;
  _next_slot = 1 - _next_slot;
  return true;
}

}  // namespace storage
}  // namespace roadnode
