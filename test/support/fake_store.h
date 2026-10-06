#pragma once

#include <string.h>

#include "storage/vehicle_storage.h"

// In-memory two-slot store with fault injection, shared by host tests.
class FakeStore : public roadnode::storage::SlotStore {
public:
  static constexpr size_t N = roadnode::storage::VehicleStorage::ENCODED_SIZE;
  uint8_t data[2][N];
  bool present[2] = {false, false};
  bool fail_writes = false;   // every write fails without touching the slot
  int writes = 0;             // successful writes

  bool read(uint8_t slot, uint8_t* buf, size_t len) override {
    if (!present[slot] || len != N) return false;
    memcpy(buf, data[slot], len);
    return true;
  }
  bool write(uint8_t slot, const uint8_t* buf, size_t len) override {
    if (fail_writes) return false;
    memcpy(data[slot], buf, len);
    present[slot] = true;
    writes++;
    return true;
  }
};
