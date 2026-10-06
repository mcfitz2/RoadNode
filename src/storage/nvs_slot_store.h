#pragma once

#include "vehicle_storage.h"

#ifdef ARDUINO

namespace roadnode {
namespace storage {

// SlotStore backed by ESP32 NVS (Preferences). Slots are blobs "slot0"/"slot1"
// in the "roadnode" namespace. NVS wear-levels across its partition.
class NvsSlotStore : public SlotStore {
public:
  bool read(uint8_t slot, uint8_t* buf, size_t len) override;
  bool write(uint8_t slot, const uint8_t* buf, size_t len) override;
};

}  // namespace storage
}  // namespace roadnode

#endif  // ARDUINO
