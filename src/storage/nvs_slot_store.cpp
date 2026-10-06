#include "nvs_slot_store.h"

#ifdef ARDUINO

#include <Preferences.h>

namespace roadnode {
namespace storage {

static const char* NAMESPACE = "roadnode";

static const char* keyFor(uint8_t slot) { return slot == 0 ? "slot0" : "slot1"; }

bool NvsSlotStore::read(uint8_t slot, uint8_t* buf, size_t len) {
  Preferences p;
  if (!p.begin(NAMESPACE, true)) return false;
  bool ok = p.getBytesLength(keyFor(slot)) == len && p.getBytes(keyFor(slot), buf, len) == len;
  p.end();
  return ok;
}

bool NvsSlotStore::write(uint8_t slot, const uint8_t* buf, size_t len) {
  Preferences p;
  if (!p.begin(NAMESPACE, false)) return false;
  bool ok = p.putBytes(keyFor(slot), buf, len) == len;
  p.end();
  return ok;
}

}  // namespace storage
}  // namespace roadnode

#endif  // ARDUINO
