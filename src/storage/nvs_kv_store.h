#pragma once

#include "kv_store.h"

#ifdef ARDUINO

namespace roadnode {
namespace storage {

// KvStore backed by ESP32 NVS (Preferences), namespace "roadnode_cfg".
class NvsKvStore : public KvStore {
public:
  bool getString(const char* key, char* out, size_t out_size) override;
  bool putString(const char* key, const char* value) override;
  bool remove(const char* key) override;
};

}  // namespace storage
}  // namespace roadnode

#endif  // ARDUINO
