#pragma once

#include <stddef.h>

// Small string key/value store for settings (vehicle id, VIN). Implementations:
// NvsKvStore (device), an in-memory fake in host tests.
namespace roadnode {
namespace storage {

class KvStore {
public:
  virtual ~KvStore() {}
  // Copies the stored string (NUL terminated) into out. False if missing or it does not fit.
  virtual bool getString(const char* key, char* out, size_t out_size) = 0;
  virtual bool putString(const char* key, const char* value) = 0;
  virtual bool remove(const char* key) = 0;
};

}  // namespace storage
}  // namespace roadnode
