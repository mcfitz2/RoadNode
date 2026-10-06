#pragma once

#include <map>
#include <string.h>
#include <string>

#include "storage/kv_store.h"

// In-memory KvStore with write fault injection, shared by host tests.
class FakeKv : public roadnode::storage::KvStore {
public:
  std::map<std::string, std::string> data;
  bool fail_writes = false;
  int writes = 0;

  bool getString(const char* key, char* out, size_t out_size) override {
    auto it = data.find(key);
    if (it == data.end() || it->second.size() + 1 > out_size) return false;
    memcpy(out, it->second.c_str(), it->second.size() + 1);
    return true;
  }
  bool putString(const char* key, const char* value) override {
    if (fail_writes) return false;
    data[key] = value;
    writes++;
    return true;
  }
  bool remove(const char* key) override {
    if (fail_writes) return false;
    data.erase(key);
    return true;
  }
};
