#include "nvs_kv_store.h"

#ifdef ARDUINO

#include <Preferences.h>
#include <string.h>

namespace roadnode {
namespace storage {

static const char* NAMESPACE = "roadnode_cfg";

bool NvsKvStore::getString(const char* key, char* out, size_t out_size) {
  Preferences p;
  if (!p.begin(NAMESPACE, true)) return false;
  bool ok = false;
  if (p.isKey(key)) {
    String s = p.getString(key, "");
    if (s.length() > 0 && s.length() < out_size) {
      memcpy(out, s.c_str(), s.length() + 1);
      ok = true;
    }
  }
  p.end();
  return ok;
}

bool NvsKvStore::putString(const char* key, const char* value) {
  Preferences p;
  if (!p.begin(NAMESPACE, false)) return false;
  size_t n = strlen(value);
  bool ok = p.putString(key, value) == n;
  p.end();
  return ok;
}

bool NvsKvStore::remove(const char* key) {
  Preferences p;
  if (!p.begin(NAMESPACE, false)) return false;
  bool ok = !p.isKey(key) || p.remove(key);
  p.end();
  return ok;
}

}  // namespace storage
}  // namespace roadnode

#endif  // ARDUINO
