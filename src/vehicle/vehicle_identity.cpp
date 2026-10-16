#include "vehicle_identity.h"

#include <string.h>

#include "obd/obd_manager.h"

namespace roadnode {
namespace vehicle {

static const char* KEY_ID = "vid";
static const char* KEY_VIN = "vin";

bool VehicleIdentity::validId(const char* id) {
  if (!id) return false;
  size_t n = strlen(id);
  if (n < 1 || n > 15) return false;
  for (size_t i = 0; i < n; i++) {
    char c = id[i];
    bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
    if (!ok) return false;
  }
  return true;
}

static void upper(char* dst, const char* src) {
  size_t i = 0;
  for (; src[i]; i++) dst[i] = (src[i] >= 'a' && src[i] <= 'z') ? (char)(src[i] - 32) : src[i];
  dst[i] = 0;
}

void VehicleIdentity::begin(const char* default_id) {
  char buf[16];
  if (_kv.getString(KEY_ID, buf, sizeof(buf)) && validId(buf)) {
    upper(_id, buf);
  } else if (validId(default_id)) {
    upper(_id, default_id);
  } else {
    strcpy(_id, "UNSET");
  }

  char vin[18];
  _vin[0] = 0;
  if (_kv.getString(KEY_VIN, vin, sizeof(vin)) && obd::vinWellFormed(vin)) {
    memcpy(_vin, vin, sizeof(_vin));
    _vin_check_ok = obd::vinValid(_vin);
  }
}

void VehicleIdentity::copyId(char out[16]) const {
  for (;;) {
    uint32_t v1 = _ver.load(std::memory_order_acquire);
    if (v1 & 1) continue;
    memcpy(out, _id, 16);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (v1 == _ver.load(std::memory_order_relaxed)) return;
  }
}

bool VehicleIdentity::setId(const char* id) {
  if (!validId(id)) return false;
  char u[16];
  upper(u, id);
  if (!_kv.putString(KEY_ID, u)) return false;
  uint32_t v = _ver.load(std::memory_order_relaxed);
  _ver.store(v + 1, std::memory_order_relaxed);  // odd: writing
  std::atomic_thread_fence(std::memory_order_release);
  memset(_id, 0, sizeof(_id));
  memcpy(_id, u, strlen(u));
  _ver.store(v + 2, std::memory_order_release);
  return true;
}

bool VehicleIdentity::hasVin() const {
  lockVin();
  bool has = _vin[0] != 0;
  unlockVin();
  return has;
}

void VehicleIdentity::copyVin(char out[18]) const {
  lockVin();
  memcpy(out, _vin, 18);
  unlockVin();
}

VehicleIdentity::VinResult VehicleIdentity::onVinRead(const char* vin) {
  if (!obd::vinWellFormed(vin)) return VinResult::Invalid;
  lockVin();
  VinResult r;
  if (_vin[0] != 0) {
    r = strcmp(_vin, vin) == 0 ? VinResult::Unchanged : VinResult::Mismatch;
    if (r == VinResult::Mismatch) _vin_mismatch = true;
  } else if (!_kv.putString(KEY_VIN, vin)) {
    r = VinResult::WriteFailed;
  } else {
    memcpy(_vin, vin, sizeof(_vin));
    _vin_check_ok = obd::vinValid(_vin);
    _vin_mismatch = false;
    r = VinResult::Stored;
  }
  unlockVin();
  return r;
}

bool VehicleIdentity::clearVin() {
  lockVin();
  bool ok = _kv.remove(KEY_VIN);
  if (ok) {
    memset(_vin, 0, sizeof(_vin));
    _vin_check_ok = false;
    _vin_mismatch = false;
    _vin_gen.fetch_add(1, std::memory_order_release);
  }
  unlockVin();
  return ok;
}

}  // namespace vehicle
}  // namespace roadnode
