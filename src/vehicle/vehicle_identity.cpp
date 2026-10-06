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
    uint32_t v1 = _ver;
    if (v1 & 1) continue;
    memcpy(out, _id, 16);
    if (v1 == _ver) return;
  }
}

bool VehicleIdentity::setId(const char* id) {
  if (!validId(id)) return false;
  char u[16];
  upper(u, id);
  if (!_kv.putString(KEY_ID, u)) return false;
  _ver++;  // odd: writing
  memset(_id, 0, sizeof(_id));
  memcpy(_id, u, strlen(u));
  _ver++;
  return true;
}

VehicleIdentity::VinResult VehicleIdentity::onVinRead(const char* vin) {
  if (!obd::vinWellFormed(vin)) return VinResult::Invalid;
  if (hasVin()) {
    if (strcmp(_vin, vin) == 0) return VinResult::Unchanged;
    _vin_mismatch = true;
    return VinResult::Mismatch;
  }
  if (!_kv.putString(KEY_VIN, vin)) return VinResult::WriteFailed;
  memcpy(_vin, vin, sizeof(_vin));
  _vin_check_ok = obd::vinValid(_vin);
  _vin_mismatch = false;
  return VinResult::Stored;
}

bool VehicleIdentity::clearVin() {
  if (!_kv.remove(KEY_VIN)) return false;
  memset(_vin, 0, sizeof(_vin));
  _vin_check_ok = false;
  _vin_mismatch = false;
  return true;
}

}  // namespace vehicle
}  // namespace roadnode
