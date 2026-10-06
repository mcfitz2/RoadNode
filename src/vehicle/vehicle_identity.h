#pragma once

#include <stdint.h>

#include "storage/kv_store.h"

// Short vehicle ID (goes on the mesh) and VIN (stays on the device) -
// issues #18 and #34. The VIN is deliberately kept out of VehicleSnapshot,
// telemetry and alerts: the only way to read it is vin(), for local use.
namespace roadnode {
namespace vehicle {

class VehicleIdentity {
public:
  explicit VehicleIdentity(storage::KvStore& kv) : _kv(kv) {}

  // Loads the persisted ID and VIN. A persisted ID wins over default_id (the
  // build-time value); default_id is only the first-boot fallback and is not
  // written back, so changing the build flag later still applies until set at runtime.
  void begin(const char* default_id);

  // Current short ID. Safe to call from another task than setId().
  void copyId(char out[16]) const;
  const char* id() const { return _id; }  // single-task use only

  // Valid: 1-15 characters from A-Z 0-9 '-' '_' (lowercase is uppercased).
  static bool validId(const char* id);
  // Validates, persists, then applies. False if invalid or the write failed (ID unchanged).
  bool setId(const char* id);

  enum class VinResult : uint8_t {
    Stored,       // first VIN read, now persisted
    Unchanged,    // matches the stored VIN
    Mismatch,     // differs from the stored VIN: stored one kept, vinMismatch() set
    Invalid,      // not 17 characters of the VIN alphabet
    WriteFailed,  // persisting failed: not remembered, will be retried on the next read
  };
  VinResult onVinRead(const char* vin);

  bool hasVin() const { return _vin[0] != 0; }
  const char* vin() const { return _vin; }
  // Check digit is only mandatory in North America, so a failing digit is recorded, not rejected.
  bool vinCheckDigitOk() const { return _vin_check_ok; }
  // True if the ECU reported a different VIN than the stored one this boot (vehicle swap?).
  bool vinMismatch() const { return _vin_mismatch; }
  // Forget the stored VIN so the next read is accepted (use when the unit moves to another vehicle).
  bool clearVin();

private:
  storage::KvStore& _kv;
  char _id[16] = {0};
  uint32_t _ver = 0;  // odd while _id is being written (seqlock)
  char _vin[18] = {0};
  bool _vin_check_ok = false;
  bool _vin_mismatch = false;
};

}  // namespace vehicle
}  // namespace roadnode
