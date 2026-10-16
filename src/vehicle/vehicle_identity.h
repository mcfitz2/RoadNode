#pragma once

#include <atomic>
#include <stdint.h>

#ifdef ARDUINO
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
#include <mutex>
#endif

#include "storage/kv_store.h"

// Short vehicle ID and VIN - issues #18 and #34. The VIN goes into
// VehicleSnapshot and the encrypted, ACL-only telemetry reply (PROJ-682,
// reversing #34's device-only rule); it is never put in adverts or alerts.
namespace roadnode {
namespace vehicle {

class VehicleIdentity {
public:
  explicit VehicleIdentity(storage::KvStore& kv) : _kv(kv) {
#ifdef ARDUINO
    _vin_mutex = xSemaphoreCreateMutex();
#endif
  }

  // Loads the persisted ID and VIN. A persisted ID wins over default_id (the
  // build-time value); default_id is only the first-boot fallback and is not
  // written back, so changing the build flag later still applies until set at runtime.
  void begin(const char* default_id);

  // Current short ID. Safe to call from another task than setId() (seqlock with acquire/release
  // ordering). Writers must not run concurrently with each other: setId() is only called from the admin command path.
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
  // The VIN is written from the poll task (onVinRead) and the admin command task (clearVin), so
  // these and the accessors marked safe take a lock.
  VinResult onVinRead(const char* vin);

  bool hasVin() const;  // safe from any task
  void copyVin(char out[18]) const;  // safe from any task; empty if not read
  const char* vin() const { return _vin; }  // single-task use only
  // Check digit is only mandatory in North America, so a failing digit is recorded, not rejected.
  bool vinCheckDigitOk() const { return _vin_check_ok; }
  // True if the ECU reported a different VIN than the stored one this boot (vehicle swap?).
  bool vinMismatch() const { return _vin_mismatch; }
  // Forget the stored VIN so the next read is accepted (use when the unit moves to another vehicle).
  // Bumps vinGeneration(), which tells the poller to read the VIN again without a reboot.
  bool clearVin();
  uint32_t vinGeneration() const { return _vin_gen.load(std::memory_order_acquire); }

private:
  storage::KvStore& _kv;
  char _id[16] = {0};
  std::atomic<uint32_t> _ver{0};  // odd while _id is being written (seqlock); one writer at a time
  char _vin[18] = {0};
  bool _vin_check_ok = false;
  bool _vin_mismatch = false;
  std::atomic<uint32_t> _vin_gen{0};
#ifdef ARDUINO
  void lockVin() const { xSemaphoreTake(_vin_mutex, portMAX_DELAY); }
  void unlockVin() const { xSemaphoreGive(_vin_mutex); }
  SemaphoreHandle_t _vin_mutex;
#else
  void lockVin() const { _vin_mutex.lock(); }
  void unlockVin() const { _vin_mutex.unlock(); }
  mutable std::mutex _vin_mutex;
#endif
};

}  // namespace vehicle
}  // namespace roadnode
