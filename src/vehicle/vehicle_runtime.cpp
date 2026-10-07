#ifdef ARDUINO

#include "vehicle_runtime.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "can/can_manager.h"
#include "can/obd_can_bus.h"
#include "mileage_tracker.h"
#include "obd/obd_manager.h"
#include "storage/nvs_kv_store.h"
#include "storage/nvs_slot_store.h"
#include "vehicle_identity.h"
#include "vehicle_poller.h"

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

namespace roadnode {
namespace vehicle {

namespace {

can::ObdCanBus s_bus;
storage::NvsSlotStore s_store;
storage::NvsKvStore s_kv;
VehicleIdentity s_identity(s_kv);
MileageTracker s_tracker(s_store);
VehicleTelemetry s_telemetry;
obd::ObdManager s_obd(s_bus, obd::genericProfile());
VehiclePoller s_poller(s_obd, s_bus, s_tracker, s_telemetry);

SemaphoreHandle_t s_lock = nullptr;  // serialises poller step() with shutdown() (different tasks)
bool s_started = false;
bool s_can_ok = false;

void pollTask(void*) {
  for (;;) {
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
      s_poller.step(millis());
      xSemaphoreGive(s_lock);
    }
    vTaskDelay(pdMS_TO_TICKS(s_obd.profile().poll_interval_ms));
  }
}

}  // namespace

bool VehicleRuntime::begin(const char* vehicle_id, bool transmit) {
  if (s_started) return s_can_ok;
  s_started = true;
  s_lock = xSemaphoreCreateMutex();

  can::Config cfg;
  cfg.bitrate_bps = CAN_BITRATE;
  cfg.mode = transmit ? can::Mode::Normal : can::Mode::ListenOnly;
  can::Result r = can::begin(cfg);
  s_can_ok = (r == can::Result::Ok);
  if (!s_can_ok) Serial.printf("# CAN begin failed: %s (esp_err %d)\n", can::resultName(r), can::lastError());
  s_obd.enableTransmit(transmit && s_can_ok);
  Serial.println(transmit ? "# OBD transmit ENABLED" : "# OBD transmit disabled (listen-only)");

  s_identity.begin(vehicle_id);
  Serial.printf("# vehicle id: %s%s\n", s_identity.id(), s_identity.hasVin() ? " (VIN stored locally)" : "");
  s_poller.setIdentity(&s_identity);
  s_tracker.setVehicleId(s_identity.id());
  bool restored = s_tracker.begin(millis());
  Serial.printf("# odometer %s: %llu mm\n", restored ? "restored" : "fresh", (unsigned long long)s_tracker.totalMm());

  xTaskCreatePinnedToCore(pollTask, "roadnode_obd", 6144, nullptr, 1, nullptr, 0);
  return s_can_ok;
}

const VehicleTelemetry& VehicleRuntime::telemetry() { return s_telemetry; }

// Called from the MeshCore task before a reboot or power off. Waits for a poll step in progress
// (a DTC read can block for a few request timeouts); if it does not finish in time, skips the
// save rather than write the tracker from two tasks.
bool VehicleRuntime::shutdown() {
  if (!s_started || !s_lock) return false;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
  bool ok = s_tracker.shutdown(millis());
  xSemaphoreGive(s_lock);  // reboot follows; the poll task may resume until then, which is harmless
  return ok;
}

// A plain aligned 32-bit store: the poll task reads it only when a trip ends.
void VehicleRuntime::setTime(uint32_t unix_seconds) {
  if (s_started) s_tracker.setTime(unix_seconds);
}

bool VehicleRuntime::addGpsDistance(uint64_t mm, uint32_t interval_start_ms) {
  if (!s_started || !s_lock) return false;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  s_tracker.addGpsDistance(millis(), mm, interval_start_ms);  // ignored (consumed) if OBD was valid in the interval
  xSemaphoreGive(s_lock);
  return true;
}

namespace {
class RuntimeOdometer : public OdometerControl {
public:
  bool read(uint64_t& total_mm, uint64_t& gps_mm) override {
    if (!s_started || !s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    total_mm = s_tracker.totalMm();
    gps_mm = s_tracker.gpsFilledMm();
    xSemaphoreGive(s_lock);
    return true;
  }
  bool setTotalMm(uint64_t total_mm) override {
    if (!s_started || !s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    uint64_t old = s_tracker.totalMm();
    bool ok = s_tracker.setTotalMm(millis(), total_mm);
    xSemaphoreGive(s_lock);
    Serial.printf("# odometer resync %s: %llu mm -> %llu mm\n", ok ? "saved" : "FAILED", (unsigned long long)old,
                  (unsigned long long)total_mm);
    return ok;
  }
};
RuntimeOdometer s_odometer;
}  // namespace

OdometerControl& VehicleRuntime::odometer() { return s_odometer; }

VehicleIdentity& VehicleRuntime::identity() { return s_identity; }

bool VehicleRuntime::canStarted() { return s_can_ok; }

}  // namespace vehicle
}  // namespace roadnode

#endif  // ARDUINO
