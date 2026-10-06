#ifdef ARDUINO

#include "vehicle_runtime.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can/can_manager.h"
#include "can/obd_can_bus.h"
#include "mileage_tracker.h"
#include "obd/obd_manager.h"
#include "storage/nvs_slot_store.h"
#include "vehicle_poller.h"

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

namespace roadnode {
namespace vehicle {

namespace {

can::ObdCanBus s_bus;
storage::NvsSlotStore s_store;
MileageTracker s_tracker(s_store);
VehicleTelemetry s_telemetry;
obd::ObdManager s_obd(s_bus, obd::genericProfile());
VehiclePoller s_poller(s_obd, s_bus, s_tracker, s_telemetry);

bool s_started = false;
bool s_can_ok = false;

void pollTask(void*) {
  for (;;) {
    s_poller.step(millis());
    vTaskDelay(pdMS_TO_TICKS(s_obd.profile().poll_interval_ms));
  }
}

}  // namespace

bool VehicleRuntime::begin(const char* vehicle_id, bool transmit) {
  if (s_started) return s_can_ok;
  s_started = true;

  can::Config cfg;
  cfg.bitrate_bps = CAN_BITRATE;
  cfg.mode = transmit ? can::Mode::Normal : can::Mode::ListenOnly;
  can::Result r = can::begin(cfg);
  s_can_ok = (r == can::Result::Ok);
  if (!s_can_ok) Serial.printf("# CAN begin failed: %s (esp_err %d)\n", can::resultName(r), can::lastError());
  s_obd.enableTransmit(transmit && s_can_ok);
  Serial.println(transmit ? "# OBD transmit ENABLED" : "# OBD transmit disabled (listen-only)");

  s_tracker.setVehicleId(vehicle_id);
  bool restored = s_tracker.begin(millis());
  Serial.printf("# odometer %s: %llu mm\n", restored ? "restored" : "fresh", (unsigned long long)s_tracker.totalMm());

  xTaskCreatePinnedToCore(pollTask, "roadnode_obd", 6144, nullptr, 1, nullptr, 0);
  return s_can_ok;
}

const VehicleTelemetry& VehicleRuntime::telemetry() { return s_telemetry; }

bool VehicleRuntime::shutdown() { return s_tracker.shutdown(millis()); }

bool VehicleRuntime::canStarted() { return s_can_ok; }

}  // namespace vehicle
}  // namespace roadnode

#endif  // ARDUINO
