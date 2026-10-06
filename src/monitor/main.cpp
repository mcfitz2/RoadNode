// OBD vehicle monitor: CAN + OBD + mileage + persistence, serial output only.
// Independent of MeshCore, so mileage keeps running whether or not telemetry exists.
//
// Build flags:
//   CAN_BITRATE            bits per second (default 500000)
//   OBD_TRANSMIT_ENABLE    explicit opt-in to transmit OBD requests. Without it
//                          the controller stays listen-only and nothing is sent.
//   VEHICLE_ID             short id stored with the odometer (default "UNSET")
#include <Arduino.h>

#include "can/can_manager.h"
#include "can/obd_can_bus.h"
#include "obd/obd_manager.h"
#include "storage/nvs_slot_store.h"
#include "vehicle/mileage_tracker.h"
#include "vehicle/vehicle_poller.h"
#include "vehicle/vehicle_state.h"

using namespace roadnode;

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif
#ifndef VEHICLE_ID
#define VEHICLE_ID "UNSET"
#endif

static can::ObdCanBus bus;
static storage::NvsSlotStore store;
static vehicle::MileageTracker tracker(store);
static vehicle::VehicleTelemetry telemetry;
static obd::ObdManager obdMgr(bus, obd::genericProfile());
static vehicle::VehiclePoller poller(obdMgr, bus, tracker, telemetry);
static uint32_t next_poll = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  can::Config cfg;
  cfg.bitrate_bps = CAN_BITRATE;
#ifdef OBD_TRANSMIT_ENABLE
  cfg.mode = can::Mode::Normal;
  obdMgr.enableTransmit(true);
  Serial.println("# OBD transmit ENABLED");
#else
  cfg.mode = can::Mode::ListenOnly;
  Serial.println("# OBD transmit disabled (listen-only); define OBD_TRANSMIT_ENABLE to poll");
#endif
  can::Result r = can::begin(cfg);
  if (r != can::Result::Ok) {
    Serial.printf("# CAN begin failed: %s (esp_err %d)\n", can::resultName(r), can::lastError());
  }

  tracker.setVehicleId(VEHICLE_ID);
  bool restored = tracker.begin(millis());
  Serial.printf("# odometer %s: %llu mm\n", restored ? "restored" : "fresh", (unsigned long long)tracker.totalMm());
}

void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - next_poll) < 0) {
    delay(5);
    return;
  }
  next_poll = now + obdMgr.profile().poll_interval_ms;

  poller.step(now);

  vehicle::VehicleSnapshot s = telemetry.snapshot();
  Serial.printf("t=%lu active=%d conn=%d eng=%d spd=%s%.0f rpm=%s%.0f batt=%s%.1f total=%.3fkm trip=%.3fkm%s\n",
                (unsigned long)now, s.vehicle_active, s.obd_connected, s.engine_running, s.has_speed ? "" : "?",
                s.speed_kmh, s.has_rpm ? "" : "?", s.rpm, s.has_battery ? "" : "?", s.battery_v,
                s.total_mm / 1e6, s.trip_mm / 1e6, tracker.savePending() ? " SAVE-PENDING" : "");
}
