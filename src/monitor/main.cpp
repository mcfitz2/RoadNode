// OBD vehicle monitor: CAN + OBD + mileage + persistence, serial output only.
// Independent of MeshCore, so mileage keeps running whether or not telemetry exists.
//
// Build flags:
//   CAN_BITRATE            bits per second (default 500000)
//   OBD_TRANSMIT_ENABLE    explicit opt-in to transmit OBD requests. Without it
//                          the controller stays listen-only and nothing is sent.
//   VEHICLE_ID             short id stored with the odometer (default "UNSET")
#include <Arduino.h>

#include "vehicle/vehicle_runtime.h"

using namespace roadnode;

#ifndef VEHICLE_ID
#define VEHICLE_ID "UNSET"
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);
#ifdef OBD_TRANSMIT_ENABLE
  vehicle::VehicleRuntime::begin(VEHICLE_ID, true);
#else
  vehicle::VehicleRuntime::begin(VEHICLE_ID, false);
#endif
}

void loop() {
  vehicle::VehicleSnapshot s = vehicle::VehicleRuntime::telemetry().snapshot();
  Serial.printf("t=%lu active=%d conn=%d eng=%d spd=%s%.0f rpm=%s%.0f batt=%s%.1f total=%.3fkm trip=%.3fkm\n",
                (unsigned long)millis(), s.vehicle_active, s.obd_connected, s.engine_running, s.has_speed ? "" : "?",
                s.speed_kmh, s.has_rpm ? "" : "?", s.rpm, s.has_battery ? "" : "?", s.battery_v,
                s.total_mm / 1e6, s.trip_mm / 1e6);
  delay(1000);
}
