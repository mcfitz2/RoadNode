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

// What the vehicle itself declares, printed once discovery has answered. Without this a car
// test can only infer support from which values never arrive, which cannot tell "the ECU says
// no" from "the request is failing". Named PIDs are called out; 0xA6 matters because a readable
// vehicle odometer would remove the need to track one by hand.
static void dumpCapabilitiesOnce() {
  static bool done = false;
  if (done) return;
  const obd::CapabilityTable& caps = vehicle::VehicleRuntime::capabilities();
  if (!caps.rangeKnown(0x0C)) return;  // discovery has not answered yet
  done = true;
  struct Named {
    uint8_t pid;
    const char* name;
  };
  static const Named named[] = {
      {0x04, "load"},     {0x05, "coolant"},  {0x0C, "rpm"},        {0x0D, "speed"},
      {0x0F, "intake"},   {0x2F, "fuel"},     {0x42, "module_v"},   {0x46, "ambient"},
      {0xA6, "odometer"},
  };
  Serial.print("# caps named:");
  for (const Named& n : named)
    Serial.printf(" %s(%02X)=%s", n.name, n.pid,
                  !caps.rangeKnown(n.pid) ? "unknown" : (caps.supported(n.pid) ? "yes" : "no"));
  Serial.println();
  Serial.print("# caps supported pids:");
  for (uint16_t pid = 0x01; pid <= 0xE0; pid++)
    if (caps.rangeKnown((uint8_t)pid) && caps.supported((uint8_t)pid)) Serial.printf(" %02X", pid);
  Serial.println();
}

void loop() {
  dumpCapabilitiesOnce();
  vehicle::VehicleSnapshot s = vehicle::VehicleRuntime::telemetry().snapshot();
  // The slow PIDs are printed too, so a car test can check them against a scan tool (PROJ-576)
  // without the mesh firmware and an app. "?" means the value has never been read.
  Serial.printf("t=%lu active=%d conn=%d eng=%d spd=%s%.0f rpm=%s%.0f batt=%s%.1f "
                "cool=%s%.0fC load=%s%.0f%% iat=%s%.0fC fuel=%s%.0f%% total=%.3fkm trip=%.3fkm\n",
                (unsigned long)millis(), s.vehicle_active, s.obd_connected, s.engine_running, s.has_speed ? "" : "?",
                s.speed_kmh, s.has_rpm ? "" : "?", s.rpm, s.has_battery ? "" : "?", s.battery_v,
                s.has_coolant ? "" : "?", s.coolant_c, s.has_load ? "" : "?", s.load_pct, s.has_intake ? "" : "?",
                s.intake_c, s.has_fuel ? "" : "?", s.fuel_pct, s.total_mm / 1e6, s.trip_mm / 1e6);
  delay(1000);
}
