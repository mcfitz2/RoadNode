// RoadNode copy of vendor/MeshCore/examples/simple_sensor/main.cpp. Differences from
// the vendor file are limited to the alert logic in MyMesh (engine start / parked
// alerts carrying position). Re-diff against the vendor file on every MeshCore bump.
#include "SensorMesh.h"
#include "obd/dtc.h"
#include "vehicle/alert_logic.h"
#include "vehicle/identity_commands.h"
#include "vehicle/vehicle_runtime.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  Trigger vehicle_started, vehicle_parked, vehicle_periodic, vehicle_dtc;
  roadnode::vehicle::AlertLogic vehicle_alerts = roadnode::vehicle::AlertLogic(alertConfig());

  static roadnode::vehicle::AlertConfig alertConfig() {
    roadnode::vehicle::AlertConfig cfg;
#ifdef PERIODIC_ALERT_MINUTES
    cfg.periodic_interval_ms = (uint32_t)PERIODIC_ALERT_MINUTES * 60u * 1000u;
#endif
    return cfg;
  }

  // Alerts go only to ACL clients that opted in (PERM_RECV_ALERTS_*), encrypted.
  // node_lat/lon hold the last valid GPS fix; 0,0 means no fix since boot.
  void positionText(char* out, size_t n, const char* what) {
    if (sensors.node_lat == 0 && sensors.node_lon == 0) {
      snprintf(out, n, "%s (no GPS fix)", what);
    } else {
      snprintf(out, n, "%s at %.5f,%.5f", what, sensors.node_lat, sensors.node_lon);
    }
  }
  void dtcText(char* out, size_t n, const roadnode::vehicle::VehicleSnapshot& s) {
    size_t used = snprintf(out, n, "Vehicle DTC:");
    char code[6];
    for (uint8_t i = 0; i < s.dtc_count && i < 6 && used < n; i++) {
      roadnode::obd::formatDtc((uint8_t)(s.dtc_raw[i] >> 8), (uint8_t)s.dtc_raw[i], code);
      used += snprintf(out + used, n - used, " %s", code);
    }
    if (s.dtc_total > 6 && used < n) snprintf(out + used, n - used, " +%u", (unsigned)(s.dtc_total - 6));
  }

  TimeSeriesData  battery_data;

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");

        roadnode::vehicle::VehicleSnapshot snap = roadnode::vehicle::VehicleRuntime::telemetry().snapshot();
    roadnode::vehicle::AlertConditions c = vehicle_alerts.update(snap, millis());
    char text[64];
    positionText(text, sizeof(text), "Vehicle started");
    alertIf(c.started, vehicle_started, LOW_PRI_ALERT, text);
    positionText(text, sizeof(text), "Vehicle parked");
    alertIf(c.parked, vehicle_parked, HIGH_PRI_ALERT, text);
    positionText(text, sizeof(text), "Vehicle moving");
    alertIf(c.periodic, vehicle_periodic, LOW_PRI_ALERT, text);

    // New trouble code: list the current codes (first 6) so the message is self-contained.
    char dtc_text[96];
    dtcText(dtc_text, sizeof(dtc_text), snap);
    alertIf(c.dtc_new, vehicle_dtc, HIGH_PRI_ALERT, dtc_text);
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "magic") == 0) {    // example 'custom' command handling
      strcpy(reply, "**Magic now done**");
      return true;   // handled
    }
    if (roadnode::vehicle::handleIdentityCommand(roadnode::vehicle::VehicleRuntime::identity(), command, reply))
      return true;
    return false;  // not handled
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif
}
