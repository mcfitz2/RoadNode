// Derived from MeshCore (MIT, see THIRD_PARTY_NOTICES.md).
// RoadNode copy of vendor/MeshCore/examples/simple_sensor/main.cpp. Differences from
// the vendor file are limited to the alert logic in MyMesh (engine start / parked
// alerts carrying position, also posted to an optional private channel) and the
// parked deep sleep (PROJ-596). Re-diff against the vendor file on every MeshCore bump.
#include "SensorMesh.h"
#include "vehicle/alert_channel.h"
#include "vehicle/alert_logic.h"
#include "storage/nvs_kv_store.h"
#include "vehicle/alert_settings.h"
#include "vehicle/identity_commands.h"
#include "vehicle/odometer_commands.h"
#include "vehicle/sleep_policy.h"
#include "vehicle/vehicle_runtime.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#ifdef PIN_CAR_ON
#include <driver/rtc_io.h>

// Parked deep sleep. The carrier's CAR_ON line (open drain, 1M pull-up to its
// always-on 3.3 V rail) goes high while the engine runs or the hybrid is Ready;
// ext0 wakes on that and the node boots fresh. Checkpoints the odometer first.
// Mirrors ESP32Board::enterDeepSleep/HeltecV4Board::powerOff, which can't be used
// as they clear every wake source.
static bool carOn() { return digitalRead(PIN_CAR_ON) == HIGH; }

static void enterParkedSleep() {
  bool saved = roadnode::vehicle::VehicleRuntime::shutdown();
  Serial.printf("Parked: odometer %s, deep sleep until CAR_ON\n", saved ? "saved" : "NOT saved");
  radio_driver.powerOff();
  digitalWrite(P_LORA_NSS, HIGH);
  rtc_gpio_hold_en((gpio_num_t)P_LORA_NSS);
  digitalWrite(P_LORA_PA_POWER, LOW);
  rtc_gpio_hold_en((gpio_num_t)P_LORA_PA_POWER);
  if (sensors.getLocationProvider() != NULL) sensors.getLocationProvider()->stop();
  Serial.flush();
  delay(100);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  rtc_gpio_pullup_dis((gpio_num_t)PIN_CAR_ON);   // the carrier pulls up
  rtc_gpio_pulldown_dis((gpio_num_t)PIN_CAR_ON);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_CAR_ON, 1);
  esp_deep_sleep_start();  // never returns; wake is a fresh boot
}
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

  void loadVehicleSettings() {
    roadnode::vehicle::loadAlertSettings(settings_kv, vehicle_alerts);
    roadnode::vehicle::loadAlertChannel(settings_kv, alert_channel);
  }

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  roadnode::storage::NvsKvStore settings_kv;
  Trigger vehicle_started, vehicle_parked, vehicle_periodic;
  roadnode::vehicle::AlertLogic vehicle_alerts = roadnode::vehicle::AlertLogic(alertConfig());
  roadnode::vehicle::AlertChannel alert_channel;
  bool was_low_batt = false, was_critical_batt = false;
  bool was_started = false, was_parked = false, was_periodic = false;
#ifdef PIN_CAR_ON
  roadnode::vehicle::SleepPolicy sleep_policy;
#endif

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

  // One group text per alert edge, flooded like a stock channel message ("<name>: <text>"),
  // so every client holding the channel secret sees it. Not retried and not ACKed.
  void postToChannel(const char* text) {
    if (!alert_channel.enabled) return;
    mesh::GroupChannel ch;
    memset(ch.secret, 0, sizeof(ch.secret));
    memcpy(ch.secret, alert_channel.secret, alert_channel.len);
    mesh::Utils::sha256(ch.hash, sizeof(ch.hash), ch.secret, alert_channel.len);

    NodePrefs* prefs = getNodePrefs();
    uint8_t data[5 + sizeof(prefs->node_name) + 2 + 96];
    uint32_t ts = getRTCClock()->getCurrentTimeUnique();
    memcpy(data, &ts, 4);
    data[4] = 0;  // TXT_TYPE_PLAIN
    int len = snprintf((char*)&data[5], sizeof(data) - 5, "%s: %s", prefs->node_name, text);
    if (len < 0) return;
    if (len > (int)sizeof(data) - 6) len = sizeof(data) - 6;
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, ch, data, 5 + len);
    if (pkt) sendFlood(pkt, (uint32_t)0, prefs->path_hash_mode + 1);
  }

  // Direct alerts to opted-in ACL clients (stock), plus the channel post on the rising edge.
  void notify(bool condition, bool& was, Trigger& t, AlertPriority pri, const char* text) {
    if (condition && !was) postToChannel(text);
    was = condition;
    alertIf(condition, t, pri, text);
  }

  TimeSeriesData  battery_data;

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    notify(batt_voltage < 3.4f, was_critical_batt, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    notify(batt_voltage < 3.6f, was_low_batt, low_batt, LOW_PRI_ALERT, "Battery is low");

    // Trip-end timestamp: only once GPS has set the RTC (a stock RTC holds an arbitrary value).
#if ENV_INCLUDE_GPS
    if (sensors.clockFromGps()) roadnode::vehicle::VehicleRuntime::setTime(getRTCClock()->getCurrentTime());
#endif

    roadnode::vehicle::VehicleSnapshot snap = roadnode::vehicle::VehicleRuntime::telemetry().snapshot();
    roadnode::vehicle::AlertConditions c = vehicle_alerts.update(snap, millis());
    char text[64];
    positionText(text, sizeof(text), "Vehicle started");
    notify(c.started, was_started, vehicle_started, LOW_PRI_ALERT, text);
    positionText(text, sizeof(text), "Vehicle parked");
    notify(c.parked, was_parked, vehicle_parked, HIGH_PRI_ALERT, text);
    positionText(text, sizeof(text), "Vehicle moving");
    notify(c.periodic, was_periodic, vehicle_periodic, LOW_PRI_ALERT, text);

#ifdef PIN_CAR_ON
    // Runs once per sensor read (60 s), so sleep comes up to a minute after the grace period.
    bool operating = snap.trip_active || snap.engine_running;
    if (sleep_policy.update(operating, c.parked, carOn(), millis())) enterParkedSleep();
#endif
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
    if (roadnode::vehicle::handleAlertCommand(settings_kv, vehicle_alerts, command, reply))
      return true;
    if (roadnode::vehicle::handleAlertChannelCommand(settings_kv, alert_channel, command, reply))
      return true;
    if (roadnode::vehicle::handleOdometerCommand(roadnode::vehicle::VehicleRuntime::odometer(), command, reply))
      return true;
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
#ifdef PIN_CAR_ON
  // Undo the holds from the last parked sleep before the radio and PA are driven again.
  rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
  rtc_gpio_hold_dis((gpio_num_t)P_LORA_PA_POWER);
  pinMode(PIN_CAR_ON, INPUT);
#endif

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

#ifdef PIN_CAR_ON
  Serial.printf("Wake: %s, CAR_ON %s\n", esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 ? "CAR_ON" : "power-on/reset",
                carOn() ? "high" : "low");
#endif
  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);
  the_mesh.loadVehicleSettings();

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

#ifdef PIN_CAR_ON
  static int car_on_seen = -1;
  if ((int)carOn() != car_on_seen) {
    car_on_seen = carOn();
    Serial.printf("CAR_ON %s\n", car_on_seen ? "high" : "low");
  }
#endif
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
