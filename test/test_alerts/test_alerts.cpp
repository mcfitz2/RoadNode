#include <initializer_list>
#include <unity.h>

#include "vehicle/alert_logic.h"

using namespace roadnode::vehicle;

// `active` is "the bus is awake and the ECU is answering": both flags move together on a
// healthy vehicle. Tests that need them to disagree (a chattering gateway, a mid-drive dropout)
// set obd_connected themselves.
static VehicleSnapshot snap(bool engine, bool active) {
  VehicleSnapshot s;
  s.engine_running = engine;
  s.vehicle_active = active;
  s.obd_connected = active;
  return s;
}

void setUp() {}
void tearDown() {}

void test_boot_with_quiet_bus_is_not_parked() {
  AlertLogic a;
  AlertConditions c = a.update(snap(false, false), 0);
  TEST_ASSERT_FALSE(c.started);
  TEST_ASSERT_FALSE(c.parked);
}

void test_engine_on_is_started_not_parked() {
  AlertLogic a;
  AlertConditions c = a.update(snap(true, true), 0);
  TEST_ASSERT_TRUE(c.started);
  TEST_ASSERT_FALSE(c.parked);
}

void test_parked_after_drive_once_bus_quiet() {
  AlertLogic a;
  a.update(snap(true, true), 0);
  AlertConditions idle = a.update(snap(false, true), 0);  // engine off, bus still awake
  TEST_ASSERT_FALSE(idle.parked);
  AlertConditions quiet = a.update(snap(false, false), 0);
  TEST_ASSERT_TRUE(quiet.parked);
  TEST_ASSERT_FALSE(quiet.started);
}

void test_restart_clears_parked_and_rearms() {
  AlertLogic a;
  a.update(snap(true, true), 0);
  TEST_ASSERT_TRUE(a.update(snap(false, false), 0).parked);
  AlertConditions c = a.update(snap(true, true), 0);
  TEST_ASSERT_TRUE(c.started);
  TEST_ASSERT_FALSE(c.parked);
  TEST_ASSERT_TRUE(a.update(snap(false, false), 0).parked);
}

static VehicleSnapshot moving(float kmh) {
  VehicleSnapshot s = snap(true, true);
  s.has_speed = true;
  s.speed_kmh = kmh;
  return s;
}

static const uint32_t MIN = 60u * 1000u;

static AlertLogic logic15() {
  AlertConfig cfg;
  cfg.periodic_interval_ms = 15 * MIN;
  cfg.periodic_hold_ms = 5 * MIN;
  return AlertLogic(cfg);
}

void test_defaults_every_five_minutes_and_rearm() {
  AlertLogic a;
  a.update(moving(80), 0);
  TEST_ASSERT_FALSE(a.update(moving(80), 4 * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(moving(80), 5 * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(moving(80), 6 * MIN).periodic);
  TEST_ASSERT_FALSE(a.update(moving(80), 7 * MIN + 30000).periodic);  // released before next due
  TEST_ASSERT_TRUE(a.update(moving(80), 10 * MIN).periodic);
}

void test_hold_clamped_below_interval() {
  AlertConfig cfg;
  cfg.periodic_interval_ms = 2 * MIN;
  cfg.periodic_hold_ms = 5 * MIN;
  AlertLogic a(cfg);
  a.update(moving(80), 0);
  TEST_ASSERT_TRUE(a.update(moving(80), 2 * MIN).periodic);
  TEST_ASSERT_FALSE(a.update(moving(80), 3 * MIN).periodic);  // hold clamped to 1 min
}

void test_periodic_first_report_one_interval_after_start() {
  AlertLogic a = logic15();
  TEST_ASSERT_FALSE(a.update(moving(80), 0).periodic);
  TEST_ASSERT_FALSE(a.update(moving(80), 14 * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(moving(80), 15 * MIN).periodic);
}

void test_periodic_held_for_window_then_released_then_repeats() {
  AlertLogic a = logic15();
  a.update(moving(80), 0);
  TEST_ASSERT_TRUE(a.update(moving(80), 15 * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(moving(80), 19 * MIN).periodic);   // still inside 5 min hold
  TEST_ASSERT_FALSE(a.update(moving(80), 20 * MIN).periodic);  // released so alertIf re-arms
  TEST_ASSERT_FALSE(a.update(moving(80), 29 * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(moving(80), 30 * MIN).periodic);
}

void test_periodic_waits_until_moving() {
  AlertLogic a = logic15();
  a.update(moving(80), 0);
  TEST_ASSERT_FALSE(a.update(moving(0), 20 * MIN).periodic);   // due but stopped
  TEST_ASSERT_FALSE(a.update(snap(true, true), 21 * MIN).periodic);  // speed unknown
  TEST_ASSERT_TRUE(a.update(moving(60), 22 * MIN).periodic);
}

void test_periodic_stops_and_resets_when_engine_off() {
  AlertLogic a = logic15();
  a.update(moving(80), 0);
  TEST_ASSERT_TRUE(a.update(moving(80), 15 * MIN).periodic);
  TEST_ASSERT_FALSE(a.update(snap(false, true), 16 * MIN).periodic);
  TEST_ASSERT_FALSE(a.update(moving(80), 17 * MIN).periodic);  // new start, timer restarts
  TEST_ASSERT_TRUE(a.update(moving(80), 32 * MIN).periodic);
}

void test_periodic_survives_millis_wrap() {
  AlertLogic a = logic15();
  uint32_t t0 = 0xFFFFFFFFu - 5 * MIN;
  a.update(moving(80), t0);
  TEST_ASSERT_TRUE(a.update(moving(80), t0 + 15 * MIN).periodic);
}

// A hybrid mid-drive: the trip is in progress, the ICE may or may not be turning.
static VehicleSnapshot hybrid(bool ice, float kmh) {
  VehicleSnapshot s = snap(ice, true);
  s.trip_active = true;
  s.has_speed = true;
  s.speed_kmh = kmh;
  return s;
}

// The ICE stopping mid-drive must not end the "started" condition: main.cpp posts on each
// false->true edge, so a drop would re-announce "Vehicle started" (and the position) on
// every restart.
void test_hybrid_ice_cycling_does_not_refire_started() {
  AlertLogic a;
  TEST_ASSERT_TRUE(a.update(hybrid(true, 50), 0).started);
  for (uint32_t t = 1; t <= 10; t++) {
    bool ice = (t % 2) == 0;  // engine cycling on and off while the trip continues
    AlertConditions c = a.update(hybrid(ice, 50), t * MIN);
    TEST_ASSERT_TRUE(c.started);  // no falling edge, so no repeat alert
    TEST_ASSERT_FALSE(c.parked);
  }
}

// The periodic countdown must survive ICE stops: restarting it on every stop meant a hybrid
// cycling faster than the interval never reported at all.
void test_hybrid_ice_cycling_still_reports_periodic() {
  AlertLogic a = logic15();
  a.update(hybrid(true, 50), 0);
  for (uint32_t t = 1; t < 15; t++) TEST_ASSERT_FALSE(a.update(hybrid((t % 2) == 0, 50), t * MIN).periodic);
  TEST_ASSERT_TRUE(a.update(hybrid(false, 50), 15 * MIN).periodic);  // due, ICE off at the time
}

// Once the trip really ends and the bus goes quiet, parking is still reported.
void test_hybrid_parked_after_trip_ends() {
  AlertLogic a;
  a.update(hybrid(true, 50), 0);
  VehicleSnapshot off = snap(false, false);  // trip over, engine off, bus quiet
  AlertConditions c = a.update(off, 20 * MIN);
  TEST_ASSERT_TRUE(c.parked);
  TEST_ASSERT_FALSE(c.started);
}

// The failure from the first Maverick run. The gateway answers the poller's own requests with
// negative responses after shutdown, so vehicle_active never goes stale and parking was never
// reported. Measured: vehicle fully powered off, vehicle_active true, no valid PID at all.
void test_parked_fires_while_gateway_still_answers() {
  AlertLogic a;
  a.update(hybrid(true, 50), 0);  // drove, so parking is armed
  VehicleSnapshot off;
  off.engine_running = false;
  off.trip_active = false;
  off.vehicle_active = true;   // frames still arriving: replies to our own polls
  off.obd_connected = false;   // but the ECU serves no data
  TEST_ASSERT_TRUE(a.update(off, 20 * MIN).parked);
}

// The other direction: a few failed polls mid-drive must not look like parking.
void test_obd_dropout_mid_drive_is_not_parked() {
  AlertLogic a;
  a.update(hybrid(true, 50), 0);
  VehicleSnapshot glitch = hybrid(false, 50);  // ICE off but still moving, trip running
  glitch.obd_connected = false;                // transient CAN failure
  TEST_ASSERT_FALSE(a.update(glitch, 5 * MIN).parked);
}

// An EV-only trip never runs the ICE, so parking must still arm from trip state alone.
void test_ev_only_trip_arms_parked() {
  AlertLogic a;
  AlertConditions c = a.update(hybrid(false, 40), 0);  // moving, ICE never ran
  TEST_ASSERT_TRUE(c.started);
  TEST_ASSERT_FALSE(c.parked);
  TEST_ASSERT_TRUE(a.update(snap(false, false), 10 * MIN).parked);
}

static VehicleSnapshot withDtcs(std::initializer_list<uint16_t> raws) {
  VehicleSnapshot s = snap(true, true);
  s.has_dtcs = true;
  for (uint16_t r : raws) {
    s.dtc_raw[s.dtc_count] = r;
    s.dtc_kind[s.dtc_count] = DTC_STORED;
    s.dtc_count++;
  }
  s.dtc_total = s.dtc_count;
  return s;
}

void test_dtc_first_read_is_baseline_no_alert() {
  AlertLogic a;
  TEST_ASSERT_FALSE(a.update(withDtcs({0x0301}), 0).dtc_new);
  TEST_ASSERT_FALSE(a.update(withDtcs({0x0301}), MIN).dtc_new);
}

void test_dtc_new_code_alerts_then_releases() {
  AlertLogic a;
  a.update(withDtcs({}), 0);
  TEST_ASSERT_TRUE(a.update(withDtcs({0x0301}), MIN).dtc_new);
  TEST_ASSERT_TRUE(a.update(withDtcs({0x0301}), 2 * MIN).dtc_new);   // held for the window
  TEST_ASSERT_FALSE(a.update(withDtcs({0x0301}), 3 * MIN).dtc_new);  // released, no repeat
  TEST_ASSERT_FALSE(a.update(withDtcs({0x0301}), 10 * MIN).dtc_new);
}

void test_dtc_additional_code_alerts_again() {
  AlertLogic a;
  a.update(withDtcs({0x0301}), 0);
  TEST_ASSERT_FALSE(a.update(withDtcs({0x0301}), MIN).dtc_new);
  TEST_ASSERT_TRUE(a.update(withDtcs({0x0301, 0x0420}), 2 * MIN).dtc_new);
}

void test_dtc_cleared_then_returns_alerts() {
  AlertLogic a;
  a.update(withDtcs({0x0301}), 0);
  a.update(withDtcs({}), MIN);
  TEST_ASSERT_TRUE(a.update(withDtcs({0x0301}), 2 * MIN).dtc_new);
}

void test_dtc_unknown_never_alerts() {
  AlertLogic a;
  VehicleSnapshot s = snap(true, true);  // has_dtcs false
  TEST_ASSERT_FALSE(a.update(s, 0).dtc_new);
  TEST_ASSERT_FALSE(a.update(s, MIN).dtc_new);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_boot_with_quiet_bus_is_not_parked);
  RUN_TEST(test_engine_on_is_started_not_parked);
  RUN_TEST(test_parked_after_drive_once_bus_quiet);
  RUN_TEST(test_restart_clears_parked_and_rearms);
  RUN_TEST(test_dtc_first_read_is_baseline_no_alert);
  RUN_TEST(test_dtc_new_code_alerts_then_releases);
  RUN_TEST(test_dtc_additional_code_alerts_again);
  RUN_TEST(test_dtc_cleared_then_returns_alerts);
  RUN_TEST(test_dtc_unknown_never_alerts);
  RUN_TEST(test_defaults_every_five_minutes_and_rearm);
  RUN_TEST(test_hold_clamped_below_interval);
  RUN_TEST(test_periodic_first_report_one_interval_after_start);
  RUN_TEST(test_periodic_held_for_window_then_released_then_repeats);
  RUN_TEST(test_hybrid_ice_cycling_does_not_refire_started);
  RUN_TEST(test_hybrid_ice_cycling_still_reports_periodic);
  RUN_TEST(test_hybrid_parked_after_trip_ends);
  RUN_TEST(test_parked_fires_while_gateway_still_answers);
  RUN_TEST(test_obd_dropout_mid_drive_is_not_parked);
  RUN_TEST(test_ev_only_trip_arms_parked);
  RUN_TEST(test_periodic_waits_until_moving);
  RUN_TEST(test_periodic_stops_and_resets_when_engine_off);
  RUN_TEST(test_periodic_survives_millis_wrap);
  return UNITY_END();
}
