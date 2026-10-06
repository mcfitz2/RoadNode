#include <unity.h>

#include "vehicle/alert_logic.h"

using namespace roadnode::vehicle;

static VehicleSnapshot snap(bool engine, bool active) {
  VehicleSnapshot s;
  s.engine_running = engine;
  s.vehicle_active = active;
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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_boot_with_quiet_bus_is_not_parked);
  RUN_TEST(test_engine_on_is_started_not_parked);
  RUN_TEST(test_parked_after_drive_once_bus_quiet);
  RUN_TEST(test_restart_clears_parked_and_rearms);
  RUN_TEST(test_defaults_every_five_minutes_and_rearm);
  RUN_TEST(test_hold_clamped_below_interval);
  RUN_TEST(test_periodic_first_report_one_interval_after_start);
  RUN_TEST(test_periodic_held_for_window_then_released_then_repeats);
  RUN_TEST(test_periodic_waits_until_moving);
  RUN_TEST(test_periodic_stops_and_resets_when_engine_off);
  RUN_TEST(test_periodic_survives_millis_wrap);
  return UNITY_END();
}
