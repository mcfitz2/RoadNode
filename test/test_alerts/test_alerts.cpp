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
  AlertConditions c = a.update(snap(false, false));
  TEST_ASSERT_FALSE(c.started);
  TEST_ASSERT_FALSE(c.parked);
}

void test_engine_on_is_started_not_parked() {
  AlertLogic a;
  AlertConditions c = a.update(snap(true, true));
  TEST_ASSERT_TRUE(c.started);
  TEST_ASSERT_FALSE(c.parked);
}

void test_parked_after_drive_once_bus_quiet() {
  AlertLogic a;
  a.update(snap(true, true));
  AlertConditions idle = a.update(snap(false, true));  // engine off, bus still awake
  TEST_ASSERT_FALSE(idle.parked);
  AlertConditions quiet = a.update(snap(false, false));
  TEST_ASSERT_TRUE(quiet.parked);
  TEST_ASSERT_FALSE(quiet.started);
}

void test_restart_clears_parked_and_rearms() {
  AlertLogic a;
  a.update(snap(true, true));
  TEST_ASSERT_TRUE(a.update(snap(false, false)).parked);
  AlertConditions c = a.update(snap(true, true));
  TEST_ASSERT_TRUE(c.started);
  TEST_ASSERT_FALSE(c.parked);
  TEST_ASSERT_TRUE(a.update(snap(false, false)).parked);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_boot_with_quiet_bus_is_not_parked);
  RUN_TEST(test_engine_on_is_started_not_parked);
  RUN_TEST(test_parked_after_drive_once_bus_quiet);
  RUN_TEST(test_restart_clears_parked_and_rearms);
  return UNITY_END();
}
