#include <string.h>
#include <unity.h>

#include "fake_kv.h"
#include "vehicle/alert_settings.h"

using namespace roadnode::vehicle;

void setUp() {}
void tearDown() {}

static VehicleSnapshot moving(uint8_t) {
  VehicleSnapshot s;
  s.engine_running = true;
  s.vehicle_active = true;
  s.has_speed = true;
  s.speed_kmh = 60;
  return s;
}

void test_show_default() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  TEST_ASSERT_TRUE(handleAlertCommand(kv, l, "alert", r));
  TEST_ASSERT_EQUAL_STRING("periodic 5 min", r);
}

void test_set_persists_and_reloads() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  TEST_ASSERT_TRUE(handleAlertCommand(kv, l, "alert periodic 15", r));
  TEST_ASSERT_EQUAL_STRING("OK periodic 15 min", r);
  TEST_ASSERT_EQUAL(15 * 60000u, l.periodicIntervalMs());
  AlertLogic after_reboot;
  loadAlertSettings(kv, after_reboot);
  TEST_ASSERT_EQUAL(15 * 60000u, after_reboot.periodicIntervalMs());
}

void test_no_saved_value_keeps_default() {
  FakeKv kv;
  AlertLogic l;
  loadAlertSettings(kv, l);
  TEST_ASSERT_EQUAL(5 * 60000u, l.periodicIntervalMs());
}

void test_invalid_rejected_and_unchanged() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  const char* bad[] = {"alert periodic ", "alert periodic x", "alert periodic -1", "alert periodic 1441", "alert periodic 99999999999"};
  for (const char* c : bad) {
    TEST_ASSERT_TRUE(handleAlertCommand(kv, l, c, r));
    TEST_ASSERT_TRUE(strncmp(r, "Err", 3) == 0);
  }
  TEST_ASSERT_EQUAL(5 * 60000u, l.periodicIntervalMs());
  TEST_ASSERT_EQUAL(0, kv.writes);
}

void test_save_failure_leaves_setting() {
  FakeKv kv;
  kv.fail_writes = true;
  AlertLogic l;
  char r[64];
  handleAlertCommand(kv, l, "alert periodic 30", r);
  TEST_ASSERT_TRUE(strncmp(r, "Err", 3) == 0);
  TEST_ASSERT_EQUAL(5 * 60000u, l.periodicIntervalMs());
}

void test_zero_disables_periodic() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  handleAlertCommand(kv, l, "alert periodic 0", r);
  TEST_ASSERT_EQUAL_STRING("OK periodic off", r);
  uint32_t t = 0;
  bool any = false;
  for (int i = 0; i < 300; i++, t += 60000) any |= l.update(moving(0), t).periodic;  // 5 h driving
  TEST_ASSERT_FALSE(any);
}

void test_new_interval_applies() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  handleAlertCommand(kv, l, "alert periodic 10", r);
  uint32_t t = 0;
  int first = -1;
  for (int i = 0; i < 30; i++, t += 60000)
    if (l.update(moving(0), t).periodic && first < 0) first = i;
  TEST_ASSERT_EQUAL(10, first);  // 10 min after engine start
}

void test_unknown_command_not_handled() {
  FakeKv kv;
  AlertLogic l;
  char r[64];
  TEST_ASSERT_FALSE(handleAlertCommand(kv, l, "magic", r));
  TEST_ASSERT_FALSE(handleAlertCommand(kv, l, "alertx", r));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_show_default);
  RUN_TEST(test_set_persists_and_reloads);
  RUN_TEST(test_no_saved_value_keeps_default);
  RUN_TEST(test_invalid_rejected_and_unchanged);
  RUN_TEST(test_save_failure_leaves_setting);
  RUN_TEST(test_zero_disables_periodic);
  RUN_TEST(test_new_interval_applies);
  RUN_TEST(test_unknown_command_not_handled);
  return UNITY_END();
}
