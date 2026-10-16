#include <unity.h>

#include "vehicle/sleep_policy.h"

using namespace roadnode::vehicle;

static const uint32_t MIN = 60u * 1000u;

void setUp() {}
void tearDown() {}

void test_parked_sleeps_after_grace() {
  SleepPolicy p;
  TEST_ASSERT_FALSE(p.update(true, false, true, 0));      // driving
  TEST_ASSERT_FALSE(p.update(false, true, false, 10 * MIN));  // parked: alert goes out
  TEST_ASSERT_FALSE(p.update(false, true, false, 11 * MIN));
  TEST_ASSERT_TRUE(p.update(false, true, false, 12 * MIN));
}

void test_car_on_blocks_sleep() {
  SleepPolicy p;
  p.update(false, true, true, 0);
  TEST_ASSERT_FALSE(p.update(false, true, true, 60 * MIN));  // ext0 would wake at once
}

void test_car_on_restarts_grace() {
  SleepPolicy p;
  p.update(false, true, false, 0);
  p.update(false, true, true, 1 * MIN);  // engine restarted briefly
  TEST_ASSERT_FALSE(p.update(false, true, false, 2 * MIN));  // grace starts again here
  TEST_ASSERT_FALSE(p.update(false, true, false, 3 * MIN));
  TEST_ASSERT_TRUE(p.update(false, true, false, 4 * MIN));
}

void test_operating_never_sleeps() {
  SleepPolicy p;
  for (uint32_t t = 0; t <= 120 * MIN; t += MIN) TEST_ASSERT_FALSE(p.update(true, false, false, t));
}

void test_idle_boot_sleeps_after_idle_timeout() {
  SleepPolicy p;
  TEST_ASSERT_FALSE(p.update(false, false, false, 0));  // e.g. first install, engine off
  TEST_ASSERT_FALSE(p.update(false, false, false, 9 * MIN));
  TEST_ASSERT_TRUE(p.update(false, false, false, 10 * MIN));
}

void test_idle_counts_from_last_operation() {
  SleepPolicy p;
  p.update(true, false, false, 0);
  p.update(true, false, false, 30 * MIN);
  TEST_ASSERT_FALSE(p.update(false, false, false, 39 * MIN));  // engine off, key on, ECU answering
  TEST_ASSERT_TRUE(p.update(false, false, false, 40 * MIN));
}

void test_millis_wrap() {
  SleepPolicy p;
  uint32_t t0 = 0xFFFFFFFFu - MIN;
  p.update(false, true, false, t0);
  TEST_ASSERT_TRUE(p.update(false, true, false, t0 + 2 * MIN));  // wraps past zero
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parked_sleeps_after_grace);
  RUN_TEST(test_car_on_blocks_sleep);
  RUN_TEST(test_car_on_restarts_grace);
  RUN_TEST(test_operating_never_sleeps);
  RUN_TEST(test_idle_boot_sleeps_after_idle_timeout);
  RUN_TEST(test_idle_counts_from_last_operation);
  RUN_TEST(test_millis_wrap);
  return UNITY_END();
}
