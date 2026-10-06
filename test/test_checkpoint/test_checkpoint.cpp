#include <unity.h>

#include "storage/checkpoint_policy.h"

using namespace roadnode::storage;

static const uint64_t HALF_MILE_MM = 804672;

void setUp() {}
void tearDown() {}

void test_defaults_are_half_mile_and_two_minutes() {
  CheckpointConfig c;
  TEST_ASSERT_EQUAL_UINT64(HALF_MILE_MM, c.distance_mm);
  TEST_ASSERT_EQUAL_UINT32(120000, c.interval_ms);
}

void test_nothing_due_while_parked() {
  CheckpointPolicy p;
  p.markSaved(0, 1000);
  TEST_ASSERT_FALSE(p.due(10UL * 60 * 60 * 1000, 1000));  // 10 h, no distance change
}

void test_distance_trigger() {
  CheckpointPolicy p;
  p.markSaved(0, 0);
  TEST_ASSERT_FALSE(p.due(1000, HALF_MILE_MM - 1));
  TEST_ASSERT_TRUE(p.due(1000, HALF_MILE_MM));
}

void test_time_trigger_needs_movement() {
  CheckpointPolicy p;
  p.markSaved(0, 0);
  TEST_ASSERT_FALSE(p.due(119999, 10));
  TEST_ASSERT_TRUE(p.due(120000, 10));
}

void test_mark_saved_resets_both_counters() {
  CheckpointPolicy p;
  p.markSaved(0, 0);
  TEST_ASSERT_TRUE(p.due(120000, 500));
  p.markSaved(120000, 500);
  TEST_ASSERT_FALSE(p.due(120001, 501));
  TEST_ASSERT_FALSE(p.due(239999, 600));
  TEST_ASSERT_TRUE(p.due(240000, 600));
  TEST_ASSERT_TRUE(p.due(130000, 500 + HALF_MILE_MM));
}

void test_clock_wraparound() {
  CheckpointPolicy p;
  p.markSaved(0xFFFFFF00u, 0);
  TEST_ASSERT_FALSE(p.due(0xFFFFFF00u + 1000, 5));
  TEST_ASSERT_TRUE(p.due(0xFFFFFF00u + 120000, 5));  // crosses 2^32
}

void test_custom_config() {
  CheckpointConfig c;
  c.distance_mm = 100;
  c.interval_ms = 1000;
  CheckpointPolicy p(c);
  TEST_ASSERT_TRUE(p.due(0, 100));
  TEST_ASSERT_TRUE(p.due(1000, 1));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_half_mile_and_two_minutes);
  RUN_TEST(test_nothing_due_while_parked);
  RUN_TEST(test_distance_trigger);
  RUN_TEST(test_time_trigger_needs_movement);
  RUN_TEST(test_mark_saved_resets_both_counters);
  RUN_TEST(test_clock_wraparound);
  RUN_TEST(test_custom_config);
  return UNITY_END();
}
