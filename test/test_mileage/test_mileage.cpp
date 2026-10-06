#include <unity.h>

#include "vehicle/mileage.h"

using roadnode::vehicle::Mileage;

void setUp() {}
void tearDown() {}

void test_constant_speed_exact() {
  // 60 km/h for 3 s = 50 m. Each 1 s step is 16666.67 mm; the remainder carries.
  Mileage m;
  m.startTrip();
  m.update(0, 60);
  for (uint32_t t = 1000; t <= 3000; t += 1000) m.update(t, 60);
  TEST_ASSERT_EQUAL_UINT64(50000, m.totalMm());
  TEST_ASSERT_EQUAL_UINT64(50000, m.tripMm());
}

void test_single_second_truncates() {
  Mileage m;
  m.update(0, 60);
  m.update(1000, 60);
  TEST_ASSERT_EQUAL_UINT64(16666, m.totalMm());
}

void test_no_drift_over_million_samples() {
  // 100 ms steps at 36 km/h = 10 m/s = 1 m per step. 1e6 steps must give exactly 1e6 m.
  Mileage m;
  uint32_t t = 0;
  m.update(t, 36);
  for (int i = 0; i < 1000000; i++) {
    t += 100;
    m.update(t, 36);
  }
  TEST_ASSERT_EQUAL_UINT64(1000000ULL * 1000ULL, m.totalMm());
}

void test_no_drift_with_odd_step() {
  // 37 ms at 1 km/h is 10.277 mm; over 36e6 steps the total must match the exact value.
  Mileage m(100);
  uint32_t t = 0;
  m.update(t, 1);
  const uint32_t steps = 36000000;
  for (uint32_t i = 0; i < steps; i++) {
    t += 37;
    m.update(t, 1);
  }
  // total ms = steps*37 ; mm = ms * 5 / 18  (exact: 36e6*37*5/18 = 370e6)
  TEST_ASSERT_EQUAL_UINT64(370000000ULL, m.totalMm());
}

void test_trapezoid_acceleration() {
  // 0 -> 36 km/h over 10 s averages 18 km/h = 5 m/s -> 50 m.
  Mileage m;
  m.update(0, 0);
  m.update(10000, 36);
  // 10 s step exceeds the default 5 s max gap; use a larger gap limit.
  Mileage g(20000);
  g.update(0, 0);
  g.update(10000, 36);
  TEST_ASSERT_EQUAL_UINT64(50000, g.totalMm());
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
  TEST_ASSERT_EQUAL_UINT32(1, m.gapsSkipped());
}

void test_gap_not_integrated() {
  Mileage m;
  m.update(0, 100);
  m.update(60000, 100);  // 60 s dropout
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
  TEST_ASSERT_EQUAL_UINT32(1, m.gapsSkipped());
  m.update(61000, 100);  // integrates again from the new baseline
  TEST_ASSERT_EQUAL_UINT64(27777, m.totalMm());
}

void test_clock_wraparound() {
  Mileage m;
  uint32_t t = 0xFFFFFC00u;  // 1024 ms before wrap
  m.update(t, 36);
  m.update(t + 1000, 36);
  m.update(t + 2000, 36);  // crosses 2^32
  TEST_ASSERT_EQUAL_UINT64(20000, m.totalMm());
}

void test_break_continuity() {
  Mileage m;
  m.update(0, 50);
  m.breakContinuity();
  m.update(1000, 50);  // baseline only
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
  m.update(2000, 50);
  TEST_ASSERT_EQUAL_UINT64(13888, m.totalMm());
}

void test_zero_speed_adds_nothing() {
  Mileage m;
  m.update(0, 0);
  m.update(1000, 0);
  m.update(2000, 0);
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
}

void test_trip_only_while_active() {
  Mileage m;
  m.update(0, 36);
  m.update(1000, 36);  // 10 m, no trip
  m.startTrip();
  m.update(2000, 36);  // 10 m in trip
  m.endTrip();
  m.update(3000, 36);  // 10 m after trip, trip frozen
  TEST_ASSERT_EQUAL_UINT64(30000, m.totalMm());
  TEST_ASSERT_EQUAL_UINT64(10000, m.tripMm());
  m.startTrip();
  TEST_ASSERT_EQUAL_UINT64(0, m.tripMm());
}

void test_restore() {
  Mileage m;
  m.restore(123456789ULL, 4242);
  TEST_ASSERT_EQUAL_UINT64(123456789ULL, m.totalMm());
  TEST_ASSERT_EQUAL_UINT64(4242, m.tripMm());
  m.update(0, 36);
  TEST_ASSERT_EQUAL_UINT64(123456789ULL, m.totalMm());  // first sample is a baseline
}

void test_large_total_beyond_32_bits() {
  // 5,000,000 km in mm is above 2^32.
  Mileage m;
  m.restore(5000000ULL * 1000000ULL, 0);
  m.update(0, 36);
  m.update(1000, 36);
  TEST_ASSERT_EQUAL_UINT64(5000000ULL * 1000000ULL + 10000, m.totalMm());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_constant_speed_exact);
  RUN_TEST(test_single_second_truncates);
  RUN_TEST(test_no_drift_over_million_samples);
  RUN_TEST(test_no_drift_with_odd_step);
  RUN_TEST(test_trapezoid_acceleration);
  RUN_TEST(test_gap_not_integrated);
  RUN_TEST(test_clock_wraparound);
  RUN_TEST(test_break_continuity);
  RUN_TEST(test_zero_speed_adds_nothing);
  RUN_TEST(test_trip_only_while_active);
  RUN_TEST(test_restore);
  RUN_TEST(test_large_total_beyond_32_bits);
  return UNITY_END();
}
