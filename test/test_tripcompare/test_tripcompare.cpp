#include <unity.h>

#include "gps/trip_compare.h"

using roadnode::gps::compareTrip;
using roadnode::gps::TripCompareConfig;
using roadnode::gps::TripCompareResult;

void setUp() {}
void tearDown() {}

void test_healthy_trip_is_valid_and_ok() {
  TripCompareResult r = compareTrip(10000000, 9900000, 600000, 600000);  // 10 km vs 9.9 km, full coverage
  TEST_ASSERT_TRUE(r.valid);
  TEST_ASSERT_FALSE(r.suspect);
  TEST_ASSERT_EQUAL(10000, r.obd_m);
  TEST_ASSERT_EQUAL(9900, r.gps_m);
  TEST_ASSERT_EQUAL(100, r.diff_m);
  TEST_ASSERT_EQUAL(990, r.ratio_pm);
  TEST_ASSERT_EQUAL(1000, r.coverage_pm);
}

void test_obd_reading_high_is_suspect() {
  TripCompareResult r = compareTrip(10000000, 9000000, 600000, 600000);  // GPS 90% of OBD
  TEST_ASSERT_TRUE(r.valid);
  TEST_ASSERT_TRUE(r.suspect);
  TEST_ASSERT_EQUAL(900, r.ratio_pm);
}

void test_gps_reading_high_is_suspect() {
  TripCompareResult r = compareTrip(10000000, 11000000, 600000, 600000);
  TEST_ASSERT_TRUE(r.valid);
  TEST_ASSERT_TRUE(r.suspect);
  TEST_ASSERT_EQUAL(-1000, r.diff_m);
}

void test_short_trip_is_invalid() {
  TripCompareResult r = compareTrip(500000, 400000, 60000, 60000);  // 500 m
  TEST_ASSERT_FALSE(r.valid);
  TEST_ASSERT_FALSE(r.suspect);
  TEST_ASSERT_EQUAL(0, r.ratio_pm);
}

void test_low_gps_coverage_is_invalid_not_suspect() {
  // Half the trip had no fix: GPS distance is low for that reason alone.
  TripCompareResult r = compareTrip(10000000, 5000000, 300000, 600000);
  TEST_ASSERT_FALSE(r.valid);
  TEST_ASSERT_FALSE(r.suspect);
  TEST_ASSERT_EQUAL(500, r.coverage_pm);
}

void test_zero_obd_and_zero_time_do_not_divide_by_zero() {
  TripCompareConfig cfg;
  cfg.min_trip_m = 0;
  TripCompareResult a = compareTrip(0, 0, 0, 0, cfg);
  TEST_ASSERT_FALSE(a.valid);
  TripCompareResult b = compareTrip(5000000, 4900000, 0, 0, cfg);  // no sampled time at all
  TEST_ASSERT_FALSE(b.valid);
  TEST_ASSERT_EQUAL(0, b.coverage_pm);
}

void test_thresholds_come_from_config() {
  TripCompareConfig cfg;
  cfg.warn_low_pm = 990;
  TripCompareResult r = compareTrip(10000000, 9800000, 600000, 600000, cfg);
  TEST_ASSERT_TRUE(r.suspect);
  TEST_ASSERT_FALSE(compareTrip(10000000, 9800000, 600000, 600000).suspect);
}

void test_ratio_and_coverage_are_capped() {
  TripCompareResult r = compareTrip(1000000, 900000000, 700000, 600000);  // GPS 900x OBD, fix_ms > total
  TEST_ASSERT_EQUAL(1000, r.coverage_pm);
  TEST_ASSERT_EQUAL(65535, r.ratio_pm);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_healthy_trip_is_valid_and_ok);
  RUN_TEST(test_obd_reading_high_is_suspect);
  RUN_TEST(test_gps_reading_high_is_suspect);
  RUN_TEST(test_short_trip_is_invalid);
  RUN_TEST(test_low_gps_coverage_is_invalid_not_suspect);
  RUN_TEST(test_zero_obd_and_zero_time_do_not_divide_by_zero);
  RUN_TEST(test_thresholds_come_from_config);
  RUN_TEST(test_ratio_and_coverage_are_capped);
  return UNITY_END();
}
