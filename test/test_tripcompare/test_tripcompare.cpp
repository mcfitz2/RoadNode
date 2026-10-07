#include <unity.h>

#include "fake_kv.h"
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

void test_save_load_roundtrip() {
  FakeKv kv;
  TripCompareResult r = compareTrip(10000000, 9900000, 600000, 600000);
  TEST_ASSERT_TRUE(roadnode::gps::saveTripCompare(kv, r));
  TripCompareResult back;
  TEST_ASSERT_TRUE(roadnode::gps::loadTripCompare(kv, back));
  TEST_ASSERT_TRUE(back.valid);
  TEST_ASSERT_FALSE(back.suspect);
  TEST_ASSERT_EQUAL(10000, back.obd_m);
  TEST_ASSERT_EQUAL(9900, back.gps_m);
  TEST_ASSERT_EQUAL(100, back.diff_m);
  TEST_ASSERT_EQUAL(990, back.ratio_pm);
  TEST_ASSERT_EQUAL(1000, back.coverage_pm);
}

void test_suspect_and_invalid_survive_roundtrip() {
  FakeKv kv;
  TripCompareResult back;
  roadnode::gps::saveTripCompare(kv, compareTrip(10000000, 9000000, 600000, 600000));
  TEST_ASSERT_TRUE(roadnode::gps::loadTripCompare(kv, back));
  TEST_ASSERT_TRUE(back.valid && back.suspect);
  // A later n/a trip replaces it: no stale valid ratio is left behind.
  roadnode::gps::saveTripCompare(kv, compareTrip(500000, 400000, 60000, 60000));
  TEST_ASSERT_TRUE(roadnode::gps::loadTripCompare(kv, back));
  TEST_ASSERT_FALSE(back.valid);
  TEST_ASSERT_FALSE(back.suspect);
}

void test_load_missing_or_corrupt_leaves_output_untouched() {
  FakeKv kv;
  TripCompareResult r;
  r.obd_m = 7;
  TEST_ASSERT_FALSE(roadnode::gps::loadTripCompare(kv, r));
  const char* bad[] = {"", "garbage", "1,2,3,4", "1,2,3,4,5,6", "1,2,3,4,9", "1,2,2000,4,1", "1,2,3,70000,1", "1,2,3,4,1x", "-1,2,3,4,1", "99999999999,2,3,4,1"};
  for (const char* b : bad) {
    kv.data["trip_cmp"] = b;
    TEST_ASSERT_FALSE(roadnode::gps::loadTripCompare(kv, r));
  }
  TEST_ASSERT_EQUAL(7, r.obd_m);
}

void test_save_failure_reported() {
  FakeKv kv;
  kv.fail_writes = true;
  TEST_ASSERT_FALSE(roadnode::gps::saveTripCompare(kv, TripCompareResult()));
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
  RUN_TEST(test_save_load_roundtrip);
  RUN_TEST(test_suspect_and_invalid_survive_roundtrip);
  RUN_TEST(test_load_missing_or_corrupt_leaves_output_untouched);
  RUN_TEST(test_save_failure_reported);
  return UNITY_END();
}
