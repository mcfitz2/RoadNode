#include <math.h>
#include <unity.h>

#include "gps/gps_track.h"

using roadnode::gps::GpsTrack;
using roadnode::gps::GpsTrackState;

void setUp() {}
void tearDown() {}

// 1 degree of latitude = 111194.9 m (spherical, R = 6371 km).
static const double M_PER_DEG = 111194.93;
static int32_t north_e6(double metres) { return (int32_t)(metres / M_PER_DEG * 1e6); }

void test_distance_and_bearing() {
  TEST_ASSERT_FLOAT_WITHIN(1.0, 1000.0, GpsTrack::distanceM(0, 0, north_e6(1000), 0));
  TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, GpsTrack::bearingDeg(0, 0, 1000, 0));      // north
  TEST_ASSERT_FLOAT_WITHIN(0.5, 90.0, GpsTrack::bearingDeg(0, 0, 0, 1000));     // east
  TEST_ASSERT_FLOAT_WITHIN(0.5, 180.0, GpsTrack::bearingDeg(0, 0, -1000, 0));   // south
  TEST_ASSERT_FLOAT_WITHIN(0.5, 270.0, GpsTrack::bearingDeg(0, 0, 0, -1000));   // west
}

void test_steady_drive_distance_speed_heading() {
  GpsTrack g;
  uint32_t t = 1000;
  for (int i = 0; i <= 100; i++, t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * i), -93000000);  // 20 m/s = 72 km/h
  GpsTrackState s = g.state(t - 1000);
  TEST_ASSERT_TRUE(s.has_fix && s.has_motion && s.has_heading);
  TEST_ASSERT_FLOAT_WITHIN(1.0, 72.0, s.speed_kmh);
  TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, s.heading_deg > 180 ? s.heading_deg - 360.0f : s.heading_deg);
  TEST_ASSERT_FLOAT_WITHIN(5.0, 2000.0, s.trip_mm / 1000.0);
}

void test_stationary_wander_adds_nothing() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i < 600; i++, t += 1000) {
    int32_t jx = (i * 37) % 5 - 2, jy = (i * 53) % 5 - 2;  // about +-2e-6 deg = +-0.2 m
    g.update(t, true, 45000000 + jx * 10, -93000000 + jy * 10);
  }
  GpsTrackState s = g.state(t);
  TEST_ASSERT_EQUAL(0, (int)s.trip_mm);
  TEST_ASSERT_TRUE(s.has_motion);
  TEST_ASSERT_EQUAL(0, (int)s.speed_kmh);
}

void test_slow_speed_accumulates_to_threshold() {
  GpsTrack g;  // 2 m/s: below min_step per second, still counted once 8 m builds up
  uint32_t t = 0;
  for (int i = 0; i <= 100; i++, t += 1000) g.update(t, true, 45000000 + north_e6(2.0 * i), -93000000);
  GpsTrackState s = g.state(t - 1000);
  TEST_ASSERT_FLOAT_WITHIN(10.0, 200.0, s.trip_mm / 1000.0);  // within one step of 200 m
  TEST_ASSERT_FLOAT_WITHIN(1.0, 7.2, s.speed_kmh);
}

void test_stops_reads_zero_keeps_heading() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i <= 20; i++, t += 1000) g.update(t, true, 45000000, -93000000 + (int32_t)(i * 20.0 / 78000.0 * 1e6));  // east
  int32_t lat = 45000000, lon = -93000000 + (int32_t)(20 * 20.0 / 78000.0 * 1e6);
  for (int i = 0; i < 10; i++, t += 1000) g.update(t, true, lat, lon);
  GpsTrackState s = g.state(t);
  TEST_ASSERT_TRUE(s.has_motion);
  TEST_ASSERT_EQUAL(0, (int)s.speed_kmh);
  TEST_ASSERT_TRUE(s.has_heading);
  TEST_ASSERT_INT_WITHIN(2, 90, s.heading_deg);
}

void test_no_fix_means_nothing_known() {
  GpsTrack g;
  GpsTrackState s = g.state(5000);
  TEST_ASSERT_FALSE(s.has_fix || s.has_motion || s.has_heading);
  g.update(1000, false, 0, 0);
  s = g.state(2000);
  TEST_ASSERT_FALSE(s.has_fix);
}

void test_single_fix_has_no_speed() {
  GpsTrack g;
  g.update(1000, true, 45000000, -93000000);
  GpsTrackState s = g.state(1500);
  TEST_ASSERT_TRUE(s.has_fix);
  TEST_ASSERT_FALSE(s.has_motion);  // unknown, not zero
  TEST_ASSERT_FALSE(s.has_heading);
}

void test_fix_loss_gap_not_integrated() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i <= 10; i++, t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * i), -93000000);
  uint64_t before = g.state(t - 1000).trip_mm;
  t += 60000;  // tunnel: 60 s without fix, car moved ~1.2 km
  g.update(t, true, 45000000 + north_e6(20.0 * 10 + 1200), -93000000);
  TEST_ASSERT_EQUAL((int)before, (int)g.state(t).trip_mm);
  TEST_ASSERT_FALSE(g.state(t + 20000).has_fix);  // and no fix reads unknown
}

void test_fix_loss_reads_unknown() {
  GpsTrack g;
  for (uint32_t t = 0; t <= 5000; t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * (t / 1000)), -93000000);
  TEST_ASSERT_TRUE(g.state(5000).has_fix);
  GpsTrackState s = g.state(30000);
  TEST_ASSERT_FALSE(s.has_fix);
  TEST_ASSERT_FALSE(s.has_motion);
}

void test_position_jump_not_counted() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i <= 5; i++, t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * i), -93000000);
  uint64_t before = g.state(t - 1000).trip_mm;
  g.update(t, true, 45000000 + north_e6(100000), -93000000);  // 100 km in 1 s: multipath glitch
  TEST_ASSERT_EQUAL((int)before, (int)g.state(t).trip_mm);
}

void test_trip_coverage_counts_fix_time_and_resets() {
  GpsTrack g;
  g.resetTrip();
  uint32_t t = 1000;
  g.update(t, true, 45000000, -93000000);
  for (int i = 0; i < 6; i++) g.update(t += 1000, true, 45000000, -93000000);  // 6 s with fix
  for (int i = 0; i < 4; i++) g.update(t += 1000, false, 0, 0);                // 4 s without
  GpsTrackState s = g.state(t);
  TEST_ASSERT_EQUAL(6000, s.trip_fix_ms);
  TEST_ASSERT_EQUAL(10000, s.trip_total_ms);
  g.resetTrip();
  g.update(t += 1000, true, 45000000, -93000000);  // first sample after a reset has no interval yet
  s = g.state(t);
  TEST_ASSERT_EQUAL(0, s.trip_fix_ms);
  TEST_ASSERT_EQUAL(0, s.trip_total_ms);
}

void test_lifetime_total_survives_trip_reset() {
  GpsTrack g;
  uint32_t t = 1000;
  for (int i = 0; i <= 10; i++, t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * i), -93000000);
  GpsTrackState a = g.state(t - 1000);
  TEST_ASSERT_TRUE(a.total_mm > 190000);
  TEST_ASSERT_EQUAL_UINT64(a.trip_mm, a.total_mm);
  g.resetTrip();
  GpsTrackState b = g.state(t - 1000);
  TEST_ASSERT_EQUAL_UINT64(0, b.trip_mm);
  TEST_ASSERT_EQUAL_UINT64(a.total_mm, b.total_mm);
}

void test_reset_trip() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i <= 10; i++, t += 1000) g.update(t, true, 45000000 + north_e6(20.0 * i), -93000000);
  TEST_ASSERT_TRUE(g.state(t - 1000).trip_mm > 0);
  g.resetTrip();
  TEST_ASSERT_EQUAL(0, (int)g.state(t - 1000).trip_mm);
}

void test_no_drift_over_many_small_steps() {
  GpsTrack g;
  uint32_t t = 0;
  for (int i = 0; i <= 3600; i++, t += 1000) g.update(t, true, 45000000 + north_e6(9.0 * i), -93000000);  // 9 m/s for 1 h
  TEST_ASSERT_FLOAT_WITHIN(20.0, 32400.0, g.state(t - 1000).trip_mm / 1000.0);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_distance_and_bearing);
  RUN_TEST(test_steady_drive_distance_speed_heading);
  RUN_TEST(test_stationary_wander_adds_nothing);
  RUN_TEST(test_slow_speed_accumulates_to_threshold);
  RUN_TEST(test_stops_reads_zero_keeps_heading);
  RUN_TEST(test_no_fix_means_nothing_known);
  RUN_TEST(test_single_fix_has_no_speed);
  RUN_TEST(test_fix_loss_gap_not_integrated);
  RUN_TEST(test_fix_loss_reads_unknown);
  RUN_TEST(test_position_jump_not_counted);
  RUN_TEST(test_trip_coverage_counts_fix_time_and_resets);
  RUN_TEST(test_lifetime_total_survives_trip_reset);
  RUN_TEST(test_reset_trip);
  RUN_TEST(test_no_drift_over_many_small_steps);
  return UNITY_END();
}
