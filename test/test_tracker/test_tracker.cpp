#include <unity.h>

#include "fake_store.h"
#include "vehicle/mileage_tracker.h"

using namespace roadnode::vehicle;
using namespace roadnode::storage;

static const uint64_t HALF_MILE_MM = 804672;

static TripInput driving(uint8_t kmh) { return {true, true, kmh}; }
static TripInput parked() { return {false, false, 0}; }

static TripConfig tripCfg(uint32_t timeout_ms) {
  TripConfig c;
  c.end_timeout_ms = timeout_ms;
  return c;
}

// Drives at kmh for seconds, one sample per second, advancing t.
static void drive(MileageTracker& m, uint32_t& t, uint8_t kmh, int seconds, bool valid = true) {
  for (int i = 0; i < seconds; i++) {
    t += 1000;
    m.update(t, driving(kmh), valid);
  }
}

static uint64_t persistedTotal(FakeStore& s) {
  VehicleStorage st(s);
  VehicleRecord r;
  return st.load(r) ? r.total_mm : 0;
}

void setUp() {}
void tearDown() {}

void test_fresh_device() {
  FakeStore s;
  MileageTracker m(s);
  TEST_ASSERT_FALSE(m.begin(0));
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
}

void test_trip_end_checkpoints_and_survives_reboot() {
  FakeStore s;
  uint32_t t = 0;
  {
    MileageTracker m(s, tripCfg(5000));
    m.begin(t);
    m.setVehicleId("RAV4");
    m.setTime(1700000123);
    m.update(t, driving(36), true);  // trip starts
    drive(m, t, 36, 30);             // 300 m
    for (int i = 0; i < 6; i++) {    // park until the trip ends
      t += 1000;
      m.update(t, parked(), true);
    }
    TEST_ASSERT_FALSE(m.tripActive());
    TEST_ASSERT_FALSE(m.savePending());
    TEST_ASSERT_EQUAL_UINT64(305000, persistedTotal(s));  // +5 m: decel sample 36->0 km/h
  }
  MileageTracker again(s);  // reboot
  TEST_ASSERT_TRUE(again.begin(0));
  TEST_ASSERT_EQUAL_UINT64(305000, again.totalMm());
  TEST_ASSERT_EQUAL_UINT64(305000, again.tripMm());  // last trip retained
}

void test_total_continues_across_reboots() {
  FakeStore s;
  uint32_t t = 0;
  uint64_t expect = 0;
  for (int boot = 0; boot < 3; boot++) {
    MileageTracker m(s);
    m.begin(t);
    TEST_ASSERT_EQUAL_UINT64(expect, m.totalMm());
    m.update(t, driving(36), true);
    drive(m, t, 36, 100);  // 1000 m
    TEST_ASSERT_TRUE(m.shutdown(t));
    expect = m.totalMm();
    TEST_ASSERT_EQUAL_UINT64(1000000ULL * (boot + 1), expect);
  }
}

void test_crash_loses_at_most_one_checkpoint_interval() {
  // No shutdown(): power cut mid-trip. Worst case loss is bounded by the policy.
  FakeStore s;
  uint32_t t = 0;
  uint64_t before_crash;
  {
    MileageTracker m(s);
    m.begin(t);
    m.update(t, driving(100), true);
    drive(m, t, 100, 1000);  // ~27.8 km, many checkpoints
    before_crash = m.totalMm();
  }
  uint64_t saved = persistedTotal(s);
  TEST_ASSERT_TRUE(saved > 0);
  TEST_ASSERT_TRUE(before_crash - saved <= HALF_MILE_MM);
}

void test_loss_bounded_by_time_at_low_speed() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  m.update(t, driving(10), true);
  drive(m, t, 10, 600);  // 10 minutes crawling
  uint64_t saved = persistedTotal(s);
  uint64_t lost = m.totalMm() - saved;
  // at most 2 minutes at 10 km/h = about 333 m
  TEST_ASSERT_TRUE(lost <= 334000);
}

void test_no_writes_while_parked() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  for (int i = 0; i < 3600; i++) {
    t += 1000;
    m.update(t, parked(), true);
  }
  TEST_ASSERT_EQUAL(0, s.writes);
}

void test_few_writes_while_driving() {
  // 30 min at 50 km/h = 25 km = about 15.5 miles -> about 31 distance checkpoints, never per-sample.
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  m.update(t, driving(50), true);
  drive(m, t, 50, 1800);
  TEST_ASSERT_TRUE(s.writes >= 25 && s.writes <= 35);
}

void test_can_loss_does_not_invent_or_lose_distance() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  m.update(t, driving(36), true);
  drive(m, t, 36, 10);               // 100 m
  drive(m, t, 36, 120, false);       // OBD link down 2 minutes: nothing integrated
  TEST_ASSERT_EQUAL_UINT64(100000, m.totalMm());
  drive(m, t, 36, 10);               // first valid sample is a baseline only
  TEST_ASSERT_EQUAL_UINT64(190000, m.totalMm());
}

void test_failed_trip_end_save_is_retried() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s, tripCfg(3000));
  m.begin(t);
  m.update(t, driving(36), true);
  drive(m, t, 36, 20);

  s.fail_writes = true;
  for (int i = 0; i < 5; i++) {  // trip ends while flash is failing
    t += 1000;
    m.update(t, parked(), true);
  }
  TEST_ASSERT_FALSE(m.tripActive());
  TEST_ASSERT_TRUE(m.savePending());
  TEST_ASSERT_TRUE(m.saveFailures() > 0);

  s.fail_writes = false;
  t += 1000;
  m.update(t, parked(), true);
  TEST_ASSERT_FALSE(m.savePending());
  TEST_ASSERT_EQUAL_UINT64(205000, persistedTotal(s));
}

void test_failed_save_never_loses_previous_record() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  m.update(t, driving(100), true);
  drive(m, t, 100, 200);
  uint64_t good = persistedTotal(s);
  TEST_ASSERT_TRUE(good > 0);
  s.fail_writes = true;
  drive(m, t, 100, 500);
  TEST_ASSERT_EQUAL_UINT64(good, persistedTotal(s));  // still the last good one
}

void test_shutdown_failure_reported_and_retried() {
  FakeStore s;
  uint32_t t = 0;
  MileageTracker m(s);
  m.begin(t);
  m.update(t, driving(36), true);
  drive(m, t, 36, 5);
  s.fail_writes = true;
  TEST_ASSERT_FALSE(m.shutdown(t));
  TEST_ASSERT_TRUE(m.savePending());
  s.fail_writes = false;
  t += 1000;
  m.update(t, driving(36), true);
  TEST_ASSERT_FALSE(m.savePending());
}

void test_vehicle_id_and_timestamp_persist() {
  FakeStore s;
  uint32_t t = 0;
  {
    MileageTracker m(s, tripCfg(1000));
    m.begin(t);
    m.setVehicleId("MAVERICK");
    m.setTime(1700000999);
    m.update(t, driving(36), true);
    drive(m, t, 36, 3);
    for (int i = 0; i < 3; i++) {
      t += 1000;
      m.update(t, parked(), true);
    }
  }
  VehicleStorage st(s);
  VehicleRecord r;
  TEST_ASSERT_TRUE(st.load(r));
  TEST_ASSERT_EQUAL_STRING("MAVERICK", r.vehicle_id);
  TEST_ASSERT_EQUAL_UINT32(1700000999, r.last_trip_timestamp);
}

void test_trip_event_returned() {
  FakeStore s;
  MileageTracker m(s, tripCfg(1000));
  m.begin(0);
  TEST_ASSERT_EQUAL(TripEvent::Started, m.update(0, driving(10), true));
  TEST_ASSERT_EQUAL(TripEvent::None, m.update(100, parked(), true));
  TEST_ASSERT_EQUAL(TripEvent::Ended, m.update(1100, parked(), true));
}

// GPS gap fill (odometer): counted only while OBD speed is unavailable.
void test_gps_fills_only_when_obd_unavailable() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  uint32_t t = 0;
  drive(m, t, 60, 10);  // OBD valid until t=10000
  uint64_t after_obd = m.totalMm();
  // Interval 9000..10000 overlaps valid OBD: ignored.
  TEST_ASSERT_FALSE(m.addGpsDistance(t, 16000, t - 1000));
  TEST_ASSERT_EQUAL_UINT64(after_obd, m.totalMm());
  // OBD drops: samples invalid from here.
  drive(m, t, 60, 5, false);
  TEST_ASSERT_TRUE(m.addGpsDistance(t, 16000, t - 1000));  // interval starts after the last valid sample
  TEST_ASSERT_EQUAL_UINT64(after_obd + 16000, m.totalMm());
  TEST_ASSERT_EQUAL_UINT64(16000, m.gpsFilledMm());
}

void test_gps_fills_when_obd_never_valid_and_trip_follows_trip_state() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  TEST_ASSERT_TRUE(m.addGpsDistance(1000, 5000, 0));
  TEST_ASSERT_EQUAL_UINT64(5000, m.totalMm());
  TEST_ASSERT_EQUAL_UINT64(0, m.tripMm());  // no trip active
  uint32_t t = 1000;
  m.update(t += 1000, driving(0), false);  // trip starts (engine running) with no usable speed
  TEST_ASSERT_TRUE(m.tripActive());
  TEST_ASSERT_TRUE(m.addGpsDistance(t + 1000, 7000, t));
  TEST_ASSERT_EQUAL_UINT64(7000, m.tripMm());
  TEST_ASSERT_EQUAL_UINT64(12000, m.totalMm());
}

void test_gps_fill_checkpoints_and_survives_reboot() {
  FakeStore s;
  {
    MileageTracker m(s);
    m.begin(0);
    TEST_ASSERT_TRUE(m.addGpsDistance(200000, HALF_MILE_MM, 0));  // over the checkpoint distance
  }
  TEST_ASSERT_EQUAL_UINT64(HALF_MILE_MM, persistedTotal(s));
}

// 5 minutes at 60 km/h with OBD out for the middle minute: the total should land close to
// the true 5 km, not 4 km, and OBD time must not be double counted.
void test_mixed_obd_gps_drive_total_close_to_truth() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  uint32_t t = 0, gps_start = 0;
  for (int sec = 0; sec < 300; sec++) {
    t += 1000;
    bool obd = !(sec >= 120 && sec < 180);
    m.update(t, driving(60), obd);
    // GPS offers 16.67 m every second (rounded), like a perfect receiver
    m.addGpsDistance(t, 16667, gps_start);
    gps_start = t;
  }
  uint64_t truth = 5000000;
  TEST_ASSERT_UINT64_WITHIN(60000, truth, m.totalMm());  // within 60 m of 5 km
  TEST_ASSERT_TRUE(m.gpsFilledMm() > 900000 && m.gpsFilledMm() < 1100000);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_fresh_device);
  RUN_TEST(test_trip_end_checkpoints_and_survives_reboot);
  RUN_TEST(test_total_continues_across_reboots);
  RUN_TEST(test_crash_loses_at_most_one_checkpoint_interval);
  RUN_TEST(test_loss_bounded_by_time_at_low_speed);
  RUN_TEST(test_no_writes_while_parked);
  RUN_TEST(test_few_writes_while_driving);
  RUN_TEST(test_can_loss_does_not_invent_or_lose_distance);
  RUN_TEST(test_failed_trip_end_save_is_retried);
  RUN_TEST(test_failed_save_never_loses_previous_record);
  RUN_TEST(test_shutdown_failure_reported_and_retried);
  RUN_TEST(test_vehicle_id_and_timestamp_persist);
  RUN_TEST(test_trip_event_returned);
  RUN_TEST(test_gps_fills_only_when_obd_unavailable);
  RUN_TEST(test_gps_fills_when_obd_never_valid_and_trip_follows_trip_state);
  RUN_TEST(test_gps_fill_checkpoints_and_survives_reboot);
  RUN_TEST(test_mixed_obd_gps_drive_total_close_to_truth);
  return UNITY_END();
}
