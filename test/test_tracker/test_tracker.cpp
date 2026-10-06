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
  return UNITY_END();
}
