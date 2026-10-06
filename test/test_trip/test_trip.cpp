#include <unity.h>

#include "vehicle/trip.h"

using namespace roadnode::vehicle;

static TripInput driving(uint8_t kmh = 50) { return {true, true, kmh}; }
static TripInput idling() { return {true, true, 0}; }
static TripInput off() { return {false, false, 0}; }

static TripConfig cfg(uint32_t timeout_ms) {
  TripConfig c;
  c.end_timeout_ms = timeout_ms;
  return c;
}

void setUp() {}
void tearDown() {}

void test_starts_on_operation() {
  Trip t(cfg(1000));
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(0, off()));
  TEST_ASSERT_FALSE(t.active());
  TEST_ASSERT_EQUAL(TripEvent::Started, t.update(100, driving()));
  TEST_ASSERT_TRUE(t.active());
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(200, driving()));
}

void test_starts_on_engine_running_without_speed() {
  Trip t(cfg(1000));
  TEST_ASSERT_EQUAL(TripEvent::Started, t.update(0, idling()));
}

void test_not_started_when_inactive_even_if_speed() {
  Trip t(cfg(1000));
  TripInput in = {false, false, 30};
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(0, in));
}

void test_ends_after_timeout() {
  Trip t(cfg(1000));
  t.update(0, driving());
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(100, off()));   // countdown starts
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(1099, off()));  // 999 ms
  TEST_ASSERT_EQUAL(TripEvent::Ended, t.update(1100, off())); // 1000 ms
  TEST_ASSERT_FALSE(t.active());
}

void test_activity_cancels_countdown() {
  Trip t(cfg(1000));
  t.update(0, driving());
  t.update(100, off());
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(900, driving()));  // back before timeout
  t.update(1000, off());                                         // new countdown from 1000
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(1999, off()));
  TEST_ASSERT_EQUAL(TripEvent::Ended, t.update(2000, off()));
}

void test_flapping_does_not_end_or_restart() {
  Trip t(cfg(1000));
  TEST_ASSERT_EQUAL(TripEvent::Started, t.update(0, driving()));
  for (uint32_t ts = 100; ts < 5000; ts += 200) {
    TEST_ASSERT_EQUAL(TripEvent::None, t.update(ts, off()));
    TEST_ASSERT_EQUAL(TripEvent::None, t.update(ts + 100, driving()));
  }
  TEST_ASSERT_TRUE(t.active());
}

void test_restarts_after_end() {
  Trip t(cfg(1000));
  t.update(0, driving());
  t.update(10, off());
  TEST_ASSERT_EQUAL(TripEvent::Ended, t.update(1010, off()));
  TEST_ASSERT_EQUAL(TripEvent::Started, t.update(2000, driving()));
}

void test_stopped_with_engine_off_ends_trip() {
  Trip t(cfg(1000));
  t.update(0, driving());
  TripInput parked = {true, false, 0};  // ignition on, engine off, not moving
  t.update(100, parked);
  TEST_ASSERT_EQUAL(TripEvent::Ended, t.update(1100, parked));
}

void test_idling_keeps_trip_alive() {
  Trip t(cfg(1000));
  t.update(0, driving());
  for (uint32_t ts = 100; ts < 10000; ts += 100) TEST_ASSERT_EQUAL(TripEvent::None, t.update(ts, idling()));
  TEST_ASSERT_TRUE(t.active());
}

void test_timeout_survives_clock_wraparound() {
  Trip t(cfg(1000));
  uint32_t start = 0xFFFFFF00u;
  t.update(start, driving());
  t.update(start + 10, off());
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(start + 900, off()));
  TEST_ASSERT_EQUAL(TripEvent::Ended, t.update(start + 1010, off()));  // crosses 2^32
}

void test_reset() {
  Trip t(cfg(1000));
  t.update(0, driving());
  t.reset();
  TEST_ASSERT_FALSE(t.active());
  TEST_ASSERT_EQUAL(TripEvent::None, t.update(10, off()));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_on_operation);
  RUN_TEST(test_starts_on_engine_running_without_speed);
  RUN_TEST(test_not_started_when_inactive_even_if_speed);
  RUN_TEST(test_ends_after_timeout);
  RUN_TEST(test_activity_cancels_countdown);
  RUN_TEST(test_flapping_does_not_end_or_restart);
  RUN_TEST(test_restarts_after_end);
  RUN_TEST(test_stopped_with_engine_off_ends_trip);
  RUN_TEST(test_idling_keeps_trip_alive);
  RUN_TEST(test_timeout_survives_clock_wraparound);
  RUN_TEST(test_reset);
  return UNITY_END();
}
