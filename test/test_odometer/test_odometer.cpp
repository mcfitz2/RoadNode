#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "fake_store.h"
#include "vehicle/mileage_tracker.h"
#include "vehicle/odometer_commands.h"

using namespace roadnode::vehicle;

void setUp() {}
void tearDown() {}

// OdometerControl over a MileageTracker, like the firmware adapter minus the lock.
class TrackerOdo : public OdometerControl {
public:
  explicit TrackerOdo(MileageTracker& m) : _m(m) {}
  bool busy = false;
  bool read(uint64_t& total, uint64_t& gps) override {
    if (busy) return false;
    total = _m.totalMm();
    gps = _m.gpsFilledMm();
    return true;
  }
  bool setTotalMm(uint64_t mm) override { return !busy && _m.setTotalMm(1000, mm); }

private:
  MileageTracker& _m;
};

static const char* run(OdometerControl& o, const char* cmd, char* reply) {
  reply[0] = 0;
  return handleOdometerCommand(o, cmd, reply) ? reply : nullptr;
}

void test_miles_conversion() {
  TEST_ASSERT_EQUAL_UINT64(1609344, milesTenthsToMm(10));  // 1 mile
  TEST_ASSERT_EQUAL_UINT64(160934, milesTenthsToMm(1));
  TEST_ASSERT_EQUAL_UINT32(10, mmToMilesTenths(1609344));
  for (uint32_t t : {0u, 1u, 999u, 1874325u, 10000000u}) TEST_ASSERT_EQUAL_UINT32(t, mmToMilesTenths(milesTenthsToMm(t)));
}

void test_show_and_set() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  TrackerOdo odo(m);
  char r[160];
  TEST_ASSERT_EQUAL_STRING("odo 0.0 mi (gps filled 0.0 mi since boot)", run(odo, "odo", r));
  TEST_ASSERT_EQUAL_STRING("OK odo 187432.5 mi (was 0.0, +187432.5)", run(odo, "odo set 187432.5", r));
  TEST_ASSERT_EQUAL_UINT32(1874325, mmToMilesTenths(m.totalMm()));
  TEST_ASSERT_EQUAL_STRING("OK odo 187400.0 mi (was 187432.5, -32.5)", run(odo, "odo set 187400", r));
  TEST_ASSERT_EQUAL_STRING("OK odo 0.0 mi (was 187400.0, -187400.0)", run(odo, "odo set 0", r));
  TEST_ASSERT_EQUAL_STRING("OK odo 1000000.0 mi (was 0.0, +1000000.0)", run(odo, "odo set 1000000", r));
}

void test_set_survives_reboot_and_continues_from_new_value() {
  FakeStore s;
  {
    MileageTracker m(s);
    m.begin(0);
    TrackerOdo odo(m);
    char r[160];
    run(odo, "odo set 50000.0", r);
  }
  MileageTracker again(s);
  TEST_ASSERT_TRUE(again.begin(0));
  TEST_ASSERT_EQUAL_UINT32(500000, mmToMilesTenths(again.totalMm()));
}

void test_rejects_bad_values_and_changes_nothing() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  TrackerOdo odo(m);
  char r[160];
  odo.setTotalMm(milesTenthsToMm(1000));
  uint64_t before = m.totalMm();
  const char* bad[] = {"odo set", "odo set ", "odo set -5", "odo set 12.34", "odo set 12.", "odo set .5", "odo set abc", "odo set 12abc",
                       "odo set 1000000.1", "odo set 1000001", "odo set 99999999999999999999", "odo set 1 2"};
  for (const char* c : bad) {
    const char* out = run(odo, c, r);
    if (strcmp(c, "odo set") == 0) {
      TEST_ASSERT_NULL(out);  // not a full command: left for other handlers
      continue;
    }
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_EQUAL(0, strncmp(out, "Err", 3));
    TEST_ASSERT_EQUAL_UINT64(before, m.totalMm());
  }
  TEST_ASSERT_NULL(run(odo, "odometer", r));
  TEST_ASSERT_NULL(run(odo, "magic", r));
}

void test_save_failure_reverts_and_reports() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  TrackerOdo odo(m);
  char r[160];
  run(odo, "odo set 100.0", r);
  uint64_t before = m.totalMm();
  s.fail_writes = true;
  TEST_ASSERT_EQUAL_STRING("Err: not saved, odo unchanged", run(odo, "odo set 200.0", r));
  TEST_ASSERT_EQUAL_UINT64(before, m.totalMm());
  s.fail_writes = false;
  TEST_ASSERT_EQUAL_STRING("OK odo 200.0 mi (was 100.0, +100.0)", run(odo, "odo set 200.0", r));
}

void test_busy_reports_and_changes_nothing() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  TrackerOdo odo(m);
  odo.busy = true;
  char r[160];
  TEST_ASSERT_EQUAL_STRING("Err: busy, retry", run(odo, "odo", r));
  TEST_ASSERT_EQUAL_STRING("Err: busy, retry", run(odo, "odo set 5", r));
  TEST_ASSERT_EQUAL_UINT64(0, m.totalMm());
}

void test_set_leaves_trip_untouched_and_driving_continues() {
  FakeStore s;
  MileageTracker m(s);
  m.begin(0);
  uint32_t t = 0;
  for (int i = 0; i < 60; i++) {
    t += 1000;
    m.update(t, {true, true, 60}, true);
  }
  uint64_t trip = m.tripMm();
  TEST_ASSERT_TRUE(trip > 0);
  TrackerOdo odo(m);
  char r[160];
  run(odo, "odo set 1000", r);
  TEST_ASSERT_EQUAL_UINT64(trip, m.tripMm());
  uint64_t base = m.totalMm();
  for (int i = 0; i < 60; i++) {
    t += 1000;
    m.update(t, {true, true, 60}, true);
  }
  TEST_ASSERT_TRUE(m.totalMm() > base + 900000);  // about 1 km more
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_miles_conversion);
  RUN_TEST(test_show_and_set);
  RUN_TEST(test_set_survives_reboot_and_continues_from_new_value);
  RUN_TEST(test_rejects_bad_values_and_changes_nothing);
  RUN_TEST(test_save_failure_reverts_and_reports);
  RUN_TEST(test_busy_reports_and_changes_nothing);
  RUN_TEST(test_set_leaves_trip_untouched_and_driving_continues);
  return UNITY_END();
}
