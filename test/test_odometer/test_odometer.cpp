#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "fake_kv.h"
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
  FakeKv kv;
  bool scaleBp(uint32_t& bp) override {
    if (busy) return false;
    bp = _m.scaleBp();
    return true;
  }
  bool setScaleBp(uint32_t bp) override {
    uint32_t old = _m.scaleBp();
    if (busy || !_m.setScaleBp(bp)) return false;
    if (saveOdoScale(kv, bp)) return true;
    _m.setScaleBp(old);
    return false;
  }

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

static void drive(MileageTracker& m, uint32_t& t, int secs, uint8_t kmh) {
  for (int i = 0; i < secs; i++) {
    t += 1000;
    m.update(t, {true, true, kmh}, true);
  }
}

void test_parse_percent() {
  uint32_t bp = 0;
  TEST_ASSERT_TRUE(parsePercentBp("100", bp));
  TEST_ASSERT_EQUAL_UINT32(10000, bp);
  TEST_ASSERT_TRUE(parsePercentBp("102.5", bp));
  TEST_ASSERT_EQUAL_UINT32(10250, bp);
  TEST_ASSERT_TRUE(parsePercentBp("97.05", bp));
  TEST_ASSERT_EQUAL_UINT32(9705, bp);
  TEST_ASSERT_TRUE(parsePercentBp("80", bp));
  TEST_ASSERT_TRUE(parsePercentBp("120", bp));
  TEST_ASSERT_EQUAL_UINT32(12000, bp);
  for (const char* bad : {"", "79.99", "120.01", "-5", "abc", "1e2", "100.", ".5", "100.123", "1000", "100%", " 100"})
    TEST_ASSERT_FALSE_MESSAGE(parsePercentBp(bad, bp), bad);
}

void test_scale_command_show_set_and_reject() {
  FakeStore s;
  MileageTracker m(s);
  TrackerOdo odo(m);
  char r[160];
  TEST_ASSERT_EQUAL_STRING("odo scale 100.00%", run(odo, "odo scale", r));
  TEST_ASSERT_EQUAL_STRING("OK odo scale 102.50% (was 100.00%)", run(odo, "odo scale 102.5", r));
  TEST_ASSERT_EQUAL_UINT32(10250, m.scaleBp());
  TEST_ASSERT_EQUAL_STRING("odo scale 102.50%", run(odo, "odo scale", r));
  TEST_ASSERT_TRUE(strncmp(run(odo, "odo scale 130", r), "Err", 3) == 0);
  TEST_ASSERT_TRUE(strncmp(run(odo, "odo scale x", r), "Err", 3) == 0);
  TEST_ASSERT_EQUAL_UINT32(10250, m.scaleBp());
  odo.busy = true;
  TEST_ASSERT_TRUE(strncmp(run(odo, "odo scale 101", r), "Err: busy", 9) == 0);
  TEST_ASSERT_EQUAL_UINT32(10250, m.scaleBp());
}

void test_scale_save_failure_keeps_old_factor() {
  FakeStore s;
  MileageTracker m(s);
  TrackerOdo odo(m);
  char r[160];
  odo.kv.fail_writes = true;
  TEST_ASSERT_TRUE(strncmp(run(odo, "odo scale 105", r), "Err: not saved", 14) == 0);
  TEST_ASSERT_EQUAL_UINT32(10000, m.scaleBp());
}

void test_scale_persists_and_loads() {
  FakeKv kv;
  uint32_t bp = 0;
  TEST_ASSERT_FALSE(loadOdoScale(kv, bp));
  TEST_ASSERT_TRUE(saveOdoScale(kv, 10250));
  TEST_ASSERT_TRUE(loadOdoScale(kv, bp));
  TEST_ASSERT_EQUAL_UINT32(10250, bp);
  kv.data["odo_scale"] = "99999";  // corrupt: out of range is ignored
  bp = 10000;
  TEST_ASSERT_FALSE(loadOdoScale(kv, bp));
  TEST_ASSERT_EQUAL_UINT32(10000, bp);
  kv.data["odo_scale"] = "12x";
  TEST_ASSERT_FALSE(loadOdoScale(kv, bp));
}

void test_scale_multiplies_obd_and_gps_distance() {
  FakeStore s1, s2;
  MileageTracker a(s1), b(s2);
  TEST_ASSERT_TRUE(b.setScaleBp(10250));
  uint32_t ta = 0, tb = 0;
  drive(a, ta, 600, 60);
  drive(b, tb, 600, 60);
  double ratio = (double)b.totalMm() / (double)a.totalMm();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.025f, (float)ratio);
  TEST_ASSERT_EQUAL_UINT64(b.totalMm(), b.tripMm());  // trip follows the same factor
  FakeStore s3;
  MileageTracker g(s3);
  g.setScaleBp(10250);
  TEST_ASSERT_TRUE(g.addGpsDistance(1000, 100000, 0));
  TEST_ASSERT_EQUAL_UINT64(102500, g.totalMm());
  TEST_ASSERT_EQUAL_UINT64(102500, g.gpsFilledMm());
}

void test_scale_out_of_range_rejected_by_mileage() {
  FakeStore s;
  MileageTracker m(s);
  TEST_ASSERT_FALSE(m.setScaleBp(7999));
  TEST_ASSERT_FALSE(m.setScaleBp(12001));
  TEST_ASSERT_EQUAL_UINT32(10000, m.scaleBp());
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
  RUN_TEST(test_parse_percent);
  RUN_TEST(test_scale_command_show_set_and_reject);
  RUN_TEST(test_scale_save_failure_keeps_old_factor);
  RUN_TEST(test_scale_persists_and_loads);
  RUN_TEST(test_scale_multiplies_obd_and_gps_distance);
  RUN_TEST(test_scale_out_of_range_rejected_by_mileage);
  return UNITY_END();
}
