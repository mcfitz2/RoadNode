#include <string.h>
#include <unity.h>

#include "storage/vehicle_storage.h"

using namespace roadnode::storage;

// In-memory two-slot store with fault injection.
class FakeStore : public SlotStore {
public:
  uint8_t data[2][VehicleStorage::ENCODED_SIZE];
  bool present[2] = {false, false};
  int fail_after_bytes = -1;  // >= 0: next write is torn after N bytes, then reports failure
  int writes = 0;

  bool read(uint8_t slot, uint8_t* buf, size_t len) override {
    if (!present[slot] || len != VehicleStorage::ENCODED_SIZE) return false;
    memcpy(buf, data[slot], len);
    return true;
  }
  bool write(uint8_t slot, const uint8_t* buf, size_t len) override {
    writes++;
    if (fail_after_bytes >= 0) {
      memcpy(data[slot], buf, fail_after_bytes);  // rest of the slot keeps its old bytes
      present[slot] = true;
      fail_after_bytes = -1;
      return false;
    }
    memcpy(data[slot], buf, len);
    present[slot] = true;
    return true;
  }
};

static VehicleRecord rec(uint64_t total, uint64_t trip, const char* id = "RAV4") {
  VehicleRecord r;
  r.total_mm = total;
  r.trip_mm = trip;
  r.last_trip_timestamp = 1700000000;
  strncpy(r.vehicle_id, id, sizeof(r.vehicle_id) - 1);
  return r;
}

void setUp() {}
void tearDown() {}

void test_fresh_device_loads_nothing() {
  FakeStore s;
  VehicleStorage st(s);
  VehicleRecord r = rec(7, 7);
  TEST_ASSERT_FALSE(st.load(r));
  TEST_ASSERT_EQUAL_UINT64(7, r.total_mm);  // untouched
}

void test_save_load_roundtrip() {
  FakeStore s;
  VehicleStorage a(s);
  TEST_ASSERT_TRUE(a.save(rec(123456789012ULL, 4242, "SILVERADO")));

  VehicleStorage b(s);  // simulates a reboot
  VehicleRecord r;
  TEST_ASSERT_TRUE(b.load(r));
  TEST_ASSERT_EQUAL_UINT64(123456789012ULL, r.total_mm);
  TEST_ASSERT_EQUAL_UINT64(4242, r.trip_mm);
  TEST_ASSERT_EQUAL_UINT32(1700000000, r.last_trip_timestamp);
  TEST_ASSERT_EQUAL_STRING("SILVERADO", r.vehicle_id);
}

void test_saves_alternate_slots_and_newest_wins() {
  FakeStore s;
  VehicleStorage st(s);
  st.save(rec(100, 1));
  st.save(rec(200, 2));
  st.save(rec(300, 3));
  TEST_ASSERT_TRUE(s.present[0] && s.present[1]);

  VehicleStorage b(s);
  VehicleRecord r;
  TEST_ASSERT_TRUE(b.load(r));
  TEST_ASSERT_EQUAL_UINT64(300, r.total_mm);
  TEST_ASSERT_EQUAL_UINT32(3, b.sequence());
}

void test_save_after_reboot_continues_sequence_and_keeps_newest() {
  FakeStore s;
  VehicleStorage a(s);
  a.save(rec(100, 0));
  a.save(rec(200, 0));

  VehicleStorage b(s);
  b.save(rec(300, 0));  // no explicit load() first
  VehicleStorage c(s);
  VehicleRecord r;
  c.load(r);
  TEST_ASSERT_EQUAL_UINT64(300, r.total_mm);
  TEST_ASSERT_EQUAL_UINT32(3, c.sequence());
}

void test_torn_write_keeps_previous_record() {
  // Power loss at every possible byte offset during the third save.
  for (int cut = 0; cut < (int)VehicleStorage::ENCODED_SIZE; cut++) {
    FakeStore s;
    VehicleStorage st(s);
    st.save(rec(100, 0));
    st.save(rec(200, 0));
    s.fail_after_bytes = cut;
    TEST_ASSERT_FALSE(st.save(rec(300, 0)));

    VehicleStorage b(s);
    VehicleRecord r;
    TEST_ASSERT_TRUE(b.load(r));
    TEST_ASSERT_EQUAL_UINT64(200, r.total_mm);  // last complete save survives
  }
}

void test_torn_first_write_is_fresh_device() {
  FakeStore s;
  VehicleStorage st(s);
  s.fail_after_bytes = 20;
  TEST_ASSERT_FALSE(st.save(rec(100, 0)));
  VehicleStorage b(s);
  VehicleRecord r;
  TEST_ASSERT_FALSE(b.load(r));
}

void test_failed_save_can_be_retried() {
  FakeStore s;
  VehicleStorage st(s);
  st.save(rec(100, 0));
  s.fail_after_bytes = 10;
  TEST_ASSERT_FALSE(st.save(rec(200, 0)));
  TEST_ASSERT_TRUE(st.save(rec(250, 0)));  // same slot, sequence not consumed

  VehicleStorage b(s);
  VehicleRecord r;
  b.load(r);
  TEST_ASSERT_EQUAL_UINT64(250, r.total_mm);
  TEST_ASSERT_EQUAL_UINT32(2, b.sequence());
}

void test_corrupt_newest_falls_back_to_older() {
  FakeStore s;
  VehicleStorage st(s);
  st.save(rec(100, 0));
  st.save(rec(200, 0));  // slot 1, newest
  s.data[1][15] ^= 0x40;  // bit flip inside the record

  VehicleStorage b(s);
  VehicleRecord r;
  TEST_ASSERT_TRUE(b.load(r));
  TEST_ASSERT_EQUAL_UINT64(100, r.total_mm);
}

void test_both_corrupt_is_fresh() {
  FakeStore s;
  VehicleStorage st(s);
  st.save(rec(100, 0));
  st.save(rec(200, 0));
  s.data[0][30] ^= 1;
  s.data[1][30] ^= 1;
  VehicleStorage b(s);
  VehicleRecord r;
  TEST_ASSERT_FALSE(b.load(r));
}

void test_wrong_magic_and_short_read_rejected() {
  FakeStore s;
  VehicleStorage st(s);
  st.save(rec(100, 0));
  s.data[0][0] ^= 0xFF;
  VehicleStorage b(s);
  VehicleRecord r;
  TEST_ASSERT_FALSE(b.load(r));
}

void test_sequence_wraparound_picks_newest() {
  FakeStore s;
  VehicleRecord r1 = rec(111, 0), r2 = rec(222, 0);
  VehicleStorage::encode(r1, 0xFFFFFFFFu, s.data[0]);
  VehicleStorage::encode(r2, 0x00000000u, s.data[1]);  // wrapped, newer
  s.present[0] = s.present[1] = true;

  VehicleStorage st(s);
  VehicleRecord r;
  TEST_ASSERT_TRUE(st.load(r));
  TEST_ASSERT_EQUAL_UINT64(222, r.total_mm);
}

void test_vehicle_id_always_terminated() {
  FakeStore s;
  VehicleStorage st(s);
  VehicleRecord r;
  memset(r.vehicle_id, 'X', sizeof(r.vehicle_id));  // no NUL
  st.save(r);
  VehicleStorage b(s);
  VehicleRecord out;
  b.load(out);
  TEST_ASSERT_EQUAL(0, out.vehicle_id[sizeof(out.vehicle_id) - 1]);
}

void test_crc32_known_vector() {
  const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926, VehicleStorage::crc32(v, sizeof(v)));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_fresh_device_loads_nothing);
  RUN_TEST(test_save_load_roundtrip);
  RUN_TEST(test_saves_alternate_slots_and_newest_wins);
  RUN_TEST(test_save_after_reboot_continues_sequence_and_keeps_newest);
  RUN_TEST(test_torn_write_keeps_previous_record);
  RUN_TEST(test_torn_first_write_is_fresh_device);
  RUN_TEST(test_failed_save_can_be_retried);
  RUN_TEST(test_corrupt_newest_falls_back_to_older);
  RUN_TEST(test_both_corrupt_is_fresh);
  RUN_TEST(test_wrong_magic_and_short_read_rejected);
  RUN_TEST(test_sequence_wraparound_picks_newest);
  RUN_TEST(test_vehicle_id_always_terminated);
  RUN_TEST(test_crc32_known_vector);
  return UNITY_END();
}
