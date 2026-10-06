#include <string.h>
#include <unity.h>


#include "fake_kv.h"
#include "vehicle/identity_commands.h"
#include "vehicle/vehicle_identity.h"

using roadnode::vehicle::VehicleIdentity;
using Vin = VehicleIdentity::VinResult;

static const char* GOOD_VIN = "1HGCM82633A004352";
static const char* BAD_CHECK_VIN = "1HGCM82643A004352";  // well formed, wrong check digit

void setUp() {}
void tearDown() {}

void test_default_id_used_when_nothing_persisted() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("rav4");
  TEST_ASSERT_EQUAL_STRING("RAV4", id.id());
  TEST_ASSERT_EQUAL(0, kv.writes);  // fallback is not written back
}

void test_invalid_default_falls_back_to_unset() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("has space");
  TEST_ASSERT_EQUAL_STRING("UNSET", id.id());
}

void test_set_id_persists_and_wins_over_default() {
  FakeKv kv;
  {
    VehicleIdentity id(kv);
    id.begin("RAV4");
    TEST_ASSERT_TRUE(id.setId("truck-2"));
    TEST_ASSERT_EQUAL_STRING("TRUCK-2", id.id());
  }
  VehicleIdentity again(kv);  // reboot
  again.begin("RAV4");
  TEST_ASSERT_EQUAL_STRING("TRUCK-2", again.id());
}

void test_set_id_validation() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  TEST_ASSERT_FALSE(id.setId(""));
  TEST_ASSERT_FALSE(id.setId("ABCDEFGHIJKLMNOP"));  // 16
  TEST_ASSERT_FALSE(id.setId("a b"));
  TEST_ASSERT_FALSE(id.setId("a;b"));
  TEST_ASSERT_FALSE(id.setId(nullptr));
  TEST_ASSERT_TRUE(id.setId("ABCDEFGHIJKLMNO"));  // 15
  TEST_ASSERT_TRUE(id.setId("x_y-1"));
  TEST_ASSERT_EQUAL_STRING("X_Y-1", id.id());
}

void test_set_id_write_failure_keeps_old_id() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  kv.fail_writes = true;
  TEST_ASSERT_FALSE(id.setId("NEW"));
  TEST_ASSERT_EQUAL_STRING("RAV4", id.id());
}

void test_corrupt_persisted_id_ignored() {
  FakeKv kv;
  kv.data["vid"] = "bad id!";
  VehicleIdentity id(kv);
  id.begin("RAV4");
  TEST_ASSERT_EQUAL_STRING("RAV4", id.id());
}

void test_copy_id() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  char out[16];
  id.copyId(out);
  TEST_ASSERT_EQUAL_STRING("RAV4", out);
}

void test_first_vin_stored_and_persisted() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  TEST_ASSERT_FALSE(id.hasVin());
  TEST_ASSERT_EQUAL((int)Vin::Stored, (int)id.onVinRead(GOOD_VIN));
  TEST_ASSERT_TRUE(id.hasVin());
  TEST_ASSERT_TRUE(id.vinCheckDigitOk());
  VehicleIdentity again(kv);
  again.begin("RAV4");
  TEST_ASSERT_TRUE(again.hasVin());
  TEST_ASSERT_EQUAL_STRING(GOOD_VIN, again.vin());
  TEST_ASSERT_EQUAL((int)Vin::Unchanged, (int)again.onVinRead(GOOD_VIN));
}

void test_bad_check_digit_stored_but_flagged() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  TEST_ASSERT_EQUAL((int)Vin::Stored, (int)id.onVinRead(BAD_CHECK_VIN));
  TEST_ASSERT_FALSE(id.vinCheckDigitOk());
}

void test_malformed_vin_rejected() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  TEST_ASSERT_EQUAL((int)Vin::Invalid, (int)id.onVinRead("1HGCM8263IA004352"));  // I
  TEST_ASSERT_EQUAL((int)Vin::Invalid, (int)id.onVinRead("short"));
  TEST_ASSERT_EQUAL((int)Vin::Invalid, (int)id.onVinRead("1hgcm82633a004352"));  // lowercase
  TEST_ASSERT_FALSE(id.hasVin());
  TEST_ASSERT_EQUAL(0, kv.writes);
}

void test_vin_mismatch_keeps_stored_and_flags() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  id.onVinRead(GOOD_VIN);
  TEST_ASSERT_EQUAL((int)Vin::Mismatch, (int)id.onVinRead("2HGCM82633A004352"));
  TEST_ASSERT_TRUE(id.vinMismatch());
  TEST_ASSERT_EQUAL_STRING(GOOD_VIN, id.vin());
}

void test_vin_write_failure_not_remembered() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  kv.fail_writes = true;
  TEST_ASSERT_EQUAL((int)Vin::WriteFailed, (int)id.onVinRead(GOOD_VIN));
  TEST_ASSERT_FALSE(id.hasVin());
  kv.fail_writes = false;
  TEST_ASSERT_EQUAL((int)Vin::Stored, (int)id.onVinRead(GOOD_VIN));
}

void test_clear_vin_allows_new_vehicle() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  id.onVinRead(GOOD_VIN);
  TEST_ASSERT_TRUE(id.clearVin());
  TEST_ASSERT_FALSE(id.hasVin());
  TEST_ASSERT_EQUAL((int)Vin::Stored, (int)id.onVinRead("2HGCM82633A004352"));
  TEST_ASSERT_FALSE(id.vinMismatch());
}

void test_commands() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  char reply[160];
  TEST_ASSERT_TRUE(roadnode::vehicle::handleIdentityCommand(id, "vid", reply));
  TEST_ASSERT_EQUAL_STRING("id RAV4", reply);
  TEST_ASSERT_TRUE(roadnode::vehicle::handleIdentityCommand(id, "vid set camry", reply));
  TEST_ASSERT_EQUAL_STRING("OK id CAMRY", reply);
  TEST_ASSERT_TRUE(roadnode::vehicle::handleIdentityCommand(id, "vid set no way", reply));
  TEST_ASSERT_EQUAL_STRING("CAMRY", id.id());
  TEST_ASSERT_TRUE(strncmp(reply, "Err", 3) == 0);
  TEST_ASSERT_FALSE(roadnode::vehicle::handleIdentityCommand(id, "magic", reply));
  TEST_ASSERT_TRUE(roadnode::vehicle::handleIdentityCommand(id, "vin", reply));
  TEST_ASSERT_EQUAL_STRING("vin not read", reply);
}

void test_vin_never_in_command_replies() {
  FakeKv kv;
  VehicleIdentity id(kv);
  id.begin("RAV4");
  id.onVinRead(GOOD_VIN);
  char reply[160];
  const char* cmds[] = {"vid", "vin", "vid set x", "vin clear"};
  id.onVinRead(GOOD_VIN);
  for (const char* c : cmds) {
    roadnode::vehicle::handleIdentityCommand(id, c, reply);
    TEST_ASSERT_NULL(strstr(reply, GOOD_VIN));
    if (strcmp(c, "vin clear") != 0) id.onVinRead(GOOD_VIN);
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_default_id_used_when_nothing_persisted);
  RUN_TEST(test_invalid_default_falls_back_to_unset);
  RUN_TEST(test_set_id_persists_and_wins_over_default);
  RUN_TEST(test_set_id_validation);
  RUN_TEST(test_set_id_write_failure_keeps_old_id);
  RUN_TEST(test_corrupt_persisted_id_ignored);
  RUN_TEST(test_copy_id);
  RUN_TEST(test_first_vin_stored_and_persisted);
  RUN_TEST(test_bad_check_digit_stored_but_flagged);
  RUN_TEST(test_malformed_vin_rejected);
  RUN_TEST(test_vin_mismatch_keeps_stored_and_flags);
  RUN_TEST(test_vin_write_failure_not_remembered);
  RUN_TEST(test_clear_vin_allows_new_vehicle);
  RUN_TEST(test_commands);
  RUN_TEST(test_vin_never_in_command_replies);
  return UNITY_END();
}
