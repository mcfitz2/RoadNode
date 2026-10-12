#include <string.h>
#include <unity.h>

#include "fake_kv.h"
#include "vehicle/alert_channel.h"

using namespace roadnode::vehicle;

void setUp() {}
void tearDown() {}

// Test vector only, not a real channel secret.
static const char* KEY16 = "00112233445566778899aabbccddeeff";

void test_default_off() {
  FakeKv kv;
  AlertChannel ch;
  loadAlertChannel(kv, ch);
  TEST_ASSERT_FALSE(ch.enabled);
  char r[64];
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, "alert channel", r));
  TEST_ASSERT_EQUAL_STRING("channel off", r);
}

void test_set_persists_and_reloads() {
  FakeKv kv;
  AlertChannel ch;
  char cmd[96], r[64];
  snprintf(cmd, sizeof(cmd), "alert channel %s", KEY16);
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, cmd, r));
  TEST_ASSERT_EQUAL_STRING("OK channel on", r);
  TEST_ASSERT_TRUE(ch.enabled);
  TEST_ASSERT_EQUAL(16, ch.len);
  TEST_ASSERT_EQUAL_HEX8(0x00, ch.secret[0]);
  TEST_ASSERT_EQUAL_HEX8(0xff, ch.secret[15]);

  AlertChannel after_reboot;
  loadAlertChannel(kv, after_reboot);
  TEST_ASSERT_TRUE(after_reboot.enabled);
  TEST_ASSERT_EQUAL(16, after_reboot.len);
  TEST_ASSERT_EQUAL_MEMORY(ch.secret, after_reboot.secret, 16);
}

void test_256_bit_and_uppercase() {
  FakeKv kv;
  AlertChannel ch;
  char cmd[96], r[64];
  snprintf(cmd, sizeof(cmd), "alert channel %s%s", "00112233445566778899AABBCCDDEEFF", KEY16);
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, cmd, r));
  TEST_ASSERT_EQUAL_STRING("OK channel on", r);
  TEST_ASSERT_EQUAL(32, ch.len);
  TEST_ASSERT_EQUAL_HEX8(0xaa, ch.secret[10]);
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, "alert channel", r));
  TEST_ASSERT_EQUAL_STRING("channel on (256-bit key)", r);
}

void test_show_never_echoes_secret() {
  FakeKv kv;
  AlertChannel ch;
  char cmd[96], r[64];
  snprintf(cmd, sizeof(cmd), "alert channel %s", KEY16);
  handleAlertChannelCommand(kv, ch, cmd, r);
  TEST_ASSERT_NULL(strstr(r, "0011"));
  handleAlertChannelCommand(kv, ch, "alert channel", r);
  TEST_ASSERT_EQUAL_STRING("channel on (128-bit key)", r);
}

void test_bad_secret_rejected_keeps_old() {
  FakeKv kv;
  AlertChannel ch;
  char cmd[96], r[64];
  snprintf(cmd, sizeof(cmd), "alert channel %s", KEY16);
  handleAlertChannelCommand(kv, ch, cmd, r);
  const char* bad[] = {"alert channel 0011", "alert channel 00112233445566778899aabbccddeefg",
                       "alert channel 00112233445566778899aabbccddeeff0", "alert channel "};
  for (const char* b : bad) {
    TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, b, r));
    TEST_ASSERT_EQUAL_STRING("Err: secret must be 32 or 64 hex chars", r);
    TEST_ASSERT_TRUE(ch.enabled);
    TEST_ASSERT_EQUAL_HEX8(0xff, ch.secret[15]);
  }
  TEST_ASSERT_EQUAL(1, kv.writes);
}

void test_failed_write_keeps_old() {
  FakeKv kv;
  AlertChannel ch;
  char r[64];
  kv.fail_writes = true;
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "alert channel %s", KEY16);
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, cmd, r));
  TEST_ASSERT_EQUAL_STRING("Err: save failed", r);
  TEST_ASSERT_FALSE(ch.enabled);
}

void test_off_removes() {
  FakeKv kv;
  AlertChannel ch;
  char cmd[96], r[64];
  snprintf(cmd, sizeof(cmd), "alert channel %s", KEY16);
  handleAlertChannelCommand(kv, ch, cmd, r);
  TEST_ASSERT_TRUE(handleAlertChannelCommand(kv, ch, "alert channel off", r));
  TEST_ASSERT_EQUAL_STRING("OK channel off", r);
  TEST_ASSERT_FALSE(ch.enabled);
  AlertChannel after_reboot;
  loadAlertChannel(kv, after_reboot);
  TEST_ASSERT_FALSE(after_reboot.enabled);
}

void test_corrupt_stored_value_ignored() {
  FakeKv kv;
  kv.data["alert_ch"] = "not hex";
  AlertChannel ch;
  loadAlertChannel(kv, ch);
  TEST_ASSERT_FALSE(ch.enabled);
}

void test_other_commands_not_handled() {
  FakeKv kv;
  AlertChannel ch;
  char r[64];
  TEST_ASSERT_FALSE(handleAlertChannelCommand(kv, ch, "alert", r));
  TEST_ASSERT_FALSE(handleAlertChannelCommand(kv, ch, "alert periodic 5", r));
  TEST_ASSERT_FALSE(handleAlertChannelCommand(kv, ch, "alert channelx", r));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_default_off);
  RUN_TEST(test_set_persists_and_reloads);
  RUN_TEST(test_256_bit_and_uppercase);
  RUN_TEST(test_show_never_echoes_secret);
  RUN_TEST(test_bad_secret_rejected_keeps_old);
  RUN_TEST(test_failed_write_keeps_old);
  RUN_TEST(test_off_removes);
  RUN_TEST(test_corrupt_stored_value_ignored);
  RUN_TEST(test_other_commands_not_handled);
  return UNITY_END();
}
