#include <string.h>
#include <unity.h>

#include "can/mcp2515.h"

using namespace roadnode::can;

void setUp() {}
void tearDown() {}

// Register-level model of an MCP2515: enough of the SPI instruction set for the driver.
class FakeChip : public SpiBus {
public:
  uint8_t reg[128];
  bool dead = false;       // MISO stuck low, as with a missing chip
  bool miswired = false;   // writes are lost, as with a broken MOSI line
  bool acked = true;       // a transmitted frame finds a receiver
  int sent_count = 0;
  uint8_t last_sent[13];
  uint32_t delays = 0;

  FakeChip() { powerOn(); }

  void powerOn() {
    memset(reg, 0, sizeof(reg));
    reg[0x0E] = 0x80;
    reg[0x0F] = 0x87;
  }

  void injectRx(int n, const uint8_t* bytes13) {
    memcpy(&reg[0x61 + 0x10 * n], bytes13, 13);
    reg[0x2C] |= (uint8_t)(1u << n);
  }

  void syncMode() { reg[0x0E] = reg[0x0F] & 0xE0; }

  void transfer(const uint8_t* tx, uint8_t* rx, size_t n) override {
    if (rx) memset(rx, 0, n);
    if (dead) return;
    uint8_t cmd = tx[0];
    if (cmd == 0xC0) {
      powerOn();
    } else if (cmd == 0x03) {
      for (size_t i = 2; i < n; i++) rx[i] = reg[tx[1] + i - 2];
    } else if (cmd == 0x02) {
      if (miswired) return;
      for (size_t i = 2; i < n; i++) reg[tx[1] + i - 2] = tx[i];
      if (tx[1] == 0x0F) syncMode();
    } else if (cmd == 0x05) {
      uint8_t a = tx[1], m = tx[2], v = tx[3];
      reg[a] = (uint8_t)((reg[a] & ~m) | (v & m));
      if (a == 0x0F) {
        syncMode();
        if (reg[0x0F] & 0x10) {
          for (int b = 0; b < 3; b++) reg[0x30 + 0x10 * b] &= (uint8_t)~0x08;  // ABAT
        }
      }
    } else if ((cmd & 0xF8) == 0x40) {
      int b = (cmd >> 1) & 3;
      for (size_t i = 1; i < n; i++) reg[0x31 + 0x10 * b + i - 1] = tx[i];
    } else if ((cmd & 0xF8) == 0x80) {
      for (int b = 0; b < 3; b++) {
        if (!(cmd & (1u << b))) continue;
        sent_count++;
        memcpy(last_sent, &reg[0x31 + 0x10 * b], 13);
        if ((reg[0x0F] & 0xE0) == 0x40) {
          injectRx(0, &reg[0x31 + 0x10 * b]);  // loopback: frame arrives in RXB0
        } else if (!acked) {
          reg[0x30 + 0x10 * b] |= 0x08;  // TXREQ stays set, retrying forever
        }
      }
    } else if (cmd == 0xA0) {
      rx[1] = (uint8_t)((reg[0x2C] & 0x03));
    } else if ((cmd & 0xFB) == 0x90) {
      int b = (cmd >> 2) & 1;
      for (size_t i = 1; i < n && i <= 13; i++) rx[i] = reg[0x60 + 0x10 * b + i];
      reg[0x2C] &= (uint8_t)~(1u << b);
    }
  }
  void delayMs(uint32_t) override { delays++; }
};

static Frame mkFrame(uint32_t id, uint8_t dlc, bool ext = false) {
  Frame f;
  f.id = id;
  f.dlc = dlc;
  f.extended = ext;
  for (int i = 0; i < dlc; i++) f.data[i] = (uint8_t)(0xA0 + i);
  return f;
}

void test_timing_matches_proven_values() {
  uint8_t a, b, c;
  TEST_ASSERT_TRUE(Mcp2515::timing(500000, 16000000, a, b, c));
  TEST_ASSERT_EQUAL_HEX8(0x00, a);
  TEST_ASSERT_EQUAL_HEX8(0xF0, b);
  TEST_ASSERT_EQUAL_HEX8(0x86, c);
  TEST_ASSERT_TRUE(Mcp2515::timing(250000, 16000000, a, b, c));
  TEST_ASSERT_EQUAL_HEX8(0x01, a);
  TEST_ASSERT_TRUE(Mcp2515::timing(125000, 16000000, a, b, c));
  TEST_ASSERT_EQUAL_HEX8(0x03, a);
  TEST_ASSERT_TRUE(Mcp2515::timing(500000, 32000000, a, b, c));
  TEST_ASSERT_EQUAL_HEX8(0x01, a);
}

void test_timing_rejects_unreachable_bitrates() {
  uint8_t a, b, c;
  TEST_ASSERT_FALSE(Mcp2515::timing(1000000, 16000000, a, b, c));  // would need half a quantum divider
  TEST_ASSERT_FALSE(Mcp2515::timing(500000, 8000000, a, b, c));
  TEST_ASSERT_FALSE(Mcp2515::timing(0, 16000000, a, b, c));
  TEST_ASSERT_FALSE(Mcp2515::timing(500000, 0, a, b, c));
}

void test_begin_programs_timing_filters_and_mode() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  TEST_ASSERT_TRUE(mcp.begin(500000, 16000000, Mode::Normal));
  TEST_ASSERT_EQUAL_HEX8(0x00, chip.reg[0x2A]);
  TEST_ASSERT_EQUAL_HEX8(0xF0, chip.reg[0x29]);
  TEST_ASSERT_EQUAL_HEX8(0x86, chip.reg[0x28]);
  TEST_ASSERT_EQUAL_HEX8(0x64, chip.reg[0x60]);  // accept all, rollover
  TEST_ASSERT_EQUAL_HEX8(0x60, chip.reg[0x70]);
  TEST_ASSERT_EQUAL_HEX8(0x03, chip.reg[0x2B]);  // INT on both RX buffers
  TEST_ASSERT_EQUAL_HEX8(0x00, chip.reg[0x0E] & 0xE0);
}

void test_begin_selects_each_mode() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  TEST_ASSERT_TRUE(mcp.begin(500000, 16000000, Mode::ListenOnly));
  TEST_ASSERT_EQUAL_HEX8(0x60, chip.reg[0x0E] & 0xE0);
  TEST_ASSERT_TRUE(mcp.begin(500000, 16000000, Mode::Loopback));
  TEST_ASSERT_EQUAL_HEX8(0x40, chip.reg[0x0E] & 0xE0);
}

void test_begin_fails_without_a_chip() {
  FakeChip chip;
  chip.dead = true;
  Mcp2515 mcp(chip);
  TEST_ASSERT_FALSE(mcp.begin(500000, 16000000, Mode::Normal));
}

void test_begin_fails_when_writes_do_not_land() {
  FakeChip chip;
  chip.miswired = true;
  Mcp2515 mcp(chip);
  TEST_ASSERT_FALSE(mcp.begin(500000, 16000000, Mode::Normal));
}

void test_begin_rejects_unreachable_bitrate() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  TEST_ASSERT_FALSE(mcp.begin(1000000, 16000000, Mode::Normal));
}

void test_send_standard_frame_encoding() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  Frame f = mkFrame(0x7DF, 8);
  f.data[0] = 0x02;
  f.data[1] = 0x01;
  f.data[2] = 0x0D;
  TEST_ASSERT_TRUE(mcp.send(f));
  TEST_ASSERT_EQUAL(1, chip.sent_count);
  TEST_ASSERT_EQUAL_HEX8(0xFB, chip.last_sent[0]);  // SIDH
  TEST_ASSERT_EQUAL_HEX8(0xE0, chip.last_sent[1]);  // SIDL, IDE clear
  TEST_ASSERT_EQUAL_HEX8(0x08, chip.last_sent[4]);  // DLC
  TEST_ASSERT_EQUAL_HEX8(0x02, chip.last_sent[5]);
  TEST_ASSERT_EQUAL_HEX8(0x0D, chip.last_sent[7]);
}

void test_send_extended_frame_encoding() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  Frame f = mkFrame(0x18DAF110, 3, true);
  TEST_ASSERT_TRUE(mcp.send(f));
  TEST_ASSERT_EQUAL_HEX8(0xC6, chip.last_sent[0]);
  TEST_ASSERT_EQUAL_HEX8(0xCA, chip.last_sent[1]);  // IDE set
  TEST_ASSERT_EQUAL_HEX8(0xF1, chip.last_sent[2]);
  TEST_ASSERT_EQUAL_HEX8(0x10, chip.last_sent[3]);
  TEST_ASSERT_EQUAL_HEX8(0x03, chip.last_sent[4]);
}

void test_listen_only_never_transmits() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::ListenOnly);
  TEST_ASSERT_FALSE(mcp.send(mkFrame(0x7DF, 8)));
  TEST_ASSERT_EQUAL(0, chip.sent_count);
}

void test_send_rejects_oversize_dlc() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  Frame f = mkFrame(0x7DF, 8);
  f.dlc = 9;
  TEST_ASSERT_FALSE(mcp.send(f));
}

void test_unacked_frames_are_aborted_not_wedged() {
  FakeChip chip;
  chip.acked = false;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  TEST_ASSERT_TRUE(mcp.send(mkFrame(0x7DF, 8)));
  TEST_ASSERT_TRUE(mcp.send(mkFrame(0x7DF, 8)));
  TEST_ASSERT_TRUE(mcp.send(mkFrame(0x7DF, 8)));
  TEST_ASSERT_EQUAL(0, (int)mcp.txAborted());
  TEST_ASSERT_TRUE(mcp.send(mkFrame(0x7DF, 8)));  // buffers full: abort the stuck ones, then send
  TEST_ASSERT_EQUAL(1, (int)mcp.txAborted());
  TEST_ASSERT_EQUAL(4, chip.sent_count);
}

void test_receive_none_pending() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  Frame f;
  TEST_ASSERT_FALSE(mcp.receive(f));
}

void test_receive_standard_frame() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  // 0x7E8, 8 bytes: 04 41 0D 3C 00 00 00 00
  const uint8_t raw[13] = {0xFD, 0x00, 0, 0, 0x08, 0x04, 0x41, 0x0D, 0x3C, 0, 0, 0, 0};
  chip.injectRx(0, raw);
  Frame f;
  TEST_ASSERT_TRUE(mcp.receive(f));
  TEST_ASSERT_EQUAL_HEX32(0x7E8, f.id);
  TEST_ASSERT_FALSE(f.extended);
  TEST_ASSERT_FALSE(f.rtr);
  TEST_ASSERT_EQUAL(8, f.dlc);
  TEST_ASSERT_EQUAL_HEX8(0x41, f.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x3C, f.data[3]);
  TEST_ASSERT_FALSE(mcp.receive(f));  // flag cleared by the read
}

void test_receive_extended_frame() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  const uint8_t raw[13] = {0xC6, 0xCA, 0xF1, 0x10, 0x02, 0x11, 0x22, 0, 0, 0, 0, 0, 0};
  chip.injectRx(1, raw);
  Frame f;
  TEST_ASSERT_TRUE(mcp.receive(f));
  TEST_ASSERT_EQUAL_HEX32(0x18DAF110, f.id);
  TEST_ASSERT_TRUE(f.extended);
  TEST_ASSERT_EQUAL(2, f.dlc);
  TEST_ASSERT_EQUAL_HEX8(0x22, f.data[1]);
}

void test_receive_clamps_dlc_and_flags_rtr() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  const uint8_t raw[13] = {0x20, 0x10, 0, 0, 0x0F, 1, 2, 3, 4, 5, 6, 7, 8};  // SRR set: remote frame, DLC 15
  chip.injectRx(0, raw);
  Frame f;
  TEST_ASSERT_TRUE(mcp.receive(f));
  TEST_ASSERT_TRUE(f.rtr);
  TEST_ASSERT_EQUAL(8, f.dlc);
  TEST_ASSERT_EQUAL_HEX8(0, f.data[0]);
}

void test_receive_drains_both_buffers_in_order() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  const uint8_t a[13] = {0xFD, 0x00, 0, 0, 0x01, 0xAA, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t b[13] = {0xFD, 0x20, 0, 0, 0x01, 0xBB, 0, 0, 0, 0, 0, 0, 0};
  chip.injectRx(0, a);
  chip.injectRx(1, b);
  Frame f;
  TEST_ASSERT_TRUE(mcp.receive(f));
  TEST_ASSERT_EQUAL_HEX8(0xAA, f.data[0]);
  TEST_ASSERT_TRUE(mcp.receive(f));
  TEST_ASSERT_EQUAL_HEX8(0xBB, f.data[0]);
  TEST_ASSERT_FALSE(mcp.receive(f));
}

void test_loopback_round_trip() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Loopback);
  Frame tx = mkFrame(0x123, 5);
  TEST_ASSERT_TRUE(mcp.send(tx));
  Frame rx;
  TEST_ASSERT_TRUE(mcp.receive(rx));
  TEST_ASSERT_EQUAL_HEX32(0x123, rx.id);
  TEST_ASSERT_EQUAL(5, rx.dlc);
  TEST_ASSERT_EQUAL_MEMORY(tx.data, rx.data, 5);
}

void test_loopback_extended_round_trip() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Loopback);
  Frame tx = mkFrame(0x1FFFFFFF, 8, true);
  TEST_ASSERT_TRUE(mcp.send(tx));
  Frame rx;
  TEST_ASSERT_TRUE(mcp.receive(rx));
  TEST_ASSERT_EQUAL_HEX32(0x1FFFFFFF, rx.id);
  TEST_ASSERT_TRUE(rx.extended);
  TEST_ASSERT_EQUAL_MEMORY(tx.data, rx.data, 8);
}

void test_poll_counts_and_clears_sticky_errors() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  chip.reg[0x2D] = 0x40;  // RX0OVR
  chip.reg[0x2C] |= 0x20;  // ERRIF
  mcp.poll();
  TEST_ASSERT_EQUAL(1, (int)mcp.rxOverruns());
  TEST_ASSERT_EQUAL(1, (int)mcp.busErrors());
  mcp.poll();  // flags were cleared: nothing new
  TEST_ASSERT_EQUAL(1, (int)mcp.rxOverruns());
  TEST_ASSERT_EQUAL(1, (int)mcp.busErrors());
}

void test_bus_off_and_error_counters() {
  FakeChip chip;
  Mcp2515 mcp(chip);
  mcp.begin(500000, 16000000, Mode::Normal);
  TEST_ASSERT_FALSE(mcp.busOff());
  chip.reg[0x2D] = 0x20;
  chip.reg[0x1C] = 255;
  chip.reg[0x1D] = 7;
  TEST_ASSERT_TRUE(mcp.busOff());
  TEST_ASSERT_EQUAL(255, mcp.tec());
  TEST_ASSERT_EQUAL(7, mcp.rec());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_timing_matches_proven_values);
  RUN_TEST(test_timing_rejects_unreachable_bitrates);
  RUN_TEST(test_begin_programs_timing_filters_and_mode);
  RUN_TEST(test_begin_selects_each_mode);
  RUN_TEST(test_begin_fails_without_a_chip);
  RUN_TEST(test_begin_fails_when_writes_do_not_land);
  RUN_TEST(test_begin_rejects_unreachable_bitrate);
  RUN_TEST(test_send_standard_frame_encoding);
  RUN_TEST(test_send_extended_frame_encoding);
  RUN_TEST(test_listen_only_never_transmits);
  RUN_TEST(test_send_rejects_oversize_dlc);
  RUN_TEST(test_unacked_frames_are_aborted_not_wedged);
  RUN_TEST(test_receive_none_pending);
  RUN_TEST(test_receive_standard_frame);
  RUN_TEST(test_receive_extended_frame);
  RUN_TEST(test_receive_clamps_dlc_and_flags_rtr);
  RUN_TEST(test_receive_drains_both_buffers_in_order);
  RUN_TEST(test_loopback_round_trip);
  RUN_TEST(test_loopback_extended_round_trip);
  RUN_TEST(test_poll_counts_and_clears_sticky_errors);
  RUN_TEST(test_bus_off_and_error_counters);
  return UNITY_END();
}
