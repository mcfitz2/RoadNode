#include <string.h>
#include <unity.h>

#include "fake_bus.h"
#include "obd/obd_manager.h"
#include "obd/obd_pids.h"
#include "obd/vehicle_profiles.h"

using namespace roadnode::obd;

void setUp() {}
void tearDown() {}

static void near(float expect, float got) { TEST_ASSERT_FLOAT_WITHIN(0.01f, expect, got); }

// ---- decoders ----
void test_decode_speed() {
  float v;
  uint8_t d[] = {0x3C};
  TEST_ASSERT_TRUE(decodeSpeed(d, 1, v));
  near(60, v);
  TEST_ASSERT_FALSE(decodeSpeed(d, 0, v));
}
void test_decode_rpm() {
  float v;
  uint8_t d[] = {0x1A, 0xF8};  // 6904/4 = 1726
  TEST_ASSERT_TRUE(decodeRpm(d, 2, v));
  near(1726, v);
  TEST_ASSERT_FALSE(decodeRpm(d, 1, v));
}
void test_decode_temps() {
  float v;
  uint8_t d[] = {0x7B};  // 123-40 = 83
  TEST_ASSERT_TRUE(decodeCoolantTemp(d, 1, v));
  near(83, v);
  uint8_t cold[] = {0x00};
  TEST_ASSERT_TRUE(decodeIntakeTemp(cold, 1, v));
  near(-40, v);
}
void test_decode_percent() {
  float v;
  uint8_t d[] = {0xFF};
  TEST_ASSERT_TRUE(decodeEngineLoad(d, 1, v));
  near(100, v);
  uint8_t h[] = {0x80};
  TEST_ASSERT_TRUE(decodeFuelLevel(h, 1, v));
  near(50.196f, v);
}
void test_decode_generic_dispatch() {
  float v;
  uint8_t d[] = {0x50};
  TEST_ASSERT_TRUE(decodePid(PID_SPEED, d, 1, v));
  near(80, v);
  TEST_ASSERT_FALSE(decodePid(0x99, d, 1, v));
}

// ---- capability table ----
void test_capability_bitmask() {
  CapabilityTable t;
  uint8_t m[] = {0xBE, 0x3F, 0xA8, 0x13};
  TEST_ASSERT_TRUE(t.load(0x00, m, 4));
  TEST_ASSERT_TRUE(t.supported(0x01));
  TEST_ASSERT_FALSE(t.supported(0x02));
  TEST_ASSERT_TRUE(t.supported(0x04));
  TEST_ASSERT_TRUE(t.supported(0x0D));
  TEST_ASSERT_FALSE(t.supported(0x0A));
  TEST_ASSERT_TRUE(t.supported(0x0B));
  TEST_ASSERT_TRUE(t.supported(0x0C));
  TEST_ASSERT_TRUE(t.supported(0x20));
  TEST_ASSERT_TRUE(t.hasNextRange(0x00));
}
void test_capability_unknown_range_not_assumed() {
  CapabilityTable t;
  TEST_ASSERT_FALSE(t.rangeKnown(0x2F));
  TEST_ASSERT_FALSE(t.supported(0x2F));
  TEST_ASSERT_FALSE(t.hasNextRange(0x00));
}
void test_capability_rejects_bad_input() {
  CapabilityTable t;
  uint8_t m[] = {0xFF, 0xFF, 0xFF};
  TEST_ASSERT_FALSE(t.load(0x00, m, 3));
  uint8_t m4[] = {0, 0, 0, 0};
  TEST_ASSERT_FALSE(t.load(0x05, m4, 4));
}
void test_capability_range_indexing_and_tail() {
  CapabilityTable t;
  uint8_t m[] = {0x00, 0x00, 0x00, 0x01};
  t.load(0x00, m, 4);
  TEST_ASSERT_TRUE(t.rangeKnown(0x20));   // PID 0x20 lives in the 00 range
  TEST_ASSERT_FALSE(t.rangeKnown(0x21));  // 0x21 is in the unloaded 20 range
  t.markRemainingUnsupported(0x00);
  TEST_ASSERT_TRUE(t.rangeKnown(0x42));
  TEST_ASSERT_FALSE(t.supported(0x42));
}
void test_capability_second_range() {
  CapabilityTable t;
  uint8_t m[] = {0x00, 0x00, 0x00, 0x01};
  uint8_t m2[] = {0x04, 0x00, 0x00, 0x00};  // PID 0x20+6 = 0x26
  t.load(0x00, m, 4);
  t.load(0x20, m2, 4);
  TEST_ASSERT_TRUE(t.supported(0x26));
  TEST_ASSERT_FALSE(t.supported(0x2F));
  TEST_ASSERT_FALSE(t.hasNextRange(0x20));
}

// ---- manager ----
struct Rig {
  FakeBus bus;
  ObdManager obd;
  Rig() : obd(bus, genericProfile()) { obd.enableTransmit(true); }
};

void test_gate_blocks_transmission() {
  FakeBus bus;
  ObdManager obd(bus, genericProfile());
  Response r;
  TEST_ASSERT_FALSE(obd.transmitEnabled());
  TEST_ASSERT_EQUAL(Status::Disabled, obd.request(1, 0x0D, r));
  float v;
  TEST_ASSERT_EQUAL(Status::Disabled, obd.readPid(0x0D, v));
  TEST_ASSERT_EQUAL(Status::Disabled, obd.discover());
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Disabled, obd.readVin(vin));
  TEST_ASSERT_EQUAL(0, (int)bus.sent.size());
}

void test_request_framing() {
  Rig r;
  r.bus.push(0x7E8, {3, 0x41, 0x0D, 60});
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.readPid(0x0D, v));
  near(60, v);
  TEST_ASSERT_EQUAL(1, (int)r.bus.sent.size());
  TEST_ASSERT_EQUAL_HEX32(0x7DF, r.bus.sent[0].id);
  TEST_ASSERT_EQUAL(8, r.bus.sent[0].dlc);
  TEST_ASSERT_EQUAL_HEX8(0x02, r.bus.sent[0].data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x01, r.bus.sent[0].data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x0D, r.bus.sent[0].data[2]);
}

void test_timeout_when_silent() {
  Rig r;
  float v;
  TEST_ASSERT_EQUAL(Status::Timeout, r.obd.readPid(0x0D, v));
}

void test_ignores_unrelated_traffic() {
  Rig r;
  r.bus.push(0x123, {3, 0x41, 0x0D, 99});         // not an ECU response ID
  r.bus.push(0x7E8, {3, 0x41, 0x0C, 5});          // other PID
  r.bus.push(0x7E9, {3, 0x41, 0x0D, 42});         // the answer, from ECU 2
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.readPid(0x0D, v));
  near(42, v);
}

void test_noise_cannot_extend_timeout_forever() {
  Rig r;
  for (int i = 0; i < 500; i++) r.bus.push(0x123, {1, 2, 3});
  float v;
  TEST_ASSERT_EQUAL(Status::Timeout, r.obd.readPid(0x0D, v));
}

void test_negative_response() {
  Rig r;
  r.bus.push(0x7E8, {3, 0x7F, 0x01, 0x12});
  Response resp;
  TEST_ASSERT_EQUAL(Status::NegativeResponse, r.obd.request(1, 0x0D, resp));
  TEST_ASSERT_EQUAL_HEX8(0x12, resp.nrc);
}

void test_response_pending_then_answer() {
  Rig r;
  r.bus.push(0x7E8, {3, 0x7F, 0x01, 0x78});
  r.bus.push(0x7E8, {3, 0x41, 0x0D, 7});
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.readPid(0x0D, v));
  near(7, v);
}

void test_send_failure() {
  Rig r;
  r.bus.fail_send = true;
  float v;
  TEST_ASSERT_EQUAL(Status::BusError, r.obd.readPid(0x0D, v));
}

void test_profile_response_id_filter() {
  static const VehicleProfile p = {"fixed", Protocol::Can11bit, 500000, 0x7E0, 0x7E8, 1000, 200, QUIRK_NONE};
  FakeBus bus;
  ObdManager obd(bus, p);
  obd.enableTransmit(true);
  bus.push(0x7E9, {3, 0x41, 0x0D, 11});  // wrong ECU, ignored
  bus.push(0x7E8, {3, 0x41, 0x0D, 22});
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, obd.readPid(0x0D, v));
  near(22, v);
  TEST_ASSERT_EQUAL_HEX32(0x7E0, bus.sent[0].id);
}

void test_discovery_and_poller_skips_unsupported() {
  Rig r;
  // 00 range: PID 0x0D (byte 1, 0x08) and a next range (bit 0)
  r.bus.push(0x7E8, {6, 0x41, 0x00, 0x00, 0x08, 0x00, 0x01});  // byte1 0x08 = bit 12 = PID 0x0D
  r.bus.push(0x7E8, {6, 0x41, 0x20, 0x00, 0x00, 0x00, 0x00});  // nothing more, no next range
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.discover());
  TEST_ASSERT_EQUAL(2, (int)r.bus.sent.size());
  TEST_ASSERT_TRUE(r.obd.capabilities().supported(0x0D));
  TEST_ASSERT_FALSE(r.obd.capabilities().supported(0x0C));

  size_t before = r.bus.sent.size();
  float v;
  TEST_ASSERT_EQUAL(Status::Unsupported, r.obd.readPid(0x0C, v));
  TEST_ASSERT_EQUAL((int)before, (int)r.bus.sent.size());  // skipped without touching the bus
}

void test_discovery_no_ecu() {
  Rig r;
  TEST_ASSERT_EQUAL(Status::Timeout, r.obd.discover());
  TEST_ASSERT_FALSE(r.obd.capabilities().supported(0x0D));
}

// ---- ISO-TP / VIN ----
static const char* VIN = "1HGCM82633A004352";  // check digit '3' at position 9

static void queueVinReply(FakeBus& bus, bool with_count) {
  uint8_t msg[20];
  size_t n = 0;
  msg[n++] = 0x49;
  msg[n++] = 0x02;
  if (with_count) msg[n++] = 0x01;
  memcpy(msg + n, VIN, 17);
  n += 17;
  // first frame
  roadnode::obd::CanFrame ff;
  ff.id = 0x7E8;
  ff.dlc = 8;
  ff.data[0] = 0x10 | (n >> 8);
  ff.data[1] = n & 0xFF;
  memcpy(ff.data + 2, msg, 6);
  bus.inbox.push_back(ff);
  size_t off = 6;
  uint8_t seq = 1;
  while (off < n) {
    roadnode::obd::CanFrame cf;
    cf.id = 0x7E8;
    cf.dlc = 8;
    cf.data[0] = 0x20 | (seq++ & 0xF);
    size_t c = n - off > 7 ? 7 : n - off;
    memcpy(cf.data + 1, msg + off, c);
    off += c;
    bus.inbox.push_back(cf);
  }
}

void test_vin_multiframe() {
  Rig r;
  queueVinReply(r.bus, true);
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.readVin(vin));
  TEST_ASSERT_EQUAL_STRING(VIN, vin);
  TEST_ASSERT_TRUE(vinValid(vin));
  // request + flow control
  TEST_ASSERT_EQUAL(2, (int)r.bus.sent.size());
  TEST_ASSERT_EQUAL_HEX32(0x7E0, r.bus.sent[1].id);  // physical ID of the 0x7E8 responder
  TEST_ASSERT_EQUAL_HEX8(0x30, r.bus.sent[1].data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x09, r.bus.sent[0].data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x02, r.bus.sent[0].data[2]);
}

void test_vin_without_count_byte() {
  Rig r;
  queueVinReply(r.bus, false);
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Ok, r.obd.readVin(vin));
  TEST_ASSERT_EQUAL_STRING(VIN, vin);
}

void test_multiframe_bad_sequence() {
  Rig r;
  queueVinReply(r.bus, true);
  r.bus.inbox.erase(r.bus.inbox.begin() + 1);  // drop first consecutive frame
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Malformed, r.obd.readVin(vin));
}

void test_multiframe_truncated_times_out() {
  Rig r;
  queueVinReply(r.bus, true);
  r.bus.inbox.pop_back();
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Timeout, r.obd.readVin(vin));
}

void test_vin_validation() {
  TEST_ASSERT_TRUE(vinValid("1HGCM82633A004352"));
  TEST_ASSERT_FALSE(vinValid("1HGCM82643A004352"));  // wrong check digit
  TEST_ASSERT_TRUE(vinWellFormed("1HGCM82643A004352"));
  TEST_ASSERT_FALSE(vinValid("1HGCM8263IA004352"));  // contains I
  TEST_ASSERT_FALSE(vinValid("1HGCM82633A00435"));   // 16 chars
  TEST_ASSERT_FALSE(vinValid(nullptr));
  TEST_ASSERT_TRUE(vinValid("11111111111111111"));   // classic X-free check: sum 89%11=1 -> '1'
}

// ---- profiles ----
void test_generic_profile() {
  const VehicleProfile& p = genericProfile();
  TEST_ASSERT_EQUAL_HEX32(0x7DF, p.request_id);
  TEST_ASSERT_EQUAL_UINT32(500000, p.bitrate_bps);
  TEST_ASSERT_EQUAL(&p, &selectProfile("1HGCM82633A004352"));
  TEST_ASSERT_EQUAL(&p, &selectProfile(nullptr));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_decode_speed);
  RUN_TEST(test_decode_rpm);
  RUN_TEST(test_decode_temps);
  RUN_TEST(test_decode_percent);
  RUN_TEST(test_decode_generic_dispatch);
  RUN_TEST(test_capability_bitmask);
  RUN_TEST(test_capability_unknown_range_not_assumed);
  RUN_TEST(test_capability_rejects_bad_input);
  RUN_TEST(test_capability_range_indexing_and_tail);
  RUN_TEST(test_capability_second_range);
  RUN_TEST(test_gate_blocks_transmission);
  RUN_TEST(test_request_framing);
  RUN_TEST(test_timeout_when_silent);
  RUN_TEST(test_ignores_unrelated_traffic);
  RUN_TEST(test_noise_cannot_extend_timeout_forever);
  RUN_TEST(test_negative_response);
  RUN_TEST(test_response_pending_then_answer);
  RUN_TEST(test_send_failure);
  RUN_TEST(test_profile_response_id_filter);
  RUN_TEST(test_discovery_and_poller_skips_unsupported);
  RUN_TEST(test_discovery_no_ecu);
  RUN_TEST(test_vin_multiframe);
  RUN_TEST(test_vin_without_count_byte);
  RUN_TEST(test_multiframe_bad_sequence);
  RUN_TEST(test_multiframe_truncated_times_out);
  RUN_TEST(test_vin_validation);
  RUN_TEST(test_generic_profile);
  return UNITY_END();
}
