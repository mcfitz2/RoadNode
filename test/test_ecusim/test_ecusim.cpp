#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "ecusim/ecu_commands.h"
#include "ecusim/ecu_sim.h"
#include "obd/obd_manager.h"
#include "obd/obd_pids.h"
#include "obd/vehicle_profiles.h"

using namespace roadnode::obd;
using namespace roadnode::ecusim;

void setUp() {}
void tearDown() {}

// Wires ObdManager straight to the simulated ECU: every frame sent is fed to the sim,
// and the sim's replies are what receive() returns.
class SimBus : public CanBus {
public:
  EcuSim ecu;
  uint32_t now = 0;
  bool send(const CanFrame& f) override {
    ecu.update(now);
    ecu.onFrame(f);
    return true;
  }
  // Time advances 1 ms at a time so replies held back by the simulator become ready.
  bool receive(CanFrame& f, uint32_t timeout_ms) override {
    for (uint32_t t = 0; t <= timeout_ms; t++) {
      ecu.update(now);
      if (ecu.nextFrame(f)) {
        now += 1;
        return true;
      }
      now += 1;
    }
    return false;
  }
  uint32_t nowMs() override { return now; }
};

static ObdManager makeManager(SimBus& bus) {
  ObdManager m(bus, genericProfile());
  m.enableTransmit(true);
  return m;
}

void test_discovery_finds_supported_pids() {
  SimBus bus;
  ObdManager m = makeManager(bus);
  TEST_ASSERT_EQUAL(Status::Ok, m.discover());
  for (uint8_t pid : {0x04, 0x05, 0x0C, 0x0D, 0x0F, 0x2F, 0x42}) TEST_ASSERT_TRUE(m.capabilities().supported(pid));
  TEST_ASSERT_FALSE(m.capabilities().supported(0x11));
  TEST_ASSERT_FALSE(m.capabilities().supported(0x61));
}

void test_pid_values_round_trip() {
  SimBus bus;
  EcuScenario& s = bus.ecu.scenario();
  s.speed_kmh = 88;
  s.rpm = 2500;
  s.coolant_c = 95;
  s.load_pct = 40;
  s.intake_c = 31;
  s.fuel_pct = 62;
  s.module_volts = 12.6f;
  ObdManager m = makeManager(bus);
  m.discover();
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_SPEED, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 88, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_RPM, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 2500, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_COOLANT_TEMP, v));
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 95, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_ENGINE_LOAD, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 40, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_INTAKE_TEMP, v));
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 31, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_FUEL_LEVEL, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 62, v);
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_MODULE_VOLTAGE, v));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.6f, v);
}

void test_dtcs_multiframe_with_flow_control() {
  SimBus bus;
  DtcSet& d = bus.ecu.scenario().stored;
  for (int i = 0; i < 12; i++) d.raw[d.count++] = 0x0300 + i;  // P0300..P030B
  bus.ecu.scenario().pending.raw[0] = 0x0420;
  bus.ecu.scenario().pending.count = 1;
  ObdManager m = makeManager(bus);
  DtcList l;
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Stored, l));
  TEST_ASSERT_EQUAL(12, l.count);
  TEST_ASSERT_EQUAL_STRING("P0300", l.codes[0].code);
  TEST_ASSERT_EQUAL_STRING("P030B", l.codes[11].code);
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Pending, l));
  TEST_ASSERT_EQUAL(1, l.count);
  TEST_ASSERT_EQUAL_STRING("P0420", l.codes[0].code);
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Permanent, l));
  TEST_ASSERT_EQUAL(0, l.count);
}

void test_vin_read_and_unsupported() {
  SimBus bus;
  ObdManager m = makeManager(bus);
  char vin[18];
  TEST_ASSERT_EQUAL(Status::Ok, m.readVin(vin));
  TEST_ASSERT_EQUAL_STRING("1HGCM82633A004352", vin);
  TEST_ASSERT_TRUE(vinValid(vin));
  bus.ecu.scenario().vin_supported = false;
  TEST_ASSERT_EQUAL(Status::NegativeResponse, m.readVin(vin));
}

void test_silent_and_ignored_modes_time_out() {
  SimBus bus;
  ObdManager m = makeManager(bus);
  DtcList l;
  bus.ecu.scenario().ignore_modes = 1u << 0x0A;  // like many real ECUs
  TEST_ASSERT_EQUAL(Status::Timeout, m.readDtcs(DtcMode::Permanent, l));
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Stored, l));
  bus.ecu.scenario().silent = true;
  float v;
  TEST_ASSERT_EQUAL(Status::Timeout, m.readPid(PID_SPEED, v));
}

void test_unsupported_pid_negative_response() {
  SimBus bus;
  ObdManager m = makeManager(bus);
  Response r;
  TEST_ASSERT_EQUAL(Status::NegativeResponse, m.request(0x01, 0x11, r));
  TEST_ASSERT_EQUAL_HEX8(0x12, r.nrc);
}

void test_forbidden_mode_is_counted_and_refused() {
  SimBus bus;
  // Bypass ObdManager's allowlist: a raw mode 04 frame must be flagged by the sim.
  CanFrame f;
  f.id = EcuSim::FUNCTIONAL_REQUEST_ID;
  f.dlc = 8;
  f.data[0] = 1;
  f.data[1] = 0x04;
  bus.ecu.onFrame(f);
  TEST_ASSERT_EQUAL(1, bus.ecu.forbiddenRequests());
  TEST_ASSERT_EQUAL_HEX8(0x04, bus.ecu.lastForbiddenMode());
  // The real manager never sends one.
  ObdManager m = makeManager(bus);
  Response r;
  TEST_ASSERT_EQUAL(Status::Forbidden, m.requestMode(0x04, r));
  TEST_ASSERT_EQUAL(1, bus.ecu.forbiddenRequests());
}

void test_drive_cycle_distance_matches_integral() {
  EcuSim ecu;
  ecu.scenario().drive_cycle = true;
  for (uint32_t t = 0; t <= EcuSim::CYCLE_MS; t += 100) ecu.update(t);
  // 20 s ramp up avg 30 km/h + 60 s at 60 + 20 s ramp down avg 30 km/h
  double expect = (30.0 * 20 + 60.0 * 60 + 30.0 * 20) / 3.6;
  TEST_ASSERT_FLOAT_WITHIN(5.0f, (float)expect, (float)ecu.distanceM());
  TEST_ASSERT_EQUAL_FLOAT(0, ecu.scenario().speed_kmh);
}

void test_reply_delay_within_and_beyond_timeout() {
  SimBus bus;
  ObdManager m = makeManager(bus);  // generic profile: 200 ms per request
  float v;
  bus.ecu.scenario().speed_kmh = 50;
  bus.ecu.scenario().reply_delay_ms = 100;
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_SPEED, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50, v);
  bus.ecu.scenario().reply_delay_ms = 500;
  TEST_ASSERT_EQUAL(Status::Timeout, m.readPid(PID_SPEED, v));
}

void test_response_pending_frames_are_skipped() {
  SimBus bus;
  bus.ecu.scenario().pending_frames = 3;
  bus.ecu.scenario().speed_kmh = 33;
  ObdManager m = makeManager(bus);
  float v;
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_SPEED, v));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 33, v);
  DtcList l;
  bus.ecu.scenario().stored.raw[0] = 0x0301;
  bus.ecu.scenario().stored.count = 1;
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Stored, l));
  TEST_ASSERT_EQUAL(1, l.count);
}

void test_short_frame_is_malformed_then_recovers() {
  SimBus bus;
  ObdManager m = makeManager(bus);
  float v;
  bus.ecu.scenario().corrupt = Corrupt::ShortFrame;
  TEST_ASSERT_EQUAL(Status::Malformed, m.readPid(PID_SPEED, v));
  bus.ecu.scenario().corrupt = Corrupt::None;
  TEST_ASSERT_EQUAL(Status::Ok, m.readPid(PID_SPEED, v));
}

void test_dropped_consecutive_frame_is_malformed_then_recovers() {
  SimBus bus;
  DtcSet& d = bus.ecu.scenario().stored;
  for (int i = 0; i < 12; i++) d.raw[d.count++] = 0x0300 + i;
  ObdManager m = makeManager(bus);
  DtcList l;
  bus.ecu.scenario().corrupt = Corrupt::SkipConsecutive;
  TEST_ASSERT_EQUAL(Status::Malformed, m.readDtcs(DtcMode::Stored, l));
  // Leftover consecutive frames from the broken reply must not confuse the next request.
  bus.ecu.scenario().corrupt = Corrupt::None;
  TEST_ASSERT_EQUAL(Status::Ok, m.readDtcs(DtcMode::Stored, l));
  TEST_ASSERT_EQUAL(12, l.count);
}

void test_commands_change_scenario() {
  EcuSim ecu;
  char reply[160];
  char c1[] = "dtc 03 0301";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c1, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL(1, ecu.scenario().stored.count);
  TEST_ASSERT_EQUAL_HEX16(0x0301, ecu.scenario().stored.raw[0]);
  char c2[] = "vin none";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c2, reply, sizeof(reply)));
  TEST_ASSERT_FALSE(ecu.scenario().vin_supported);
  char c3[] = "ignore 0A";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c3, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL_HEX32(1u << 10, ecu.scenario().ignore_modes);
  char c4[] = "speed 55";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c4, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL_FLOAT(55, ecu.scenario().speed_kmh);
  char c5[] = "status";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c5, reply, sizeof(reply)));
  char c6[] = "delay 120";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c6, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL(120, ecu.scenario().reply_delay_ms);
  char c7[] = "pending 2";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c7, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL(2, ecu.scenario().pending_frames);
  char c8[] = "corrupt skipcf";
  TEST_ASSERT_TRUE(handleEcuCommand(ecu, c8, reply, sizeof(reply)));
  TEST_ASSERT_EQUAL((int)Corrupt::SkipConsecutive, (int)ecu.scenario().corrupt);
  char c9[] = "corrupt bogus";
  TEST_ASSERT_FALSE(handleEcuCommand(ecu, c9, reply, sizeof(reply)));
  char c10[] = "pending 99";
  TEST_ASSERT_FALSE(handleEcuCommand(ecu, c10, reply, sizeof(reply)));
  char bad[] = "vin SHORT";
  TEST_ASSERT_FALSE(handleEcuCommand(ecu, bad, reply, sizeof(reply)));
  char nope[] = "frobnicate 1";
  TEST_ASSERT_FALSE(handleEcuCommand(ecu, nope, reply, sizeof(reply)));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_discovery_finds_supported_pids);
  RUN_TEST(test_pid_values_round_trip);
  RUN_TEST(test_dtcs_multiframe_with_flow_control);
  RUN_TEST(test_vin_read_and_unsupported);
  RUN_TEST(test_silent_and_ignored_modes_time_out);
  RUN_TEST(test_unsupported_pid_negative_response);
  RUN_TEST(test_forbidden_mode_is_counted_and_refused);
  RUN_TEST(test_drive_cycle_distance_matches_integral);
  RUN_TEST(test_reply_delay_within_and_beyond_timeout);
  RUN_TEST(test_response_pending_frames_are_skipped);
  RUN_TEST(test_short_frame_is_malformed_then_recovers);
  RUN_TEST(test_dropped_consecutive_frame_is_malformed_then_recovers);
  RUN_TEST(test_commands_change_scenario);
  return UNITY_END();
}
