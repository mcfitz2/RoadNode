#include <string>
#include <thread>
#include <unity.h>
#include <vector>

#include "fake_bus.h"
#include "fake_kv.h"
#include "fake_store.h"
#include "vehicle/vehicle_poller.h"

using namespace roadnode::vehicle;
using namespace roadnode::obd;
using roadnode::storage::VehicleStorage;
using roadnode::storage::VehicleRecord;

// Minimal ECU: answers Mode 01 PIDs 00 (capabilities), 0C, 0D, 42 on 0x7E8.
struct Ecu {
  bool on = true;
  uint8_t speed = 0;
  uint16_t rpm_raw = 0;  // quarter rpm
  uint16_t batt_mv = 12600;
  bool support_battery = true;
  bool neg_rpm = false;
  bool malformed_speed = false;

  void attach(FakeBus& bus) {
    bus.on_send = [this, &bus](const CanFrame& f) {
      if (!on || f.id != 0x7DF || f.data[1] != 0x01) return;
      switch (f.data[2]) {
        case 0x00:  // 0C, 0D (and 42 is in the next range)
          bus.push(0x7E8, {6, 0x41, 0x00, 0x00, 0x18, 0x00, 0x01});
          break;
        case 0x20:
          bus.push(0x7E8, {6, 0x41, 0x20, 0x00, 0x00, 0x00, 0x00});
          break;
        case 0x0D:
          if (malformed_speed) bus.push(0x7E8, {2, 0x41, 0x0D});
          else bus.push(0x7E8, {3, 0x41, 0x0D, speed});
          break;
        case 0x0C:
          if (neg_rpm) bus.push(0x7E8, {3, 0x7F, 0x01, 0x12});
          else bus.push(0x7E8, {4, 0x41, 0x0C, (uint8_t)(rpm_raw >> 8), (uint8_t)rpm_raw});
          break;
        case 0x42:
          bus.push(0x7E8, {4, 0x41, 0x42, (uint8_t)(batt_mv >> 8), (uint8_t)batt_mv});
          break;
      }
    };
  }
};

// The 00-range mask above supports 0C and 0D. Voltage (0x42) is in range 40, which
// this ECU does not report, so it reads as unsupported. Tests that want it override.

struct Rig {
  FakeBus bus;
  Ecu ecu;
  FakeStore store;
  ObdManager obd;
  MileageTracker tracker;
  VehicleTelemetry telem;
  VehiclePoller poller;
  uint32_t t = 0;

  static TripConfig tc() {
    TripConfig c;
    c.end_timeout_ms = 20000;
    return c;
  }
  Rig() : obd(bus, genericProfile()), tracker(store, tc()), poller(obd, bus, tracker, telem) {
    obd.enableTransmit(true);
    ecu.attach(bus);
    tracker.begin(0);
  }
  // One poll cycle about dt_ms after the last.
  void tick(uint32_t dt_ms = 1000) {
    t += dt_ms;
    if (bus.now < t) bus.now = t;
    poller.step(bus.now);
    t = bus.now;
  }
  void run(int n, uint32_t dt_ms = 1000) {
    for (int i = 0; i < n; i++) tick(dt_ms);
  }
};

// Records every mode byte the poller transmits, passing frames on to the Rig's ECU.
struct ModeLog {
  std::vector<uint8_t> modes_sent;

  void attach(Rig& r) {
    auto ecu = r.bus.on_send;
    r.bus.on_send = [this, ecu](const CanFrame& f) {
      if (f.id == 0x7DF) modes_sent.push_back(f.data[1]);
      ecu(f);
    };
  }
};

// Layers a multi-frame Mode 09 PID 02 (VIN) reply over the Rig's ECU.
struct VinEcu {
  std::string vin = "1HGCM82633A004352";
  bool answer = true;
  bool negative = false;
  int requests = 0;

  void attach(Rig& r) {
    auto prev = r.bus.on_send;
    r.bus.on_send = [this, prev, &r](const CanFrame& f) {
      if (f.id == 0x7DF && f.data[1] == 0x09 && f.data[2] == 0x02) {
        requests++;
        if (negative) {
          CanFrame n;
          n.id = 0x7E8;
          n.dlc = 8;
          n.data[0] = 3;
          n.data[1] = 0x7F;
          n.data[2] = 0x09;
          n.data[3] = 0x12;
          r.bus.inbox.push_back(n);
        } else if (answer) {
          std::vector<uint8_t> p = {0x49, 0x02, 0x01};
          for (char c : vin) p.push_back((uint8_t)c);
          CanFrame ff;
          ff.id = 0x7E8;
          ff.dlc = 8;
          ff.data[0] = 0x10;
          ff.data[1] = (uint8_t)p.size();
          for (int i = 0; i < 6; i++) ff.data[2 + i] = p[i];
          r.bus.inbox.push_back(ff);
          uint8_t seq = 1;
          for (size_t o = 6; o < p.size(); o += 7) {
            CanFrame cf;
            cf.id = 0x7E8;
            cf.dlc = 8;
            cf.data[0] = 0x20 | (seq++ & 0x0F);
            for (size_t i = 0; i < 7 && o + i < p.size(); i++) cf.data[1 + i] = p[o + i];
            r.bus.inbox.push_back(cf);
          }
        }
        return;
      }
      prev(f);
    };
  }
};

void test_vin_read_once_stopped_and_stored() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  VinEcu v;
  v.attach(r);
  r.poller.setIdentity(&id);
  r.ecu.speed = 0;
  r.run(5);
  TEST_ASSERT_TRUE(id.hasVin());
  TEST_ASSERT_EQUAL_STRING("1HGCM82633A004352", id.vin());
  TEST_ASSERT_EQUAL(1, v.requests);  // read once, not repeated
  r.run(100);
  TEST_ASSERT_EQUAL(1, v.requests);
  TEST_ASSERT_EQUAL_STRING("RAV4", r.telem.snapshot().vehicle_id);
}

void test_vin_deferred_while_moving() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  VinEcu v;
  v.attach(r);
  r.poller.setIdentity(&id);
  r.ecu.speed = 60;
  r.run(30);
  TEST_ASSERT_EQUAL(0, v.requests);
  r.ecu.speed = 0;
  r.run(3);
  TEST_ASSERT_EQUAL(1, v.requests);
}

void test_vin_unsupported_not_retried() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  VinEcu v;
  v.negative = true;
  v.attach(r);
  r.poller.setIdentity(&id);
  r.run(200);
  TEST_ASSERT_EQUAL(1, v.requests);
  TEST_ASSERT_FALSE(id.hasVin());
}

void test_vin_silent_ecu_retries_bounded() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  VinEcu v;
  v.answer = false;
  v.attach(r);
  r.poller.setIdentity(&id);
  r.run(400);
  TEST_ASSERT_TRUE(v.requests >= 2);
  TEST_ASSERT_TRUE(v.requests <= 5);
}

void test_configured_id_overrides_mileage_record_id() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  id.setId("CAMRY");
  r.poller.setIdentity(&id);
  r.run(3);
  TEST_ASSERT_EQUAL_STRING("CAMRY", r.telem.snapshot().vehicle_id);
}

void test_vin_not_in_snapshot_bytes() {
  Rig r;
  FakeKv kv;
  roadnode::vehicle::VehicleIdentity id(kv);
  id.begin("RAV4");
  VinEcu v;
  v.attach(r);
  r.poller.setIdentity(&id);
  r.run(5);
  TEST_ASSERT_TRUE(id.hasVin());
  VehicleSnapshot s = r.telem.snapshot();
  const char* p = (const char*)&s;
  bool found = false;
  for (size_t i = 0; i + 17 <= sizeof(s); i++)
    if (memcmp(p + i, "1HGCM82633A004352", 17) == 0) found = true;
  TEST_ASSERT_FALSE(found);
}

void setUp() {}
void tearDown() {}

void test_normal_driving() {
  Rig r;
  r.ecu.speed = 36;
  r.ecu.rpm_raw = 3200;  // 800 rpm
  r.run(60);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_TRUE(s.obd_connected);
  TEST_ASSERT_TRUE(s.vehicle_active);
  TEST_ASSERT_TRUE(s.engine_running);
  TEST_ASSERT_TRUE(s.has_speed);
  TEST_ASSERT_EQUAL_FLOAT(36, s.speed_kmh);
  TEST_ASSERT_TRUE(s.has_rpm);
  TEST_ASSERT_EQUAL_FLOAT(800, s.rpm);
  TEST_ASSERT_TRUE(s.trip_active);
  // ~59 s at 10 m/s, give or take sample timing
  TEST_ASSERT_UINT64_WITHIN(60000, 590000, s.total_mm);
  TEST_ASSERT_EQUAL_UINT64(s.total_mm, s.trip_mm);
  TEST_ASSERT_TRUE(s.ever_obd_response);
}

void test_no_can_connection_at_boot() {
  Rig r;
  r.ecu.on = false;
  r.run(30);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_FALSE(s.obd_connected);
  TEST_ASSERT_FALSE(s.vehicle_active);
  TEST_ASSERT_FALSE(s.has_speed);
  TEST_ASSERT_FALSE(s.ever_obd_response);
  TEST_ASSERT_FALSE(s.ever_can_activity);
  TEST_ASSERT_EQUAL_UINT64(0, s.total_mm);
  TEST_ASSERT_EQUAL(0, r.store.writes);
  TEST_ASSERT_FALSE(r.poller.discovered());
  TEST_ASSERT_TRUE(r.poller.stats().timeouts > 0);
}

void test_unplug_mid_trip_then_reconnect() {
  Rig r;
  r.ecu.speed = 72;  // 20 m/s
  r.ecu.rpm_raw = 8000;
  r.run(20);
  uint64_t before = r.telem.snapshot().total_mm;
  TEST_ASSERT_TRUE(before > 300000);

  r.ecu.on = false;  // cable pulled
  r.run(15);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_FALSE(s.obd_connected);
  TEST_ASSERT_FALSE(s.has_speed);
  TEST_ASSERT_FALSE(s.engine_running);
  uint64_t frozen = s.total_mm;
  TEST_ASSERT_TRUE(frozen - before < 100000);  // at most the sample in flight, nothing invented
  r.run(10);
  TEST_ASSERT_EQUAL_UINT64(frozen, r.telem.snapshot().total_mm);

  r.ecu.on = true;  // plugged back in: next poll resumes without a distance jump
  r.run(1);
  uint64_t resumed = r.telem.snapshot().total_mm;
  TEST_ASSERT_TRUE(resumed - frozen < 100000);
  r.run(10);
  s = r.telem.snapshot();
  TEST_ASSERT_TRUE(s.obd_connected);
  TEST_ASSERT_TRUE(s.has_speed);
  TEST_ASSERT_TRUE(s.total_mm - resumed > 150000);  // ~10 s at 20 m/s accrues again
}

void test_unsupported_pid_skipped_not_fatal() {
  Rig r;
  r.ecu.speed = 10;
  r.ecu.rpm_raw = 4000;
  r.run(10);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_FALSE(s.has_battery);  // 0x42 not in the capability mask
  TEST_ASSERT_TRUE(s.obd_connected);
  TEST_ASSERT_TRUE(r.poller.stats().unsupported_skips > 0);
}

void test_malformed_response_counted_and_recovers() {
  Rig r;
  r.ecu.speed = 36;
  r.ecu.rpm_raw = 0;
  r.run(5);
  r.ecu.malformed_speed = true;
  r.run(3);
  TEST_ASSERT_TRUE(r.poller.stats().malformed > 0);
  r.ecu.malformed_speed = false;
  r.run(5);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_TRUE(s.has_speed);
  TEST_ASSERT_TRUE(s.obd_connected);
}

void test_negative_response_keeps_connection() {
  Rig r;
  r.ecu.speed = 36;
  r.ecu.neg_rpm = true;
  r.run(10);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_TRUE(s.obd_connected);
  TEST_ASSERT_FALSE(s.has_rpm);
  TEST_ASSERT_TRUE(s.engine_running);  // falls back to speed > 0
  TEST_ASSERT_TRUE(r.poller.stats().negative > 0);
}

void test_bus_off_recovery_rate_limited() {
  Rig r;
  r.ecu.speed = 36;
  r.ecu.rpm_raw = 4000;
  r.run(5);
  r.bus.bus_off = true;
  r.bus.recovers = false;
  r.run(5, 200);  // 1 s of polling at 200 ms: recovery attempts bounded to ~1/s
  TEST_ASSERT_TRUE(r.bus.recover_calls >= 1 && r.bus.recover_calls <= 2);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_FALSE(s.obd_connected);
  TEST_ASSERT_FALSE(s.has_speed);
  TEST_ASSERT_EQUAL(1, (int)r.poller.stats().bus_off_events);
  uint64_t frozen = s.total_mm;
  r.run(3);
  TEST_ASSERT_EQUAL_UINT64(frozen, r.telem.snapshot().total_mm);

  r.bus.recovers = true;
  r.run(3);
  r.run(10);
  s = r.telem.snapshot();
  TEST_ASSERT_TRUE(s.obd_connected);
  TEST_ASSERT_TRUE(s.has_speed);
  TEST_ASSERT_TRUE(r.poller.discovered());  // capabilities re-learned
}

void test_vehicle_shutdown_ends_trip_and_saves() {
  Rig r;
  r.tracker.setVehicleId("RAV4");
  r.ecu.speed = 36;
  r.ecu.rpm_raw = 3200;
  r.run(30);
  TEST_ASSERT_TRUE(r.telem.snapshot().trip_active);
  r.ecu.on = false;  // ignition off, ECU and bus go quiet
  r.run(60);
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_FALSE(s.vehicle_active);
  TEST_ASSERT_FALSE(s.trip_active);
  VehicleStorage st(r.store);
  VehicleRecord rec;
  TEST_ASSERT_TRUE(st.load(rec));
  TEST_ASSERT_EQUAL_UINT64(s.total_mm, rec.total_mm);
  TEST_ASSERT_EQUAL_STRING("RAV4", s.vehicle_id);
}

void test_transmit_gate_closed_sends_nothing() {
  Rig r;
  r.obd.enableTransmit(false);
  r.run(10);
  TEST_ASSERT_EQUAL(0, (int)r.bus.sent.size());
  TEST_ASSERT_FALSE(r.telem.snapshot().obd_connected);
}

void test_snapshot_consistent_across_threads() {
  VehicleTelemetry t;
  std::thread writer([&] {
    for (uint64_t i = 1; i <= 200000; i++) {
      VehicleSnapshot s;
      s.total_mm = i;
      s.trip_mm = i;  // invariant: always equal
      t.publish(s);
    }
  });
  int torn = 0;
  for (int i = 0; i < 200000; i++) {
    VehicleSnapshot s = t.snapshot();
    if (s.total_mm != s.trip_mm) torn++;
  }
  writer.join();
  TEST_ASSERT_EQUAL(0, torn);
}

void test_poller_never_transmits_clear_or_other_write_modes() {
  Rig r;
  ModeLog d;
  d.attach(r);
  r.ecu.speed = 50;
  r.ecu.rpm_raw = 3200;
  r.run(120);
  r.ecu.speed = 0;  // stopped too, so the VIN read gets its chance
  r.run(30);
  TEST_ASSERT_FALSE(d.modes_sent.empty());
  for (uint8_t m : d.modes_sent) TEST_ASSERT_TRUE(m == 0x01 || m == 0x09);
}

void test_speed_samples_keep_spacing_while_moving() {
  Rig r;
  r.ecu.speed = 72;
  r.ecu.rpm_raw = 3200;
  r.run(100);
  // 72 km/h = 20 m/s: ~99 s of integration
  VehicleSnapshot s = r.telem.snapshot();
  TEST_ASSERT_UINT64_WITHIN(150000, 1980000, s.total_mm);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_vin_read_once_stopped_and_stored);
  RUN_TEST(test_vin_deferred_while_moving);
  RUN_TEST(test_vin_unsupported_not_retried);
  RUN_TEST(test_vin_silent_ecu_retries_bounded);
  RUN_TEST(test_configured_id_overrides_mileage_record_id);
  RUN_TEST(test_vin_not_in_snapshot_bytes);
  RUN_TEST(test_speed_samples_keep_spacing_while_moving);
  RUN_TEST(test_poller_never_transmits_clear_or_other_write_modes);
  RUN_TEST(test_normal_driving);
  RUN_TEST(test_no_can_connection_at_boot);
  RUN_TEST(test_unplug_mid_trip_then_reconnect);
  RUN_TEST(test_unsupported_pid_skipped_not_fatal);
  RUN_TEST(test_malformed_response_counted_and_recovers);
  RUN_TEST(test_negative_response_keeps_connection);
  RUN_TEST(test_bus_off_recovery_rate_limited);
  RUN_TEST(test_vehicle_shutdown_ends_trip_and_saves);
  RUN_TEST(test_transmit_gate_closed_sends_nothing);
  RUN_TEST(test_snapshot_consistent_across_threads);
  return UNITY_END();
}
