#include <initializer_list>
#include <math.h>
#include <unity.h>

#include "ecusim/ecu_sim.h"
#include "fake_store.h"
#include "obd/obd_manager.h"
#include "obd/vehicle_profiles.h"
#include "vehicle/mileage_tracker.h"
#include "vehicle/vehicle_poller.h"

// Whole vehicle stack on the host: simulated ECU -> ObdManager -> VehiclePoller -> MileageTracker ->
// storage. The ECU knows the exact distance it simulated, so the odometer can be checked against truth.
// Covers the simulated side of the mileage checks (#26, #15, #28); real CAN timing, NVS and a real car
// are not covered.

using namespace roadnode::obd;
using namespace roadnode::vehicle;
using roadnode::ecusim::EcuSim;

void setUp() {}
void tearDown() {}

// The ECU as a CanBus, stepping 1 ms at a time so delayed replies become ready.
class SimBus : public CanBus {
public:
  EcuSim ecu;
  uint32_t now = 0;
  bool send(const CanFrame& f) override {
    ecu.update(now);
    ecu.onFrame(f);
    return true;
  }
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
  // Time passing with nobody asking: the drive cycle keeps running.
  void idleUntil(uint32_t t) {
    while (now < t) {
      now += 50;
      ecu.update(now);
    }
  }
};

// One boot of the node. Destroying it without shutdown() is a power cut.
struct Node {
  ObdManager obd;
  MileageTracker tracker;
  VehicleTelemetry telem;
  VehiclePoller poller;
  SimBus& bus;
  uint32_t next_poll;

  Node(SimBus& b, FakeStore& store)
      : obd(b, genericProfile()), tracker(store), poller(obd, b, tracker, telem), bus(b), next_poll(b.now) {
    obd.enableTransmit(true);
    tracker.begin(b.now);
  }
  // Poll at the profile's interval until the bus clock reaches until_ms.
  void runUntil(uint32_t until_ms) {
    uint32_t interval = obd.profile().poll_interval_ms;
    while (bus.now < until_ms) {
      bus.idleUntil(next_poll);
      poller.step(bus.now);
      next_poll = bus.now > next_poll ? bus.now + interval : next_poll + interval;  // a slow step does not queue polls
    }
  }
};

static void startDriving(SimBus& bus) {
  bus.ecu.scenario().drive_cycle = true;
  bus.ecu.update(bus.now);
  bus.ecu.resetDistance();
}

// Relative error of the odometer against the simulated distance.
static double errorPct(const Node& n, const SimBus& bus) {
  double truth = bus.ecu.distanceM();
  return (n.tracker.totalMm() / 1000.0 - truth) / truth * 100.0;
}

void test_odometer_tracks_simulated_distance_over_many_cycles() {
  SimBus bus;
  FakeStore store;
  Node n(bus, store);
  startDriving(bus);
  n.runUntil(bus.now + 20 * EcuSim::CYCLE_MS);
  double truth = bus.ecu.distanceM();
  TEST_ASSERT_TRUE(truth > 26000);  // 20 cycles, about 1.3 km each
  // Speed is a whole km/h (OBD PID 0D) and sampled about once a second: that bounds the error.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, (float)errorPct(n, bus));
  TEST_ASSERT_TRUE(n.telem.snapshot().has_speed);
}

void test_scale_factor_applies_end_to_end() {
  SimBus bus1, bus2;
  FakeStore s1, s2;
  Node plain(bus1, s1), scaled(bus2, s2);
  TEST_ASSERT_TRUE(scaled.tracker.setScaleBp(10250));
  startDriving(bus1);
  startDriving(bus2);
  plain.runUntil(plain.bus.now + 5 * EcuSim::CYCLE_MS);
  scaled.runUntil(scaled.bus.now + 5 * EcuSim::CYCLE_MS);
  double ratio = (double)scaled.tracker.totalMm() / (double)plain.tracker.totalMm();
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.025f, (float)ratio);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 2.5f, (float)errorPct(scaled, bus2));  // truth x 1.025
}

void test_clean_shutdown_and_reboot_keep_exact_value() {
  SimBus bus;
  FakeStore store;
  uint64_t before;
  {
    Node n(bus, store);
    startDriving(bus);
    n.runUntil(bus.now + 3 * EcuSim::CYCLE_MS + 40000);  // stop mid-cruise
    TEST_ASSERT_TRUE(n.poller.shutdown(bus.now));
    before = n.tracker.totalMm();
  }
  TEST_ASSERT_TRUE(before > 0);
  Node again(bus, store);
  TEST_ASSERT_EQUAL_UINT64(before, again.tracker.totalMm());
  // Driving on after the reboot continues from the stored value.
  again.runUntil(bus.now + EcuSim::CYCLE_MS);
  TEST_ASSERT_TRUE(again.tracker.totalMm() > before + 1000000);
}

void test_power_cut_loses_at_most_one_checkpoint_interval() {
  SimBus bus;
  FakeStore store;
  uint64_t at_cut;
  {
    Node n(bus, store);
    startDriving(bus);
    n.runUntil(bus.now + 4 * EcuSim::CYCLE_MS + 70000);
    at_cut = n.tracker.totalMm();  // no shutdown(): power cut
  }
  Node again(bus, store);
  TEST_ASSERT_TRUE(again.tracker.totalMm() <= at_cut);
  // Checkpoint policy: 0.5 mile (804.672 m) or 2 minutes, whichever comes first.
  TEST_ASSERT_TRUE(at_cut - again.tracker.totalMm() <= 804672u + 100000u);
}

void test_unplugged_bus_adds_nothing_and_loses_nothing() {
  SimBus bus;
  FakeStore store;
  Node n(bus, store);
  startDriving(bus);
  n.runUntil(bus.now + EcuSim::CYCLE_MS + 30000);  // cruising
  uint64_t before = n.tracker.totalMm();
  TEST_ASSERT_TRUE(before > 0);
  bus.ecu.scenario().silent = true;  // adapter pulled: no answers, the car keeps moving
  n.runUntil(bus.now + 60000);
  uint64_t during = n.tracker.totalMm();
  TEST_ASSERT_TRUE(during >= before);                // never goes backwards
  TEST_ASSERT_TRUE(during - before < 20000);         // at most a couple of seconds of samples, no invented distance
  TEST_ASSERT_FALSE(n.telem.snapshot().has_speed);   // reads as unknown, not as a stale speed
  bus.ecu.scenario().silent = false;
  n.runUntil(bus.now + 30000);
  TEST_ASSERT_TRUE(n.tracker.totalMm() > during);    // counting again once the ECU answers
}

void test_dtc_reads_while_stopped_do_not_cost_distance() {
  SimBus bus;
  FakeStore store;
  Node n(bus, store);
  bus.ecu.scenario().stored.raw[0] = 0x0133;
  bus.ecu.scenario().stored.count = 1;
  startDriving(bus);
  n.runUntil(bus.now + 10 * EcuSim::CYCLE_MS);
  TEST_ASSERT_TRUE(n.poller.stats().dtc_reads > 0);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, (float)errorPct(n, bus));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_odometer_tracks_simulated_distance_over_many_cycles);
  RUN_TEST(test_scale_factor_applies_end_to_end);
  RUN_TEST(test_clean_shutdown_and_reboot_keep_exact_value);
  RUN_TEST(test_power_cut_loses_at_most_one_checkpoint_interval);
  RUN_TEST(test_unplugged_bus_adds_nothing_and_loses_nothing);
  RUN_TEST(test_dtc_reads_while_stopped_do_not_cost_distance);
  return UNITY_END();
}
