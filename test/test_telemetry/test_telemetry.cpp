#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>
#include <vector>

#include "telemetry/vehicle_lpp.h"

using namespace roadnode;
using namespace roadnode::telemetry;
using vehicle::VehicleSnapshot;

// Writer that follows the CayenneLPP 1.6.1 wire rules (CayenneLPP.h/.cpp):
// [channel][type][value], value = uint32(value * multiplier), big endian,
// truncated to the type's size; signed types use two's complement.
class ModelLpp {
public:
  std::vector<uint8_t> buf;
  size_t max;
  bool overflow = false;
  explicit ModelLpp(size_t max_size = 176) : max(max_size) {}

  void add(uint8_t ch, uint8_t type, size_t size, uint32_t mult, bool is_signed, float value) {
    if (buf.size() + size + 2 > max) {
      overflow = true;
      return;
    }
    bool neg = value < 0;
    if (neg) value = -value;
    uint32_t v = (uint32_t)(value * mult);
    if (is_signed && neg) v = (uint32_t)(~v + 1);
    buf.push_back(ch);
    buf.push_back(type);
    for (size_t i = 0; i < size; i++) buf.push_back((uint8_t)(v >> (8 * (size - 1 - i))));
  }
  void addGenericSensor(uint8_t ch, float v) { add(ch, 100, 4, 1, false, v); }
  void addDistance(uint8_t ch, float v) { add(ch, 130, 4, 1000, false, v); }
  void addAnalogInput(uint8_t ch, float v) { add(ch, 2, 2, 100, true, v); }
  void addDigitalInput(uint8_t ch, uint32_t v) { add(ch, 0, 1, 1, false, (float)v); }
  void addTemperature(uint8_t ch, float v) { add(ch, 103, 2, 10, true, v); }
  void addPercentage(uint8_t ch, uint32_t v) { add(ch, 120, 1, 1, false, (float)v); }
  void addDirection(uint8_t ch, uint32_t v) { add(ch, 132, 2, 1, false, (float)v); }
  void addVoltage(uint8_t ch, float v) { add(ch, 116, 2, 100, false, v); }
};

// Unity double asserts are disabled in the host build.
static void near(double expect, double got, double tol) {
  char msg[80];
  snprintf(msg, sizeof(msg), "expected %.4f got %.4f", expect, got);
  TEST_ASSERT_TRUE_MESSAGE(fabs(expect - got) <= tol, msg);
}

void setUp() {}
void tearDown() {}

static VehicleSnapshot driving() {
  VehicleSnapshot s;
  s.total_mm = 123456789000ULL;  // 123,456.789 km
  s.trip_mm = 12345678;          // 12,345.678 m
  s.has_speed = true;
  s.speed_kmh = 88;
  s.has_rpm = true;
  s.rpm = 2150;
  s.has_battery = true;
  s.battery_v = 14.2f;
  s.vehicle_active = s.engine_running = s.trip_active = s.obd_connected = true;
  return s;
}

void test_roundtrip_all_fields() {
  ModelLpp lpp;
  encodeVehicle(driving(), lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_total && d.has_trip && d.has_speed && d.has_rpm && d.has_state && d.has_battery);
  near(123456.7, d.total_km, 1e-6);  // 0.1 km resolution
  near(12345.678, d.trip_m, 0.01);
  near(88, d.speed_kmh, 1e-9);
  near(2150, d.rpm, 1e-9);
  TEST_ASSERT_EQUAL_HEX8(0x0F, d.state);
  near(14.2, d.battery_v, 0.011);
}

void test_unknown_values_omitted_not_zero() {
  VehicleSnapshot s;  // nothing live
  s.total_mm = 5000000;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_total && d.has_trip && d.has_state);
  TEST_ASSERT_FALSE(d.has_speed);
  TEST_ASSERT_FALSE(d.has_rpm);
  TEST_ASSERT_FALSE(d.has_battery);
  TEST_ASSERT_EQUAL_HEX8(0, d.state);
}

void test_size_is_small_and_fits_payload() {
  ModelLpp lpp;
  encodeVehicle(driving(), lpp);
  TEST_ASSERT_EQUAL(29, (int)lpp.buf.size());  // total 6, trip 6, speed 4, rpm 6, state 3, battery 4
  TEST_ASSERT_FALSE(lpp.overflow);
  // MeshCore's buffer is MAX_PACKET_PAYLOAD-4 = 180 bytes shared with channel 1 (battery 4, GPS 11).
  TEST_ASSERT_TRUE(lpp.buf.size() + 4 + 11 < 180);
}

void test_odometer_six_digits_miles() {
  // 999,999 miles = 1,609,342 km: the case LPP distance (max ~4,295 km) cannot carry.
  VehicleSnapshot s;
  s.total_mm = 1609342000000ULL;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  near(1609342.0, d.total_km, 0.1);
}

void test_total_exact_in_float_up_to_cap() {
  for (uint64_t km : {1ULL, 99999ULL, 241000ULL, 1000000ULL, 1600000ULL}) {
    VehicleSnapshot s;
    s.total_mm = km * 1000000ULL + 50000;  // +50 m
    ModelLpp lpp;
    encodeVehicle(s, lpp);
    DecodedVehicle d;
    TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
    near((double)km, d.total_km, 0.0001);  // floor to 0.1 km, never off by a unit
  }
}

void test_total_saturates_instead_of_wrapping() {
  VehicleSnapshot s;
  s.total_mm = 9000000000000000ULL;  // 9 million km
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  near(1677721.6, d.total_km, 0.1);
}

void test_trip_saturates() {
  VehicleSnapshot s;
  s.trip_mm = 9000000000000ULL;  // 9,000 km
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  near(4294000.0, d.trip_m, 1.0);
}

void test_state_bits_individually() {
  const struct { bool VehicleSnapshot::*f; uint8_t bit; } cases[] = {
      {&VehicleSnapshot::vehicle_active, STATE_VEHICLE_ACTIVE},
      {&VehicleSnapshot::engine_running, STATE_ENGINE_RUNNING},
      {&VehicleSnapshot::trip_active, STATE_TRIP_ACTIVE},
      {&VehicleSnapshot::obd_connected, STATE_OBD_CONNECTED}};
  for (auto& c : cases) {
    VehicleSnapshot s;
    s.*(c.f) = true;
    ModelLpp lpp;
    encodeVehicle(s, lpp);
    DecodedVehicle d;
    TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
    TEST_ASSERT_EQUAL_HEX8(c.bit, d.state);
  }
}

void test_decoder_skips_meshcore_channel_one() {
  ModelLpp lpp;
  lpp.addVoltage(1, 4.01f);  // MeshCore device battery
  lpp.add(1, 136, 9, 1, false, 0);  // a GPS record from the stock sensor manager
  encodeVehicle(driving(), lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_total && d.has_battery);
  near(14.2, d.battery_v, 0.011);  // channel 7, not MeshCore's channel 1
}

void test_decoder_rejects_truncated_and_unknown() {
  ModelLpp lpp;
  encodeVehicle(driving(), lpp);
  DecodedVehicle d;
  TEST_ASSERT_FALSE(decodeVehicle(lpp.buf.data(), lpp.buf.size() - 1, d));
  uint8_t unknown[] = {2, 0x7E, 0, 0};
  DecodedVehicle d2;
  TEST_ASSERT_FALSE(decodeVehicle(unknown, sizeof(unknown), d2));
}

void test_speed_signed_encoding_range() {
  VehicleSnapshot s;
  s.has_speed = true;
  s.speed_kmh = 255;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  near(255, d.speed_kmh, 1e-9);  // int16 holds up to 327.67
}

void test_no_location_ever_encoded() {
  ModelLpp lpp;
  encodeVehicle(driving(), lpp);
  size_t i = 0;
  while (i + 2 <= lpp.buf.size()) {
    uint8_t type = lpp.buf[i + 1];
    TEST_ASSERT_NOT_EQUAL(136, type);  // LPP_GPS
    TEST_ASSERT_NOT_EQUAL(121, type);  // altitude
    size_t size = (type == 0) ? 1 : (type == 2 || type == 116) ? 2 : 4;
    i += 2 + size;
  }
}

void test_dtcs_roundtrip_and_size() {
  VehicleSnapshot s = driving();
  s.has_dtcs = true;
  s.dtc_total = 3;
  s.dtc_count = 3;
  s.dtc_raw[0] = 0x0301; s.dtc_kind[0] = vehicle::DTC_STORED;
  s.dtc_raw[1] = 0x0420; s.dtc_kind[1] = vehicle::DTC_PENDING;
  s.dtc_raw[2] = 0xC100; s.dtc_kind[2] = vehicle::DTC_PERMANENT;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_dtc_count);
  TEST_ASSERT_EQUAL(3, d.dtc_total);
  TEST_ASSERT_EQUAL(3, d.dtc_n);
  TEST_ASSERT_EQUAL_HEX16(0x0301, d.dtcs[0].raw);
  TEST_ASSERT_EQUAL(vehicle::DTC_STORED, d.dtcs[0].kind);
  TEST_ASSERT_EQUAL_HEX16(0x0420, d.dtcs[1].raw);
  TEST_ASSERT_EQUAL(vehicle::DTC_PENDING, d.dtcs[1].kind);
  TEST_ASSERT_EQUAL_HEX16(0xC100, d.dtcs[2].raw);
  TEST_ASSERT_EQUAL(vehicle::DTC_PERMANENT, d.dtcs[2].kind);
  TEST_ASSERT_EQUAL(29 + 3 + 3 * 6, (int)lpp.buf.size());  // base + count + 3 codes
}

void test_dtcs_zero_codes_sends_count_only_and_unknown_sends_nothing() {
  VehicleSnapshot s = driving();
  s.has_dtcs = true;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_dtc_count);
  TEST_ASSERT_EQUAL(0, d.dtc_total);
  TEST_ASSERT_EQUAL(0, d.dtc_n);

  VehicleSnapshot u = driving();  // never read
  ModelLpp lpp2;
  encodeVehicle(u, lpp2);
  DecodedVehicle d2;
  TEST_ASSERT_TRUE(decodeVehicle(lpp2.buf.data(), lpp2.buf.size(), d2));
  TEST_ASSERT_FALSE(d2.has_dtc_count);
}

void test_dtcs_worst_case_fits_payload() {
  VehicleSnapshot s = driving();
  s.has_dtcs = true;
  s.dtc_total = 400;  // more than we carry; count saturates
  s.dtc_count = (uint8_t)VehicleSnapshot::MAX_DTCS;
  for (size_t i = 0; i < VehicleSnapshot::MAX_DTCS; i++) {
    s.dtc_raw[i] = 0xFFFF;
    s.dtc_kind[i] = vehicle::DTC_PERMANENT;
  }
  ModelLpp lpp(180);
  encodeVehicle(s, lpp);
  TEST_ASSERT_FALSE(lpp.overflow);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_EQUAL(255, d.dtc_total);
  TEST_ASSERT_EQUAL(12, d.dtc_n);
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, d.dtcs[11].raw);
  // MeshCore adds device battery (4) and GPS (11) on channel 1
  TEST_ASSERT_TRUE(lpp.buf.size() + 4 + 11 <= 180);
}

void test_slow_values_roundtrip() {
  VehicleSnapshot s = driving();
  s.has_coolant = true;
  s.coolant_c = 90.5f;
  s.has_load = true;
  s.load_pct = 37.6f;
  s.has_intake = true;
  s.intake_c = -12.3f;
  s.has_fuel = true;
  s.fuel_pct = 62.0f;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_coolant);
  TEST_ASSERT_FLOAT_WITHIN(0.11, 90.5, d.coolant_c);
  TEST_ASSERT_EQUAL(38, (int)d.load_pct);  // rounded
  TEST_ASSERT_FLOAT_WITHIN(0.11, -12.3, d.intake_c);
  TEST_ASSERT_EQUAL(62, (int)d.fuel_pct);
}

void test_slow_values_omitted_when_unknown() {
  VehicleSnapshot s = driving();
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_FALSE(d.has_coolant || d.has_load || d.has_intake || d.has_fuel);
}

void test_percentages_clamped() {
  VehicleSnapshot s = driving();
  s.has_load = s.has_fuel = true;
  s.load_pct = 140;
  s.fuel_pct = -3;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_EQUAL(100, (int)d.load_pct);
  TEST_ASSERT_EQUAL(0, (int)d.fuel_pct);
}

void test_worst_case_with_slow_values_fits() {
  VehicleSnapshot s = driving();
  s.has_coolant = s.has_load = s.has_intake = s.has_fuel = true;
  s.has_dtcs = true;
  s.dtc_total = 12;
  s.dtc_count = (uint8_t)VehicleSnapshot::MAX_DTCS;
  ModelLpp lpp(180);
  encodeVehicle(s, lpp);
  TEST_ASSERT_FALSE(lpp.overflow);
  TEST_ASSERT_TRUE(lpp.buf.size() + 4 + 11 <= 180);
}

void test_gps_extras_roundtrip_and_separate_from_obd() {
  VehicleSnapshot s = driving();
  roadnode::gps::GpsTrackState g;
  g.has_fix = g.has_motion = g.has_heading = true;
  g.speed_kmh = 63.4f;
  g.heading_deg = 271;
  g.trip_mm = 12345678;
  ModelLpp lpp;
  encodeVehicle(s, lpp);
  encodeGps(g, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_FLOAT_WITHIN(0.1, 12345.678, d.gps_trip_m);
  TEST_ASSERT_FLOAT_WITHIN(0.02, 63.4, d.gps_speed_kmh);
  TEST_ASSERT_EQUAL(271, d.gps_heading);
  // OBD values untouched by the GPS ones
  TEST_ASSERT_TRUE(d.has_speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01, s.speed_kmh, d.speed_kmh);
  TEST_ASSERT_FLOAT_WITHIN(0.01, (double)(s.trip_mm / 1000.0), d.trip_m);
}

void test_trip_ratio_roundtrip_and_omitted_when_invalid() {
  roadnode::gps::TripCompareResult r;
  r.valid = true;
  r.ratio_pm = 983;
  ModelLpp lpp;
  encodeTripCompare(r, lpp);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(lpp.buf.data(), lpp.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_trip_ratio);
  TEST_ASSERT_FLOAT_WITHIN(0.02, 98.3, d.trip_ratio_pct);
  ModelLpp none;
  encodeTripCompare(roadnode::gps::TripCompareResult(), none);
  TEST_ASSERT_EQUAL(0, none.buf.size());
}

void test_gps_extras_omitted_without_fix_or_motion() {
  VehicleSnapshot s = driving();
  roadnode::gps::GpsTrackState none;
  ModelLpp a;
  encodeVehicle(s, a);
  size_t base = a.buf.size();
  encodeGps(none, a);
  TEST_ASSERT_EQUAL(base, a.buf.size());  // nothing added

  roadnode::gps::GpsTrackState fix_only;
  fix_only.has_fix = true;
  ModelLpp b;
  encodeGps(fix_only, b);
  DecodedVehicle d;
  TEST_ASSERT_TRUE(decodeVehicle(b.buf.data(), b.buf.size(), d));
  TEST_ASSERT_TRUE(d.has_gps_trip);
  TEST_ASSERT_FALSE(d.has_gps_speed || d.has_gps_heading);  // unknown, never zero
}

void test_gps_extension_does_not_change_existing_encoding() {
  VehicleSnapshot s = driving();
  ModelLpp a, b;
  encodeVehicle(s, a);
  encodeVehicle(s, b);
  roadnode::gps::GpsTrackState g;
  g.has_fix = true;
  encodeGps(g, b);
  TEST_ASSERT_TRUE(b.buf.size() > a.buf.size());
  TEST_ASSERT_EQUAL_MEMORY(a.buf.data(), b.buf.data(), a.buf.size());  // old bytes identical: appended only
}

void test_worst_case_with_gps_fits() {
  VehicleSnapshot s = driving();
  s.has_coolant = s.has_load = s.has_intake = s.has_fuel = true;
  s.has_dtcs = true;
  s.dtc_total = 12;
  s.dtc_count = (uint8_t)VehicleSnapshot::MAX_DTCS;
  roadnode::gps::GpsTrackState g;
  g.has_fix = g.has_motion = g.has_heading = true;
  roadnode::gps::TripCompareResult tr;
  tr.valid = true;
  tr.ratio_pm = 1000;
  ModelLpp lpp(180);
  encodeVehicle(s, lpp);
  encodeGps(g, lpp);
  encodeTripCompare(tr, lpp);
  TEST_ASSERT_FALSE(lpp.overflow);
  TEST_ASSERT_TRUE(lpp.buf.size() + 4 + 11 <= 180);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_roundtrip_all_fields);
  RUN_TEST(test_unknown_values_omitted_not_zero);
  RUN_TEST(test_size_is_small_and_fits_payload);
  RUN_TEST(test_odometer_six_digits_miles);
  RUN_TEST(test_total_exact_in_float_up_to_cap);
  RUN_TEST(test_total_saturates_instead_of_wrapping);
  RUN_TEST(test_trip_saturates);
  RUN_TEST(test_state_bits_individually);
  RUN_TEST(test_decoder_skips_meshcore_channel_one);
  RUN_TEST(test_decoder_rejects_truncated_and_unknown);
  RUN_TEST(test_speed_signed_encoding_range);
  RUN_TEST(test_no_location_ever_encoded);
  RUN_TEST(test_dtcs_roundtrip_and_size);
  RUN_TEST(test_dtcs_zero_codes_sends_count_only_and_unknown_sends_nothing);
  RUN_TEST(test_dtcs_worst_case_fits_payload);
  RUN_TEST(test_slow_values_roundtrip);
  RUN_TEST(test_slow_values_omitted_when_unknown);
  RUN_TEST(test_percentages_clamped);
  RUN_TEST(test_worst_case_with_slow_values_fits);
  RUN_TEST(test_gps_extras_roundtrip_and_separate_from_obd);
  RUN_TEST(test_trip_ratio_roundtrip_and_omitted_when_invalid);
  RUN_TEST(test_gps_extras_omitted_without_fix_or_motion);
  RUN_TEST(test_gps_extension_does_not_change_existing_encoding);
  RUN_TEST(test_worst_case_with_gps_fits);
  return UNITY_END();
}
