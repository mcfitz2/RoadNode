#pragma once

#include <stddef.h>
#include <stdint.h>

#include "gps/gps_track.h"
#include "vehicle/vehicle_state.h"

// Vehicle telemetry as Cayenne LPP (spec: docs/telemetry.md, issue #31).
// Header-only and templated on the LPP writer, so it has no MeshCore or
// CayenneLPP dependency: firmware instantiates it with CayenneLPP, host tests
// with a model of the wire format. Location is deliberately never encoded (#51).
namespace roadnode {
namespace telemetry {

// Channel 1 is MeshCore's own (device battery, GPS). Vehicle data starts at 2.
constexpr uint8_t CH_TOTAL_DISTANCE = 2;   // LPP generic sensor, units of 0.1 km
constexpr uint8_t CH_TRIP_DISTANCE = 3;    // LPP distance, metres
constexpr uint8_t CH_SPEED = 4;            // LPP analog input, km/h
constexpr uint8_t CH_RPM = 5;              // LPP generic sensor, rpm
constexpr uint8_t CH_STATE = 6;            // LPP digital input, bit flags
constexpr uint8_t CH_VEHICLE_BATTERY = 7;  // LPP voltage, volts
constexpr uint8_t CH_DTC_COUNT = 8;        // LPP digital input, total DTCs reported by the ECU (cap 255)
constexpr uint8_t CH_COOLANT = 10;         // LPP temperature, 0.1 C
constexpr uint8_t CH_LOAD = 11;            // LPP percentage, engine load
constexpr uint8_t CH_INTAKE = 12;          // LPP temperature, intake air
constexpr uint8_t CH_FUEL = 13;            // LPP percentage, fuel level
constexpr uint8_t CH_GPS_TRIP = 14;        // LPP distance, metres travelled per GPS (separate from the OBD trip, channel 3)
constexpr uint8_t CH_GPS_SPEED = 15;       // LPP analog input, km/h from GPS position change
constexpr uint8_t CH_GPS_HEADING = 16;     // LPP direction, degrees 0-359
constexpr uint8_t CH_DTC = 9;              // LPP generic sensor, repeated: kind << 16 | raw code

constexpr uint8_t STATE_VEHICLE_ACTIVE = 1 << 0;
constexpr uint8_t STATE_ENGINE_RUNNING = 1 << 1;
constexpr uint8_t STATE_TRIP_ACTIVE = 1 << 2;
constexpr uint8_t STATE_OBD_CONNECTED = 1 << 3;

// LPP encodes value * multiplier into a uint32 from a float. A float holds
// integers exactly up to 2^24, so the total is capped there (1.6 million km).
constexpr uint32_t TOTAL_UNIT_MM = 100000;
constexpr uint32_t TOTAL_MAX_UNITS = 16777216;
// LPP distance is 0.001 m in 32 bits: 4,294,967 m. Stay under it.
constexpr uint32_t TRIP_MAX_M = 4294000;

// Fields without a fresh value are omitted, never sent as zero. Total, trip
// and state are always present.
inline uint32_t pct(float v) { return v <= 0 ? 0 : v >= 100 ? 100 : (uint32_t)(v + 0.5f); }

template <class Lpp>
void encodeVehicle(const vehicle::VehicleSnapshot& s, Lpp& lpp) {
  uint64_t units = s.total_mm / TOTAL_UNIT_MM;
  if (units > TOTAL_MAX_UNITS) units = TOTAL_MAX_UNITS;
  lpp.addGenericSensor(CH_TOTAL_DISTANCE, (float)units);

  float trip_m = (float)(s.trip_mm / 1000.0);
  if (trip_m > (float)TRIP_MAX_M) trip_m = (float)TRIP_MAX_M;
  lpp.addDistance(CH_TRIP_DISTANCE, trip_m);

  if (s.has_speed) lpp.addAnalogInput(CH_SPEED, s.speed_kmh);
  if (s.has_rpm) lpp.addGenericSensor(CH_RPM, s.rpm);

  uint8_t state = 0;
  if (s.vehicle_active) state |= STATE_VEHICLE_ACTIVE;
  if (s.engine_running) state |= STATE_ENGINE_RUNNING;
  if (s.trip_active) state |= STATE_TRIP_ACTIVE;
  if (s.obd_connected) state |= STATE_OBD_CONNECTED;
  lpp.addDigitalInput(CH_STATE, state);

  if (s.has_battery) lpp.addVoltage(CH_VEHICLE_BATTERY, s.battery_v);

  // Slow engine values, each only while fresh.
  if (s.has_coolant) lpp.addTemperature(CH_COOLANT, s.coolant_c);
  if (s.has_load) lpp.addPercentage(CH_LOAD, pct(s.load_pct));
  if (s.has_intake) lpp.addTemperature(CH_INTAKE, s.intake_c);
  if (s.has_fuel) lpp.addPercentage(CH_FUEL, pct(s.fuel_pct));

  // DTCs only after a successful read (0 codes is a real answer, unknown is omitted).
  if (s.has_dtcs) {
    lpp.addDigitalInput(CH_DTC_COUNT, s.dtc_total > 255 ? 255 : (uint8_t)s.dtc_total);
    for (uint8_t i = 0; i < s.dtc_count && i < vehicle::VehicleSnapshot::MAX_DTCS; i++)
      lpp.addGenericSensor(CH_DTC, (float)(((uint32_t)s.dtc_kind[i] << 16) | s.dtc_raw[i]));
  }
}

// GPS-derived extras (#42). Latitude, longitude and altitude stay on MeshCore's
// channel 1. Each field is omitted unless known; none replaces the OBD values.
template <class Lpp>
void encodeGps(const gps::GpsTrackState& g, Lpp& lpp) {
  if (!g.has_fix) return;
  float trip_m = (float)(g.trip_mm / 1000.0);
  if (trip_m > (float)TRIP_MAX_M) trip_m = (float)TRIP_MAX_M;
  lpp.addDistance(CH_GPS_TRIP, trip_m);
  if (g.has_motion) lpp.addAnalogInput(CH_GPS_SPEED, g.speed_kmh > 300 ? 300 : g.speed_kmh);
  if (g.has_heading) lpp.addDirection(CH_GPS_HEADING, g.heading_deg);
}

struct DecodedDtc {
  uint8_t kind = 0;   // vehicle::DtcKind
  uint16_t raw = 0;   // two wire bytes
};

struct DecodedVehicle {
  bool has_total = false;
  double total_km = 0;
  bool has_trip = false;
  double trip_m = 0;
  bool has_speed = false;
  double speed_kmh = 0;
  bool has_rpm = false;
  double rpm = 0;
  bool has_state = false;
  uint8_t state = 0;
  bool has_battery = false;
  double battery_v = 0;
  bool has_coolant = false;
  double coolant_c = 0;
  bool has_load = false;
  double load_pct = 0;
  bool has_intake = false;
  double intake_c = 0;
  bool has_fuel = false;
  double fuel_pct = 0;
  bool has_gps_trip = false;
  double gps_trip_m = 0;
  bool has_gps_speed = false;
  double gps_speed_kmh = 0;
  bool has_gps_heading = false;
  uint16_t gps_heading = 0;
  bool has_dtc_count = false;
  uint8_t dtc_total = 0;
  uint8_t dtc_n = 0;
  DecodedDtc dtcs[vehicle::VehicleSnapshot::MAX_DTCS];
};

// Walks an LPP buffer and picks out the vehicle channels, skipping other
// channels (MeshCore's channel 1). Returns false on a truncated buffer or an
// LPP type this decoder does not know the size of.
inline bool decodeVehicle(const uint8_t* buf, size_t len, DecodedVehicle& out) {
  size_t i = 0;
  while (i + 2 <= len) {
    uint8_t ch = buf[i], type = buf[i + 1];
    size_t size;
    switch (type) {
      case 0: case 1: case 102: case 120: case 142: size = 1; break;
      case 2: case 3: case 101: case 103: case 115: case 116: case 117: case 121: case 125: case 128: case 132:
        size = 2;
        break;
      case 100: case 118: case 130: case 131: case 133: size = 4; break;
      case 113: case 134: size = 6; break;
      case 136: size = 9; break;
      default: return false;
    }
    if (i + 2 + size > len) return false;
    const uint8_t* p = buf + i + 2;
    uint32_t u = 0;
    for (size_t k = 0; k < size && k < 4; k++) u = (u << 8) | p[k];

    if (ch == CH_TOTAL_DISTANCE && type == 100) {
      out.has_total = true;
      out.total_km = u / 10.0;
    } else if (ch == CH_TRIP_DISTANCE && type == 130) {
      out.has_trip = true;
      out.trip_m = u / 1000.0;
    } else if (ch == CH_SPEED && type == 2) {
      out.has_speed = true;
      out.speed_kmh = (int16_t)(uint16_t)u / 100.0;
    } else if (ch == CH_RPM && type == 100) {
      out.has_rpm = true;
      out.rpm = u;
    } else if (ch == CH_STATE && type == 0) {
      out.has_state = true;
      out.state = (uint8_t)u;
    } else if (ch == CH_VEHICLE_BATTERY && type == 116) {
      out.has_battery = true;
      out.battery_v = u / 100.0;
    } else if (ch == CH_COOLANT && type == 103) {
      out.has_coolant = true;
      out.coolant_c = (int16_t)(uint16_t)u / 10.0;
    } else if (ch == CH_LOAD && type == 120) {
      out.has_load = true;
      out.load_pct = u;
    } else if (ch == CH_INTAKE && type == 103) {
      out.has_intake = true;
      out.intake_c = (int16_t)(uint16_t)u / 10.0;
    } else if (ch == CH_FUEL && type == 120) {
      out.has_fuel = true;
      out.fuel_pct = u;
    } else if (ch == CH_GPS_TRIP && type == 130) {
      out.has_gps_trip = true;
      out.gps_trip_m = u / 1000.0;
    } else if (ch == CH_GPS_SPEED && type == 2) {
      out.has_gps_speed = true;
      out.gps_speed_kmh = (int16_t)(uint16_t)u / 100.0;
    } else if (ch == CH_GPS_HEADING && type == 132) {
      out.has_gps_heading = true;
      out.gps_heading = (uint16_t)u;
    } else if (ch == CH_DTC_COUNT && type == 0) {
      out.has_dtc_count = true;
      out.dtc_total = (uint8_t)u;
    } else if (ch == CH_DTC && type == 100 && out.dtc_n < vehicle::VehicleSnapshot::MAX_DTCS) {
      out.dtcs[out.dtc_n].kind = (uint8_t)(u >> 16);
      out.dtcs[out.dtc_n].raw = (uint16_t)u;
      out.dtc_n++;
    }
    i += 2 + size;
  }
  return i == len;
}

}  // namespace telemetry
}  // namespace roadnode
