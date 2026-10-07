#include "meshcore_telemetry.h"

#include "telemetry/vehicle_lpp.h"
#include "vehicle/vehicle_runtime.h"

#ifndef VEHICLE_ID
#define VEHICLE_ID "UNSET"
#endif

namespace roadnode {

bool RoadNodeSensorManager::begin() {
  // Vehicle stack first: it must run even if the stock sensor setup fails.
#ifdef OBD_TRANSMIT_ENABLE
  vehicle::VehicleRuntime::begin(VEHICLE_ID, true);
#else
  vehicle::VehicleRuntime::begin(VEHICLE_ID, false);
#endif
  return EnvironmentSensorManager::begin();
}

#if ENV_INCLUDE_GPS
// Stock loop parses NMEA and (through MeshCore's provider) syncs the RTC from GPS time.
// On top of it, sample the provider about once a second for GPS-derived motion (#42).
void RoadNodeSensorManager::loop() {
  EnvironmentSensorManager::loop();
  uint32_t now = millis();
  if (now - _gps_last_ms < 1000) return;
  _gps_last_ms = now;
  // The GPS trip follows the OBD trip's start so both describe the same drive.
  vehicle::VehicleSnapshot snap = vehicle::VehicleRuntime::telemetry().snapshot();
  bool trip = snap.trip_active;
  if (trip && !_gps_trip_was_active) _gps_track.resetTrip();
  if (!trip && _gps_trip_was_active) {  // trip just ended: freeze the GPS side before it drifts on
    gps::GpsTrackState g = _gps_track.state(now);
    _last_trip_compare = gps::compareTrip(snap.trip_mm, g.trip_mm, g.trip_fix_ms, g.trip_total_ms);
    const gps::TripCompareResult& r = _last_trip_compare;
    Serial.printf("# trip end: obd %lu m gps %lu m cover %u.%u%%", (unsigned long)r.obd_m, (unsigned long)r.gps_m,
                  r.coverage_pm / 10, r.coverage_pm % 10);
    if (r.valid) Serial.printf(" ratio %u.%u%% %s\n", r.ratio_pm / 10, r.ratio_pm % 10, r.suspect ? "SUSPECT" : "ok");
    else Serial.println(" n/a (short trip or low GPS coverage)");
  }
  _gps_trip_was_active = trip;

  bool valid = gps_active && _location->isValid();  // GNSS off still counts as a no-fix sample for coverage
  if (valid) _clock_from_gps = true;
  _gps_track.update(now, valid, valid ? (int32_t)_location->getLatitude() : 0, valid ? (int32_t)_location->getLongitude() : 0);
}
#endif

bool RoadNodeSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  // Location (stock GPS channel 1) goes only to authenticated clients: telemetry
  // requests are encrypted and answered only for ACL entries. Adverts carry the
  // fixed ADVERT_LAT/LON prefs, never live GPS (#51).
  bool ok = EnvironmentSensorManager::querySensors(requester_permissions, telemetry);
  if (requester_permissions & TELEM_PERM_BASE) {
    telemetry::encodeVehicle(vehicle::VehicleRuntime::telemetry().snapshot(), telemetry);
#if ENV_INCLUDE_GPS
    // Same rule as channel 1: location-derived data only for requesters with location permission.
    if ((requester_permissions & TELEM_PERM_LOCATION) && gps_active) {
      telemetry::encodeGps(_gps_track.state(millis()), telemetry);
      telemetry::encodeTripCompare(_last_trip_compare, telemetry);
    }
#endif
    ok = true;
  }
  return ok;
}

}  // namespace roadnode
