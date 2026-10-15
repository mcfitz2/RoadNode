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
// On top of it, note the first fix so trip-end stamps know the clock is real.
void RoadNodeSensorManager::loop() {
  EnvironmentSensorManager::loop();
  if (!_clock_from_gps && gps_active && _location->isValid()) _clock_from_gps = true;
}
#endif

bool RoadNodeSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  // Location (stock GPS channel 1) goes only to authenticated clients: telemetry
  // requests are encrypted and answered only for ACL entries. Adverts carry the
  // fixed ADVERT_LAT/LON prefs, never live GPS (#51).
  bool ok = EnvironmentSensorManager::querySensors(requester_permissions, telemetry);
  if (requester_permissions & TELEM_PERM_BASE) {
    telemetry::encodeVehicle(vehicle::VehicleRuntime::telemetry().snapshot(), telemetry);
    ok = true;
  }
  return ok;
}

}  // namespace roadnode
