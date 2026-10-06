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

bool RoadNodeSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  // Vehicle location is never reported over the mesh (see #51): strip the
  // location permission before the stock GPS path can add it.
  bool ok = EnvironmentSensorManager::querySensors(requester_permissions & ~TELEM_PERM_LOCATION, telemetry);
  if (requester_permissions & TELEM_PERM_BASE) {
    telemetry::encodeVehicle(vehicle::VehicleRuntime::telemetry().snapshot(), telemetry);
    ok = true;
  }
  return ok;
}

}  // namespace roadnode
