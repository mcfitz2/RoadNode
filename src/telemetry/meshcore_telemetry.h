#pragma once

// The only RoadNode module allowed to include MeshCore headers (plan sections 6, 26).
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>

#include "gps/gps_track.h"

namespace roadnode {

// Stock EnvironmentSensorManager plus vehicle telemetry from VehicleRuntime.
// MeshCore only reads the latest snapshot; the vehicle stack runs without it.
class RoadNodeSensorManager : public EnvironmentSensorManager {
public:
#if ENV_INCLUDE_GPS
  explicit RoadNodeSensorManager(LocationProvider& location) : EnvironmentSensorManager(location) {}
#else
  RoadNodeSensorManager() {}
#endif

  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
#if ENV_INCLUDE_GPS
  void loop() override;

private:
  gps::GpsTrack _gps_track;
  uint32_t _gps_last_ms = 0;
  bool _gps_trip_was_active = false;
#endif
};

}  // namespace roadnode
