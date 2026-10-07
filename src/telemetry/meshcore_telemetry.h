#pragma once

// The only RoadNode module allowed to include MeshCore headers (plan sections 6, 26).
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>

#include "gps/gps_track.h"
#include "gps/trip_compare.h"
#include "storage/nvs_kv_store.h"

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
  // True once a GNSS fix has been seen this boot, i.e. MeshCore has set the RTC from GPS time.
  bool clockFromGps() const { return _clock_from_gps; }

private:
  gps::GpsTrack _gps_track;
  storage::NvsKvStore _kv;
  gps::TripCompareResult _last_trip_compare;  // OBD vs GPS distance of the last finished trip (#43)
  uint32_t _gps_last_ms = 0;
  bool _gps_trip_was_active = false;
  bool _clock_from_gps = false;
#endif
};

}  // namespace roadnode
