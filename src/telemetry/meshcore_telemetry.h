#pragma once

// The only RoadNode module allowed to include MeshCore headers (plan sections 6, 26).
#include <helpers/SensorManager.h>

namespace roadnode {

// TODO: emit vehicle telemetry as Cayenne LPP from querySensors() (see #30, #31).
class RoadNodeSensorManager : public SensorManager {
public:
  bool begin() override { return true; }
};

}  // namespace roadnode
