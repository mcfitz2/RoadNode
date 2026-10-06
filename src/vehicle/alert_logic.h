#pragma once

#include "vehicle_state.h"

// Conditions for mesh alerts (issue #30/#51). MeshCore-free: the sensor node
// feeds these to SensorMesh::alertIf, which fires once per false->true edge.
namespace roadnode {
namespace vehicle {

struct AlertConditions {
  bool started = false;  // engine running
  bool parked = false;   // was driven since last start, now engine off and CAN bus quiet
};

class AlertLogic {
public:
  AlertConditions update(const VehicleSnapshot& s) {
    if (s.engine_running) _driven = true;
    AlertConditions c;
    c.started = s.engine_running;
    // Not parked at boot: needs an engine-running observation first. The poller
    // debounces vehicle_active with its CAN-silence timeout.
    c.parked = _driven && !s.engine_running && !s.vehicle_active;
    return c;
  }

private:
  bool _driven = false;
};

}  // namespace vehicle
}  // namespace roadnode
