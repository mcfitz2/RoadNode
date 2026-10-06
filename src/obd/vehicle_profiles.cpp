#include "vehicle_profiles.h"

namespace roadnode {
namespace obd {

static const VehicleProfile GENERIC = {"Generic OBD-II", Protocol::Can11bit, 500000, 0x7DF, 0, 1000, 200, QUIRK_NONE};

const VehicleProfile& genericProfile() { return GENERIC; }

const VehicleProfile& selectProfile(const char* vin) {
  (void)vin;  // no vehicle-specific profiles yet
  return GENERIC;
}

}  // namespace obd
}  // namespace roadnode
