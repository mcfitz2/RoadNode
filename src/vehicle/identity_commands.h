#pragma once

#include "vehicle_identity.h"

namespace roadnode {
namespace vehicle {

// Admin commands: "vid" (show), "vid set <ID>", "vin" (status only), "vin clear".
// The VIN itself is never put in a reply: replies travel over the mesh.
// Returns true if the command was recognised; reply gets a short text.
bool handleIdentityCommand(VehicleIdentity& identity, const char* command, char* reply);

}  // namespace vehicle
}  // namespace roadnode
