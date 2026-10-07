#pragma once

#include <stddef.h>

#include "ecu_sim.h"

namespace roadnode {
namespace ecusim {

// Serial control for the simulator. Modifies `line` in place. Writes a one-line reply.
// Commands: status | cycle on|off | speed <kmh> | rpm <n> | coolant <c> | load <pct> | intake <c> |
//           fuel <pct> | volts <v> | dtc <03|07|0A> <hex4> | dtc clear | vin <17 chars>|none |
//           silent on|off | ignore <mode hex> | unignore <mode hex>
// Returns false if the command is not recognised.
bool handleEcuCommand(EcuSim& ecu, char* line, char* reply, size_t reply_len);

}  // namespace ecusim
}  // namespace roadnode
