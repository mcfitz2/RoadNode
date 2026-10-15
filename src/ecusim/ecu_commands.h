#pragma once

#include <stddef.h>

#include "ecu_sim.h"

namespace roadnode {
namespace ecusim {

// Serial control for the simulator. Modifies `line` in place. Writes a one-line reply.
// Commands: status | cycle on|off | speed <kmh> | rpm <n> | volts <v> | vin <17 chars>|none |
//           silent on|off | delay <ms> | pending <0-8> | corrupt none|short|skipcf | ignore <mode hex> | unignore <mode hex>
// Returns false if the command is not recognised.
bool handleEcuCommand(EcuSim& ecu, char* line, char* reply, size_t reply_len);

}  // namespace ecusim
}  // namespace roadnode
