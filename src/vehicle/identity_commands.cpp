#include "identity_commands.h"

#include <stdio.h>
#include <string.h>

namespace roadnode {
namespace vehicle {

bool handleIdentityCommand(VehicleIdentity& identity, const char* command, char* reply) {
  if (strcmp(command, "vid") == 0) {
    sprintf(reply, "id %s", identity.id());
    return true;
  }
  if (strncmp(command, "vid set ", 8) == 0) {
    if (identity.setId(command + 8)) {
      sprintf(reply, "OK id %s", identity.id());
    } else {
      strcpy(reply, "Err: id must be 1-15 of A-Z 0-9 - _");
    }
    return true;
  }
  if (strcmp(command, "vin") == 0) {
    if (!identity.hasVin()) {
      strcpy(reply, "vin not read");
    } else {
      sprintf(reply, "vin stored, check digit %s%s", identity.vinCheckDigitOk() ? "ok" : "bad",
              identity.vinMismatch() ? ", ECU reports a different VIN" : "");
    }
    return true;
  }
  if (strcmp(command, "vin clear") == 0) {
    strcpy(reply, identity.clearVin() ? "OK vin cleared" : "Err: clear failed");
    return true;
  }
  return false;
}

}  // namespace vehicle
}  // namespace roadnode
