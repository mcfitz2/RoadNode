#include "alert_settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace roadnode {
namespace vehicle {

static const char* KEY_PERIODIC_MIN = "alert_min";

static bool parseMinutes(const char* s, uint32_t& out) {
  if (!*s) return false;
  uint32_t v = 0;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
    v = v * 10 + (uint32_t)(*s - '0');
    if (v > ALERT_MINUTES_MAX) return false;
  }
  out = v;
  return true;
}

void loadAlertSettings(storage::KvStore& kv, AlertLogic& logic) {
  char buf[8];
  uint32_t m;
  if (kv.getString(KEY_PERIODIC_MIN, buf, sizeof(buf)) && parseMinutes(buf, m)) logic.setPeriodicIntervalMs(m * 60u * 1000u);
}

bool handleAlertCommand(storage::KvStore& kv, AlertLogic& logic, const char* command, char* reply) {
  if (strcmp(command, "alert") == 0) {
    uint32_t m = logic.periodicIntervalMs() / 60000u;
    if (m) sprintf(reply, "periodic %lu min", (unsigned long)m);
    else strcpy(reply, "periodic off");
    return true;
  }
  if (strncmp(command, "alert periodic ", 15) == 0) {
    uint32_t m;
    if (!parseMinutes(command + 15, m)) {
      strcpy(reply, "Err: minutes 0-1440");
      return true;
    }
    char buf[8];
    sprintf(buf, "%lu", (unsigned long)m);
    if (!kv.putString(KEY_PERIODIC_MIN, buf)) {
      strcpy(reply, "Err: save failed");
      return true;
    }
    logic.setPeriodicIntervalMs(m * 60u * 1000u);
    if (m) sprintf(reply, "OK periodic %lu min", (unsigned long)m);
    else strcpy(reply, "OK periodic off");
    return true;
  }
  return false;
}

}  // namespace vehicle
}  // namespace roadnode
