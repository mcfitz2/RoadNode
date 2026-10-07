#include "ecu_commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace roadnode {
namespace ecusim {

static bool isOn(const char* s) { return s && strcmp(s, "on") == 0; }

bool handleEcuCommand(EcuSim& ecu, char* line, char* reply, size_t n) {
  EcuScenario& sc = ecu.scenario();
  char* save = nullptr;
  char* cmd = strtok_r(line, " \t", &save);
  char* a = strtok_r(nullptr, " \t", &save);
  char* b = strtok_r(nullptr, " \t", &save);
  if (!cmd) return false;
  reply[0] = 0;

  if (strcmp(cmd, "status") == 0) {
    snprintf(reply, n, "cycle=%s speed=%.0f rpm=%.0f dtc=%u/%u/%u vin=%s silent=%s ignore=0x%lX req=%lu forbidden=%lu dist=%.0fm",
             sc.drive_cycle ? "on" : "off", sc.speed_kmh, sc.rpm, (unsigned)sc.stored.count,
             (unsigned)sc.pending.count, (unsigned)sc.permanent.count, sc.vin_supported ? sc.vin : "none",
             sc.silent ? "on" : "off", (unsigned long)sc.ignore_modes, (unsigned long)ecu.requests(),
             (unsigned long)ecu.forbiddenRequests(), ecu.distanceM());
    return true;
  }
  if (strcmp(cmd, "cycle") == 0 && a) {
    sc.drive_cycle = isOn(a);
    if (sc.drive_cycle) ecu.resetDistance();
    snprintf(reply, n, "OK cycle %s", sc.drive_cycle ? "on" : "off");
    return true;
  }
  if (strcmp(cmd, "silent") == 0 && a) {
    sc.silent = isOn(a);
    snprintf(reply, n, "OK silent %s", sc.silent ? "on" : "off");
    return true;
  }
  struct Num {
    const char* name;
    float* field;
  } nums[] = {{"speed", &sc.speed_kmh}, {"rpm", &sc.rpm},      {"coolant", &sc.coolant_c}, {"load", &sc.load_pct},
              {"intake", &sc.intake_c}, {"fuel", &sc.fuel_pct}, {"volts", &sc.module_volts}};
  for (const Num& x : nums) {
    if (strcmp(cmd, x.name) == 0 && a) {
      *x.field = (float)atof(a);
      snprintf(reply, n, "OK %s %.2f", x.name, *x.field);
      return true;
    }
  }
  if ((strcmp(cmd, "ignore") == 0 || strcmp(cmd, "unignore") == 0) && a) {
    unsigned long m = strtoul(a, nullptr, 16);
    if (m >= 32) return false;
    if (cmd[0] == 'i') sc.ignore_modes |= 1u << m;
    else sc.ignore_modes &= ~(1u << m);
    snprintf(reply, n, "OK ignore=0x%lX", (unsigned long)sc.ignore_modes);
    return true;
  }
  if (strcmp(cmd, "vin") == 0 && a) {
    if (strcmp(a, "none") == 0) {
      sc.vin_supported = false;
    } else if (strlen(a) == 17) {
      memcpy(sc.vin, a, 18);
      sc.vin_supported = true;
    } else {
      return false;
    }
    snprintf(reply, n, "OK vin %s", sc.vin_supported ? sc.vin : "none");
    return true;
  }
  if (strcmp(cmd, "dtc") == 0 && a) {
    if (strcmp(a, "clear") == 0) {  // simulator state only; the node cannot send mode 04
      sc.stored.count = sc.pending.count = sc.permanent.count = 0;
      snprintf(reply, n, "OK dtcs cleared");
      return true;
    }
    unsigned long mode = strtoul(a, nullptr, 16);
    DtcSet* s = mode == 0x03 ? &sc.stored : mode == 0x07 ? &sc.pending : mode == 0x0A ? &sc.permanent : nullptr;
    if (!s || !b || s->count >= MAX_DTCS) return false;
    s->raw[s->count++] = (uint16_t)strtoul(b, nullptr, 16);
    snprintf(reply, n, "OK mode %02lX now %u codes", mode, (unsigned)s->count);
    return true;
  }
  return false;
}

}  // namespace ecusim
}  // namespace roadnode
