#include "alert_channel.h"

#include <string.h>

namespace roadnode {
namespace vehicle {

static const char* KEY_CHANNEL = "alert_ch";

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Accepts exactly 32 or 64 hex characters (16 or 32 bytes), nothing else.
static bool parseSecret(const char* s, uint8_t* out, uint8_t& len) {
  size_t n = strlen(s);
  if (n != 32 && n != 64) return false;
  for (size_t i = 0; i < n; i += 2) {
    int hi = hexDigit(s[i]), lo = hexDigit(s[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i / 2] = (uint8_t)(hi << 4 | lo);
  }
  len = (uint8_t)(n / 2);
  return true;
}

void loadAlertChannel(storage::KvStore& kv, AlertChannel& ch) {
  char buf[65];
  AlertChannel loaded;
  if (kv.getString(KEY_CHANNEL, buf, sizeof(buf)) && parseSecret(buf, loaded.secret, loaded.len)) {
    loaded.enabled = true;
    ch = loaded;
  }
}

bool handleAlertChannelCommand(storage::KvStore& kv, AlertChannel& ch, const char* command, char* reply) {
  if (strcmp(command, "alert channel") == 0) {
    if (ch.enabled) strcpy(reply, ch.len == 32 ? "channel on (256-bit key)" : "channel on (128-bit key)");
    else strcpy(reply, "channel off");
    return true;
  }
  if (strcmp(command, "alert channel off") == 0) {
    if (!kv.remove(KEY_CHANNEL)) {
      strcpy(reply, "Err: save failed");
      return true;
    }
    ch = AlertChannel();
    strcpy(reply, "OK channel off");
    return true;
  }
  if (strncmp(command, "alert channel ", 14) == 0) {
    AlertChannel next;
    if (!parseSecret(command + 14, next.secret, next.len)) {
      strcpy(reply, "Err: secret must be 32 or 64 hex chars");
      return true;
    }
    if (!kv.putString(KEY_CHANNEL, command + 14)) {
      strcpy(reply, "Err: save failed");
      return true;
    }
    next.enabled = true;
    ch = next;
    strcpy(reply, "OK channel on");
    return true;
  }
  return false;
}

}  // namespace vehicle
}  // namespace roadnode
