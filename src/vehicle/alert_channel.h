#pragma once

#include <stddef.h>
#include <stdint.h>

#include "storage/kv_store.h"

namespace roadnode {
namespace vehicle {

// Optional private group channel that alert texts are also posted to. Stock MeshCore apps
// have no chat view for sensor contacts, so the direct alerts are ACKed but never shown;
// a channel message is shown by every client that holds the channel secret.
struct AlertChannel {
  bool enabled = false;
  uint8_t secret[32] = {};
  uint8_t len = 0;  // 16 or 32 bytes, as MeshCore channels accept
};

// Loads the persisted channel secret. No stored value (or a corrupt one) leaves it disabled.
void loadAlertChannel(storage::KvStore& kv, AlertChannel& ch);

// Admin commands: "alert channel" (show), "alert channel <hex secret>" (32 or 64 hex chars,
// as shown by the app's channel sharing), "alert channel off". Persisted. The secret is
// never echoed back. Returns true if the command was recognised; reply gets a short text.
bool handleAlertChannelCommand(storage::KvStore& kv, AlertChannel& ch, const char* command, char* reply);

}  // namespace vehicle
}  // namespace roadnode
