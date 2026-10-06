#pragma once

#include "alert_logic.h"
#include "storage/kv_store.h"

namespace roadnode {
namespace vehicle {

constexpr uint32_t ALERT_MINUTES_MAX = 1440;  // 24 h

// Loads the persisted "still moving" interval (minutes) into logic. No stored
// value keeps the logic's current (build-time default) interval.
void loadAlertSettings(storage::KvStore& kv, AlertLogic& logic);

// Admin commands: "alert" (show), "alert periodic <minutes>" (0 = off, max 1440), persisted.
// Returns true if the command was recognised; reply gets a short text.
bool handleAlertCommand(storage::KvStore& kv, AlertLogic& logic, const char* command, char* reply);

}  // namespace vehicle
}  // namespace roadnode
