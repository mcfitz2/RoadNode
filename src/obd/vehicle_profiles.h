#pragma once

#include <stddef.h>
#include <stdint.h>

// Per-vehicle settings (plan section 9). Generic OBD-II is tried first;
// anything vehicle-specific lives here, never in main.
namespace roadnode {
namespace obd {

enum class Protocol : uint8_t { Can11bit, Can29bit };

// Quirk flags, extended as real vehicles require.
constexpr uint32_t QUIRK_NONE = 0;
constexpr uint32_t QUIRK_NO_VIN_COUNT_BYTE = 1u << 0;  // Mode 09/02 reply lacks the item-count byte

struct VehicleProfile {
  const char* name;
  Protocol protocol;
  uint32_t bitrate_bps;
  uint32_t request_id;       // functional broadcast by default
  uint32_t response_id;      // 0 = accept any 0x7E8-0x7EF
  uint32_t poll_interval_ms;
  uint32_t timeout_ms;       // per request
  uint32_t quirks;
};

// Standard 11-bit, 500 kbit/s, functional request, any ECU answers.
const VehicleProfile& genericProfile();

// Hook for VIN-based selection (later). Falls back to the generic profile.
const VehicleProfile& selectProfile(const char* vin);

}  // namespace obd
}  // namespace roadnode
