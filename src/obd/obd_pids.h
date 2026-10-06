#pragma once

#include <stdint.h>

// Mode 01 PID decoders and supported-PID bitmask handling (plan section 8).
// Pure functions: no CAN, no MeshCore.
namespace roadnode {
namespace obd {

constexpr uint8_t PID_SUPPORTED_00 = 0x00;
constexpr uint8_t PID_ENGINE_LOAD = 0x04;
constexpr uint8_t PID_COOLANT_TEMP = 0x05;
constexpr uint8_t PID_RPM = 0x0C;
constexpr uint8_t PID_SPEED = 0x0D;
constexpr uint8_t PID_INTAKE_TEMP = 0x0F;
constexpr uint8_t PID_FUEL_LEVEL = 0x2F;
constexpr uint8_t PID_MODULE_VOLTAGE = 0x42;

// Decoders take the data bytes after "41 PID". Return false if too short.
bool decodeSpeed(const uint8_t* d, uint8_t len, float& kmh);
bool decodeRpm(const uint8_t* d, uint8_t len, float& rpm);
bool decodeCoolantTemp(const uint8_t* d, uint8_t len, float& deg_c);
bool decodeEngineLoad(const uint8_t* d, uint8_t len, float& percent);
bool decodeIntakeTemp(const uint8_t* d, uint8_t len, float& deg_c);
bool decodeFuelLevel(const uint8_t* d, uint8_t len, float& percent);
bool decodeModuleVoltage(const uint8_t* d, uint8_t len, float& volts);

// Generic decode by PID. Returns false for unknown PID or short data.
bool decodePid(uint8_t pid, const uint8_t* d, uint8_t len, float& value);

// Which Mode 01 PIDs (0x01..0xE0) the vehicle reports as supported.
// Nothing is assumed supported until a range has been loaded.
class CapabilityTable {
public:
  // Load the 4-byte bitmask answered by PID `base` (0x00, 0x20, ... 0xC0).
  // Bit 31 (MSB of the first byte) is base+1 ... bit 0 is base+0x20.
  bool load(uint8_t base, const uint8_t* mask, uint8_t len);

  bool rangeKnown(uint8_t pid) const;
  bool supported(uint8_t pid) const;

  // True if the range answered by `base` says the next range exists
  // (bit for PID base+0x20 is set), i.e. discovery should continue.
  bool hasNextRange(uint8_t base) const;

  // Discovery stopped at `base` because no further range exists: every later
  // range is known to be unsupported rather than unknown.
  void markRemainingUnsupported(uint8_t base);

  void clear();

private:
  uint32_t _mask[8] = {0};
  uint8_t _known = 0;  // bit per loaded range
};

}  // namespace obd
}  // namespace roadnode
