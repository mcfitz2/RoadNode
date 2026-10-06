#include "obd_pids.h"

namespace roadnode {
namespace obd {

bool decodeSpeed(const uint8_t* d, uint8_t len, float& kmh) {
  if (len < 1) return false;
  kmh = d[0];
  return true;
}

bool decodeRpm(const uint8_t* d, uint8_t len, float& rpm) {
  if (len < 2) return false;
  rpm = (d[0] * 256 + d[1]) / 4.0f;
  return true;
}

bool decodeCoolantTemp(const uint8_t* d, uint8_t len, float& deg_c) {
  if (len < 1) return false;
  deg_c = (float)d[0] - 40.0f;
  return true;
}

bool decodeEngineLoad(const uint8_t* d, uint8_t len, float& percent) {
  if (len < 1) return false;
  percent = d[0] * 100.0f / 255.0f;
  return true;
}

bool decodeIntakeTemp(const uint8_t* d, uint8_t len, float& deg_c) { return decodeCoolantTemp(d, len, deg_c); }

bool decodeFuelLevel(const uint8_t* d, uint8_t len, float& percent) { return decodeEngineLoad(d, len, percent); }

bool decodeModuleVoltage(const uint8_t* d, uint8_t len, float& volts) {
  if (len < 2) return false;
  volts = (d[0] * 256 + d[1]) / 1000.0f;
  return true;
}

bool decodePid(uint8_t pid, const uint8_t* d, uint8_t len, float& value) {
  switch (pid) {
    case PID_SPEED: return decodeSpeed(d, len, value);
    case PID_RPM: return decodeRpm(d, len, value);
    case PID_COOLANT_TEMP: return decodeCoolantTemp(d, len, value);
    case PID_ENGINE_LOAD: return decodeEngineLoad(d, len, value);
    case PID_INTAKE_TEMP: return decodeIntakeTemp(d, len, value);
    case PID_FUEL_LEVEL: return decodeFuelLevel(d, len, value);
    case PID_MODULE_VOLTAGE: return decodeModuleVoltage(d, len, value);
    default: return false;
  }
}

bool CapabilityTable::load(uint8_t base, const uint8_t* mask, uint8_t len) {
  if (len < 4 || base > 0xE0 || (base & 0x1F) != 0) return false;
  uint8_t idx = base >> 5;
  _mask[idx] = ((uint32_t)mask[0] << 24) | ((uint32_t)mask[1] << 16) | ((uint32_t)mask[2] << 8) | mask[3];
  _known |= (uint8_t)(1u << idx);
  return true;
}

bool CapabilityTable::rangeKnown(uint8_t pid) const {
  if (pid == 0) return true;
  return _known & (1u << ((pid - 1) >> 5));
}

void CapabilityTable::markRemainingUnsupported(uint8_t base) {
  for (uint8_t idx = (base >> 5) + 1; idx < 8; idx++) {
    _mask[idx] = 0;
    _known |= (uint8_t)(1u << idx);
  }
}

bool CapabilityTable::supported(uint8_t pid) const {
  if (pid == 0) return true;  // PID 00 is the discovery PID itself
  uint8_t idx = (pid - 1) >> 5;
  if (!(_known & (1u << idx))) return false;
  uint8_t bit = (pid - 1) & 0x1F;
  return (_mask[idx] >> (31 - bit)) & 1u;
}

bool CapabilityTable::hasNextRange(uint8_t base) const {
  if (base > 0xC0 || !(_known & (1u << (base >> 5)))) return false;
  return _mask[base >> 5] & 1u;
}

void CapabilityTable::clear() {
  for (auto& m : _mask) m = 0;
  _known = 0;
}

}  // namespace obd
}  // namespace roadnode
