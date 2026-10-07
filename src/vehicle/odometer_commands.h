#pragma once

#include <stdint.h>

// Admin commands for the device odometer (the car's own odometer cannot be read, so the device
// value drifts and needs resyncing by hand). MeshCore-free; the firmware passes an adapter that
// takes the vehicle task's lock.
namespace roadnode {
namespace vehicle {

constexpr uint32_t ODOMETER_MILES_MAX = 1000000;

class OdometerControl {
public:
  virtual ~OdometerControl() {}
  // total_mm: odometer; gps_mm: distance GPS added while OBD speed was unavailable, since boot.
  // False if the value could not be read (vehicle task busy).
  virtual bool read(uint64_t& total_mm, uint64_t& gps_mm) = 0;
  // Replace the odometer and persist it. False if busy or the write failed (old value kept).
  virtual bool setTotalMm(uint64_t total_mm) = 0;
};

// "odo" (show) and "odo set <miles>" (miles with at most one decimal, 0 to 1,000,000).
// Returns true if the command was recognised; reply gets a short text.
bool handleOdometerCommand(OdometerControl& odo, const char* command, char* reply);

// Exposed for tests. Tenths of a mile <-> millimetres, integer math, rounded to nearest.
uint64_t milesTenthsToMm(uint32_t tenths);
uint32_t mmToMilesTenths(uint64_t mm);

}  // namespace vehicle
}  // namespace roadnode
