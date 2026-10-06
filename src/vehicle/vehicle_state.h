#pragma once

#include <stdint.h>
#include <string.h>

#ifdef ARDUINO
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
#include <mutex>
#endif

// Output-agnostic vehicle state (plan section 15). No MeshCore dependency:
// telemetry, serial output and any future UI all read from here.
namespace roadnode {
namespace vehicle {

struct VehicleSnapshot {
  char vehicle_id[16] = {0};

  uint64_t total_mm = 0;
  uint64_t trip_mm = 0;
  bool trip_active = false;

  // Each live value is valid only if its has_* flag is set (never assume zero).
  bool has_speed = false;
  float speed_kmh = 0;
  bool has_rpm = false;
  float rpm = 0;
  bool has_battery = false;
  float battery_v = 0;

  bool vehicle_active = false;  // CAN bus traffic seen recently
  bool engine_running = false;
  bool obd_connected = false;   // ECU answering requests

  // Milliseconds on the node's clock; 0 with the matching ever_* flag clear = never.
  bool ever_can_activity = false;
  uint32_t last_can_activity_ms = 0;
  bool ever_obd_response = false;
  uint32_t last_obd_response_ms = 0;
};

// Single writer (OBD poller) publishes complete snapshots; any number of
// readers copy them. Readers never see a half-updated snapshot.
class VehicleTelemetry {
public:
  VehicleTelemetry() {
#ifdef ARDUINO
    _mutex = xSemaphoreCreateMutex();
#endif
  }
  VehicleTelemetry(const VehicleTelemetry&) = delete;
  VehicleTelemetry& operator=(const VehicleTelemetry&) = delete;

  void publish(const VehicleSnapshot& s) {
    lock();
    _snap = s;
    unlock();
  }

  VehicleSnapshot snapshot() const {
    lock();
    VehicleSnapshot copy = _snap;
    unlock();
    return copy;
  }

private:
#ifdef ARDUINO
  void lock() const { xSemaphoreTake(_mutex, portMAX_DELAY); }
  void unlock() const { xSemaphoreGive(_mutex); }
  SemaphoreHandle_t _mutex;
#else
  void lock() const { _mutex.lock(); }
  void unlock() const { _mutex.unlock(); }
  mutable std::mutex _mutex;
#endif
  VehicleSnapshot _snap;
};

}  // namespace vehicle
}  // namespace roadnode
