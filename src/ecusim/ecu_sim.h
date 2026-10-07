#pragma once

#include <stddef.h>
#include <stdint.h>

#include "obd/obd_manager.h"

// Simulated OBD-II ECU for bench testing (issue 59). Pure logic: frames in, frames out.
// No hardware, no MeshCore, so it is unit-tested on the host against the real ObdManager.
namespace roadnode {
namespace ecusim {

constexpr size_t MAX_DTCS = 20;

struct DtcSet {
  uint16_t raw[MAX_DTCS] = {0};
  size_t count = 0;
};

struct EcuScenario {
  // Mode 01 values. Overwritten every update() while drive_cycle is on (speed, rpm).
  float speed_kmh = 0;
  float rpm = 800;
  float coolant_c = 90;
  float load_pct = 20;
  float intake_c = 25;
  float fuel_pct = 50;
  float module_volts = 13.8f;

  DtcSet stored;     // mode 03
  DtcSet pending;    // mode 07
  DtcSet permanent;  // mode 0A

  char vin[18] = "1HGCM82633A004352";
  bool vin_supported = true;

  bool silent = false;       // answers nothing at all (bus silence on the ECU side)
  uint32_t ignore_modes = 0; // bit n set: mode n is never answered (many ECUs ignore 0A)
  bool drive_cycle = false;  // speed follows driveSpeedKmh()
};

class EcuSim {
public:
  static constexpr uint32_t FUNCTIONAL_REQUEST_ID = 0x7DF;
  static constexpr uint32_t PHYSICAL_REQUEST_ID = 0x7E0;
  static constexpr uint32_t RESPONSE_ID = 0x7E8;

  EcuScenario& scenario() { return _sc; }
  const EcuScenario& scenario() const { return _sc; }

  // Advances the drive cycle and integrates distance. Call at least every ~100 ms.
  void update(uint32_t now_ms);

  // Feed every frame seen on the bus. Responses are queued for nextFrame().
  void onFrame(const obd::CanFrame& f);
  bool nextFrame(obd::CanFrame& out);

  // Metres driven by the simulated speed since the drive cycle was switched on (known truth).
  double distanceM() const { return _distance_m; }
  void resetDistance() { _distance_m = 0; }

  // Speed at t seconds into the cycle: 10 s stopped, 20 s ramp to 60, 60 s cruise, 20 s ramp down, 20 s stopped.
  static float driveSpeedKmh(uint32_t cycle_ms);
  static constexpr uint32_t CYCLE_MS = 130000;

  uint32_t requests() const { return _requests; }
  // Requests for a mode outside 01/03/07/09/0A (e.g. 04 clear codes). The node must never send one.
  uint32_t forbiddenRequests() const { return _forbidden; }
  uint8_t lastForbiddenMode() const { return _last_forbidden; }

private:
  static constexpr size_t QUEUE = 16;
  static constexpr size_t MAX_PAYLOAD = 64;

  void handleRequest(const uint8_t* d, uint8_t n);
  void respond(const uint8_t* payload, size_t len);
  void negative(uint8_t mode, uint8_t nrc);
  void mode01(uint8_t pid);
  void dtcMode(uint8_t mode, const DtcSet& s);
  void mode09(uint8_t pid);
  void push(const obd::CanFrame& f);
  void sendConsecutive();

  EcuScenario _sc;
  obd::CanFrame _q[QUEUE];
  size_t _qh = 0, _qn = 0;

  // Multi-frame response waiting for the flow-control frame.
  uint8_t _pend[MAX_PAYLOAD];
  size_t _pend_len = 0, _pend_sent = 0;
  bool _awaiting_fc = false;
  uint8_t _pend_seq = 1;

  uint32_t _last_ms = 0;
  uint32_t _cycle_ms = 0;
  bool _have_last = false;
  double _distance_m = 0;
  uint32_t _requests = 0, _forbidden = 0;
  uint8_t _last_forbidden = 0;
};

}  // namespace ecusim
}  // namespace roadnode
