#pragma once

#include <stdint.h>

// Trip detection state machine (plan section 11). Pure logic: it reports
// events and leaves distance and storage to the caller.
namespace roadnode {
namespace vehicle {

struct TripConfig {
  uint32_t end_timeout_ms = 5UL * 60UL * 1000UL;  // inactivity before a trip ends
};

struct TripInput {
  bool vehicle_active = false;  // ignition / bus activity
  bool engine_running = false;  // e.g. RPM > 0
  uint8_t speed_kmh = 0;
};

enum class TripEvent : uint8_t { None, Started, Ended };

class Trip {
public:
  explicit Trip(const TripConfig& cfg = TripConfig()) : _cfg(cfg) {}

  // Call at every sample. Returns Started or Ended on a transition.
  // A trip runs while the vehicle is "operating": active, and engine running
  // or moving. It ends once it has been not operating for end_timeout_ms;
  // becoming operating again before then cancels the countdown.
  TripEvent update(uint32_t now_ms, const TripInput& in);

  bool active() const { return _active; }

  // Abandon the current trip without an Ended event (e.g. before sleep after checkpointing).
  void reset();

private:
  TripConfig _cfg;
  bool _active = false;
  bool _idle_timing = false;
  uint32_t _idle_since_ms = 0;
};

}  // namespace vehicle
}  // namespace roadnode
