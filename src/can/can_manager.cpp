#include "can_manager.h"

#ifndef CAN_BACKEND_MCP2515

#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"

namespace roadnode {
namespace can {

static const char* TAG = "can";

static bool s_running = false;
static Mode s_mode = Mode::ListenOnly;
static esp_err_t s_last_err = ESP_OK;

static bool timingFor(uint32_t bps, twai_timing_config_t& t) {
  switch (bps) {
    case 25000: t = TWAI_TIMING_CONFIG_25KBITS(); return true;
    case 50000: t = TWAI_TIMING_CONFIG_50KBITS(); return true;
    case 100000: t = TWAI_TIMING_CONFIG_100KBITS(); return true;
    case 125000: t = TWAI_TIMING_CONFIG_125KBITS(); return true;
    case 250000: t = TWAI_TIMING_CONFIG_250KBITS(); return true;
    case 500000: t = TWAI_TIMING_CONFIG_500KBITS(); return true;
    case 800000: t = TWAI_TIMING_CONFIG_800KBITS(); return true;
    case 1000000: t = TWAI_TIMING_CONFIG_1MBITS(); return true;
    default: return false;
  }
}

static twai_mode_t driverMode(Mode m) {
  switch (m) {
    case Mode::Loopback: return TWAI_MODE_NO_ACK;
    case Mode::ListenOnly: return TWAI_MODE_LISTEN_ONLY;
    default: return TWAI_MODE_NORMAL;
  }
}

Result begin(const Config& cfg) {
  if (s_running) return Result::InvalidState;

  twai_timing_config_t timing;
  if (!timingFor(cfg.bitrate_bps, timing)) {
    ESP_LOGE(TAG, "unsupported bitrate %u", (unsigned)cfg.bitrate_bps);
    return Result::InvalidBitrate;
  }

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(cfg.tx_pin, cfg.rx_pin, driverMode(cfg.mode));
  general.rx_queue_len = cfg.rx_queue_len;
  general.tx_queue_len = cfg.tx_queue_len;
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  s_last_err = twai_driver_install(&general, &timing, &filter);
  if (s_last_err != ESP_OK) {
    ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(s_last_err));
    return Result::DriverError;
  }
  s_last_err = twai_start();
  if (s_last_err != ESP_OK) {
    ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(s_last_err));
    twai_driver_uninstall();
    return Result::DriverError;
  }

  s_running = true;
  s_mode = cfg.mode;
  ESP_LOGI(TAG, "started: %u bps, mode %d, tx=%d rx=%d", (unsigned)cfg.bitrate_bps, (int)cfg.mode,
           (int)cfg.tx_pin, (int)cfg.rx_pin);
  return Result::Ok;
}

Result end() {
  if (!s_running) return Result::InvalidState;

  s_last_err = twai_stop();
  if (s_last_err != ESP_OK) {
    ESP_LOGE(TAG, "twai_stop: %s", esp_err_to_name(s_last_err));
    return Result::DriverError;
  }
  s_last_err = twai_driver_uninstall();
  if (s_last_err != ESP_OK) {
    ESP_LOGE(TAG, "twai_driver_uninstall: %s", esp_err_to_name(s_last_err));
    return Result::DriverError;
  }
  s_running = false;
  return Result::Ok;
}

bool running() { return s_running; }

static TickType_t ticks(uint32_t ms) { return pdMS_TO_TICKS(ms); }

Result transmit(const Frame& f, uint32_t timeout_ms) {
  if (!s_running || s_mode == Mode::ListenOnly || f.dlc > 8) return Result::InvalidState;

  twai_message_t m = {};
  m.identifier = f.id;
  m.data_length_code = f.dlc;
  m.extd = f.extended;
  m.rtr = f.rtr;
  m.self = f.self;
  memcpy(m.data, f.data, f.dlc);

  s_last_err = twai_transmit(&m, ticks(timeout_ms));
  if (s_last_err == ESP_ERR_TIMEOUT) return Result::Timeout;
  return s_last_err == ESP_OK ? Result::Ok : Result::DriverError;
}

Result receive(Frame& f, uint32_t timeout_ms) {
  if (!s_running) return Result::InvalidState;

  twai_message_t m;
  s_last_err = twai_receive(&m, ticks(timeout_ms));
  if (s_last_err == ESP_ERR_TIMEOUT) return Result::Timeout;
  if (s_last_err != ESP_OK) return Result::DriverError;

  f.id = m.identifier;
  f.dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
  f.extended = m.extd;
  f.rtr = m.rtr;
  f.self = m.self;
  memset(f.data, 0, sizeof(f.data));
  if (!m.rtr) memcpy(f.data, m.data, f.dlc);
  return Result::Ok;
}

Result status(Status& s) {
  if (!s_running) return Result::InvalidState;

  twai_status_info_t i;
  s_last_err = twai_get_status_info(&i);
  if (s_last_err != ESP_OK) return Result::DriverError;

  s.rx_queued = i.msgs_to_rx;
  s.tx_queued = i.msgs_to_tx;
  s.tx_errors = i.tx_error_counter;
  s.rx_errors = i.rx_error_counter;
  s.tx_failed = i.tx_failed_count;
  s.rx_missed = i.rx_missed_count;
  s.rx_overrun = i.rx_overrun_count;
  s.arb_lost = i.arb_lost_count;
  s.bus_errors = i.bus_error_count;
  switch (i.state) {
    case TWAI_STATE_STOPPED: s.state = "stopped"; break;
    case TWAI_STATE_RUNNING: s.state = "running"; break;
    case TWAI_STATE_BUS_OFF: s.state = "bus-off"; break;
    default: s.state = "recovering"; break;
  }
  return Result::Ok;
}

Result recover() {
  if (!s_running) return Result::InvalidState;
  twai_status_info_t i;
  s_last_err = twai_get_status_info(&i);
  if (s_last_err != ESP_OK) return Result::DriverError;
  if (i.state == TWAI_STATE_BUS_OFF) {
    s_last_err = twai_initiate_recovery();
  } else if (i.state == TWAI_STATE_STOPPED) {
    s_last_err = twai_start();
  } else {
    return Result::Ok;  // running or already recovering
  }
  return s_last_err == ESP_OK ? Result::Ok : Result::DriverError;
}

bool busOff() {
  if (!s_running) return false;
  twai_status_info_t i;
  return twai_get_status_info(&i) == ESP_OK && i.state == TWAI_STATE_BUS_OFF;
}

int lastError() { return (int)s_last_err; }

}  // namespace can
}  // namespace roadnode

#endif  // !CAN_BACKEND_MCP2515
