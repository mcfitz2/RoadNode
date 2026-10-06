#include "can_manager.h"

#include "driver/twai.h"
#include "esp_log.h"

namespace roadnode {
namespace can {

static const char* TAG = "can";

static bool s_running = false;
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

int lastError() { return (int)s_last_err; }

const char* resultName(Result r) {
  switch (r) {
    case Result::Ok: return "ok";
    case Result::InvalidBitrate: return "invalid bitrate";
    case Result::InvalidState: return "invalid state";
    default: return "driver error";
  }
}

}  // namespace can
}  // namespace roadnode
