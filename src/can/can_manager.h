#pragma once

#include <stdint.h>

#include "driver/gpio.h"

// ESP32-S3 TWAI (CAN) controller wrapper. No MeshCore dependencies.
namespace roadnode {
namespace can {

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 3
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 4
#endif

enum class Mode : uint8_t {
  Loopback,    // no-ack self-test mode: frames sent with self-reception are received back
  ListenOnly,  // receive only; never transmits or acknowledges
  Normal,      // transmit and receive (needed for OBD requests)
};

enum class Result : uint8_t {
  Ok,
  InvalidBitrate,
  InvalidState,  // begin() while running, or end() while stopped
  DriverError,   // see lastError()
};

struct Config {
  gpio_num_t tx_pin = (gpio_num_t)CAN_TX_PIN;
  gpio_num_t rx_pin = (gpio_num_t)CAN_RX_PIN;
  Mode mode = Mode::ListenOnly;
  uint32_t bitrate_bps = 500000;  // 25k/50k/100k/125k/250k/500k/800k/1M
  uint32_t rx_queue_len = 64;
  uint32_t tx_queue_len = 8;
};

// Installs and starts the TWAI driver with an accept-all filter.
Result begin(const Config& cfg);

// Stops and uninstalls the driver.
Result end();

bool running();

// Last esp_err_t returned by the driver when a call returned Result::DriverError.
int lastError();

const char* resultName(Result r);

}  // namespace can
}  // namespace roadnode
