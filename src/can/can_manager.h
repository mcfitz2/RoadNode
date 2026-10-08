#pragma once

#include <stdint.h>

#include "can_types.h"
#include "driver/gpio.h"

// CAN controller wrapper. Backend is chosen at build time: the ESP32-S3 TWAI peripheral (default, needs an
// external transceiver) or an MCP2515 over SPI (-D CAN_BACKEND_MCP2515, e.g. the Adafruit PiCowbell CAN Bus).
// No MeshCore dependencies.
namespace roadnode {
namespace can {

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 3
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 4
#endif
// MCP2515 wiring on the Heltec V4 header. GPIO 8-14 belong to the LoRa radio and 34/38/39/42 to the GNSS
// connector, so the controller gets its own SPI bus on otherwise unused pins.
#ifndef CAN_SPI_SCK
#define CAN_SPI_SCK 47
#endif
#ifndef CAN_SPI_MISO
#define CAN_SPI_MISO 41
#endif
#ifndef CAN_SPI_MOSI
#define CAN_SPI_MOSI 48
#endif
#ifndef CAN_SPI_CS
#define CAN_SPI_CS 3
#endif
#ifndef CAN_SPI_INT
#define CAN_SPI_INT 4
#endif
#ifndef CAN_MCP2515_OSC_HZ
#define CAN_MCP2515_OSC_HZ 16000000  // PiCowbell CAN Bus crystal (Y1, 16 MHz)
#endif

struct Config {
  // TWAI backend (ESP32 on-chip controller, external transceiver).
  gpio_num_t tx_pin = (gpio_num_t)CAN_TX_PIN;
  gpio_num_t rx_pin = (gpio_num_t)CAN_RX_PIN;
  // MCP2515 backend (CAN_BACKEND_MCP2515): SPI controller with its own transceiver.
  int spi_sck = CAN_SPI_SCK;
  int spi_miso = CAN_SPI_MISO;
  int spi_mosi = CAN_SPI_MOSI;
  int spi_cs = CAN_SPI_CS;
  int spi_int = CAN_SPI_INT;           // active-low interrupt; -1 = poll
  uint32_t osc_hz = CAN_MCP2515_OSC_HZ;
  Mode mode = Mode::ListenOnly;
  uint32_t bitrate_bps = 500000;  // TWAI: 25k..1M. MCP2515: 25k/50k/100k/125k/250k/500k
  uint32_t rx_queue_len = 64;
  uint32_t tx_queue_len = 8;
};

// Starts the controller with an accept-all filter.
Result begin(const Config& cfg);

// Stops the controller.
Result end();

bool running();

// Queues a frame for transmission. Refused (InvalidState) in ListenOnly mode.
Result transmit(const Frame& f, uint32_t timeout_ms);

// Waits up to timeout_ms for a frame. Returns Timeout if none arrived.
Result receive(Frame& f, uint32_t timeout_ms);

Result status(Status& s);

// Bus-off recovery: starts the driver's recovery sequence when bus-off, and
// restarts the controller once it has stopped. Safe to call repeatedly.
Result recover();

// True while the controller is in the bus-off state.
bool busOff();

// Backend error code when a call returned Result::DriverError (TWAI: esp_err_t; MCP2515: 1 = chip not
// answering on SPI, 2 = out of RTOS resources).
int lastError();

}  // namespace can
}  // namespace roadnode
