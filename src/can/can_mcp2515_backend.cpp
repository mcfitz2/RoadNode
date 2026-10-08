#include "can_manager.h"

#ifdef CAN_BACKEND_MCP2515

#include <Arduino.h>
#include <SPI.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mcp2515.h"

namespace roadnode {
namespace can {

static const char* TAG = "can";

namespace {

constexpr uint32_t SPI_HZ = 4000000;  // chip allows 10 MHz; stay slow for jumper wires

class ArduinoSpi : public SpiBus {
public:
  void attach(SPIClass* spi, int cs) {
    _spi = spi;
    _cs = cs;
  }
  void transfer(const uint8_t* tx, uint8_t* rx, size_t n) override {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi->transferBytes(tx, rx, n);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
  }
  void delayMs(uint32_t ms) override { delay(ms); }

private:
  SPIClass* _spi = nullptr;
  int _cs = -1;
};

SPIClass s_spi_bus(HSPI);  // separate from the LoRa radio's bus
ArduinoSpi s_bus;
Mcp2515 s_chip(s_bus);

SemaphoreHandle_t s_lock = nullptr;  // serialises every SPI transaction
SemaphoreHandle_t s_irq = nullptr;
QueueHandle_t s_rx = nullptr;
TaskHandle_t s_task = nullptr;
volatile bool s_stop = false;
bool s_running = false;
Mode s_mode = Mode::ListenOnly;
int s_int_pin = -1;
uint32_t s_rx_missed = 0;
int s_last_err = 0;

enum { ERR_NONE = 0, ERR_NO_CHIP = 1, ERR_RESOURCES = 2 };

void IRAM_ATTR onInterrupt() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_irq, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// Drains the controller's two hardware buffers into the queue as soon as INT asserts. Without an INT pin it
// polls every tick.
void rxTask(void*) {
  while (!s_stop) {
    xSemaphoreTake(s_irq, pdMS_TO_TICKS(s_int_pin >= 0 ? 20 : 1));
    if (s_stop) break;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    Frame f;
    while (s_chip.receive(f)) {
      if (xQueueSend(s_rx, &f, 0) != pdTRUE) s_rx_missed++;
    }
    s_chip.poll();
    xSemaphoreGive(s_lock);
  }
  s_task = nullptr;
  vTaskDelete(nullptr);
}

}  // namespace

Result begin(const Config& cfg) {
  if (s_running) return Result::InvalidState;
  if (!s_chip.validBitrate(cfg.bitrate_bps, cfg.osc_hz)) {
    ESP_LOGE(TAG, "unsupported bitrate %u at %u Hz oscillator", (unsigned)cfg.bitrate_bps, (unsigned)cfg.osc_hz);
    return Result::InvalidBitrate;
  }

  if (!s_lock) s_lock = xSemaphoreCreateMutex();
  if (!s_irq) s_irq = xSemaphoreCreateBinary();
  s_rx = xQueueCreate(cfg.rx_queue_len, sizeof(Frame));
  if (!s_lock || !s_irq || !s_rx) {
    s_last_err = ERR_RESOURCES;
    return Result::DriverError;
  }

  pinMode(cfg.spi_cs, OUTPUT);
  digitalWrite(cfg.spi_cs, HIGH);
  s_spi_bus.begin(cfg.spi_sck, cfg.spi_miso, cfg.spi_mosi, -1);
  s_bus.attach(&s_spi_bus, cfg.spi_cs);

  if (!s_chip.begin(cfg.bitrate_bps, cfg.osc_hz, cfg.mode)) {
    ESP_LOGE(TAG, "MCP2515 not answering on SPI (sck=%d miso=%d mosi=%d cs=%d)", cfg.spi_sck, cfg.spi_miso,
             cfg.spi_mosi, cfg.spi_cs);
    s_spi_bus.end();
    vQueueDelete(s_rx);
    s_rx = nullptr;
    s_last_err = ERR_NO_CHIP;
    return Result::DriverError;
  }

  s_stop = false;
  s_rx_missed = 0;
  s_int_pin = cfg.spi_int;
  xSemaphoreTake(s_irq, 0);
  if (s_int_pin >= 0) {
    pinMode(s_int_pin, INPUT_PULLUP);
    attachInterrupt(s_int_pin, onInterrupt, FALLING);
  }
  if (xTaskCreate(rxTask, "can_rx", 4096, nullptr, 3, &s_task) != pdPASS) {
    if (s_int_pin >= 0) detachInterrupt(s_int_pin);
    s_spi_bus.end();
    vQueueDelete(s_rx);
    s_rx = nullptr;
    s_last_err = ERR_RESOURCES;
    return Result::DriverError;
  }

  s_running = true;
  s_mode = cfg.mode;
  s_last_err = ERR_NONE;
  ESP_LOGI(TAG, "MCP2515 started: %u bps, mode %d, sck=%d miso=%d mosi=%d cs=%d int=%d", (unsigned)cfg.bitrate_bps,
           (int)cfg.mode, cfg.spi_sck, cfg.spi_miso, cfg.spi_mosi, cfg.spi_cs, cfg.spi_int);
  return Result::Ok;
}

Result end() {
  if (!s_running) return Result::InvalidState;
  s_stop = true;
  xSemaphoreGive(s_irq);
  for (int i = 0; i < 100 && s_task; i++) delay(1);
  if (s_int_pin >= 0) detachInterrupt(s_int_pin);
  s_spi_bus.end();
  vQueueDelete(s_rx);
  s_rx = nullptr;
  s_running = false;
  return Result::Ok;
}

bool running() { return s_running; }

Result transmit(const Frame& f, uint32_t timeout_ms) {
  if (!s_running || s_mode == Mode::ListenOnly || f.dlc > 8) return Result::InvalidState;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return Result::Timeout;
  bool ok = s_chip.send(f);
  xSemaphoreGive(s_lock);
  return ok ? Result::Ok : Result::Timeout;
}

Result receive(Frame& f, uint32_t timeout_ms) {
  if (!s_running) return Result::InvalidState;
  return xQueueReceive(s_rx, &f, pdMS_TO_TICKS(timeout_ms)) == pdTRUE ? Result::Ok : Result::Timeout;
}

Result status(Status& s) {
  if (!s_running) return Result::InvalidState;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) return Result::Timeout;
  s_chip.poll();
  bool off = s_chip.busOff();
  s.tx_errors = s_chip.tec();
  s.rx_errors = s_chip.rec();
  s.tx_failed = s_chip.txAborted();
  s.rx_overrun = s_chip.rxOverruns();
  s.bus_errors = s_chip.busErrors();
  xSemaphoreGive(s_lock);
  s.rx_queued = uxQueueMessagesWaiting(s_rx);
  s.tx_queued = 0;
  s.rx_missed = s_rx_missed;
  s.arb_lost = 0;
  s.state = off ? "bus-off" : "running";
  return Result::Ok;
}

// The MCP2515 leaves bus-off by itself after 128 x 11 recessive bits, so there is nothing to restart.
Result recover() { return s_running ? Result::Ok : Result::InvalidState; }

bool busOff() {
  if (!s_running) return false;
  if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  bool off = s_chip.busOff();
  xSemaphoreGive(s_lock);
  return off;
}

int lastError() { return s_last_err; }

}  // namespace can
}  // namespace roadnode

#endif  // CAN_BACKEND_MCP2515
