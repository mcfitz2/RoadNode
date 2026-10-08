// CAN sniffer / loopback self-test firmware. Independent of MeshCore.
//
// Build flags:
//   CAN_BITRATE          bits per second (default 500000)
//   SNIFFER_LOOPBACK     run the loopback self-test instead of sniffing
//   CAN_LOG_TIMESTAMPS   prefix each frame with a millisecond timestamp
//
// Normal sniffer runs in listen-only mode: it never transmits or acknowledges.
#include <Arduino.h>

#include "can/can_manager.h"

using namespace roadnode;

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

static const uint32_t STATUS_INTERVAL_MS = 5000;

static void printFrame(const can::Frame& f) {
  char line[96];
  int n = 0;
#ifdef CAN_LOG_TIMESTAMPS
  n += snprintf(line + n, sizeof(line) - n, "[%10lu] ", (unsigned long)millis());
#endif
  if (f.extended) {
    n += snprintf(line + n, sizeof(line) - n, "CAN 0x%08lX EXT", (unsigned long)f.id);
  } else {
    n += snprintf(line + n, sizeof(line) - n, "CAN 0x%03lX", (unsigned long)f.id);
  }
  if (f.rtr) {
    n += snprintf(line + n, sizeof(line) - n, " RTR DLC=%u", f.dlc);
  } else {
    n += snprintf(line + n, sizeof(line) - n, " DLC=%u", f.dlc);
    for (uint8_t i = 0; i < f.dlc; i++) n += snprintf(line + n, sizeof(line) - n, " %02X", f.data[i]);
  }
  Serial.println(line);
}

static void printStatus() {
  can::Status s;
  if (can::status(s) != can::Result::Ok) return;
  Serial.printf("# state=%s rx_queued=%lu tec=%lu rec=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu\n", s.state,
                (unsigned long)s.rx_queued, (unsigned long)s.tx_errors, (unsigned long)s.rx_errors,
                (unsigned long)s.rx_missed, (unsigned long)s.rx_overrun, (unsigned long)s.bus_errors);
}

#ifdef SNIFFER_LOOPBACK

static bool sameFrame(const can::Frame& a, const can::Frame& b) {
  return a.id == b.id && a.dlc == b.dlc && a.extended == b.extended && a.rtr == b.rtr &&
         memcmp(a.data, b.data, 8) == 0;
}

// Sends frames with varied IDs, DLCs and payloads, and checks each one comes back intact.
static void runLoopbackTest() {
  const uint32_t ids[] = {0x000, 0x001, 0x123, 0x456, 0x7DF, 0x7E8, 0x7FF};
  const uint32_t ext_ids[] = {0x800, 0x18DAF110, 0x1FFFFFFF};
  int sent = 0, failed = 0;

  for (int pass = 0; pass < 2; pass++) {
    const uint32_t* list = pass ? ext_ids : ids;
    int count = pass ? 3 : 7;
    for (int i = 0; i < count; i++) {
      for (uint8_t dlc = 0; dlc <= 8; dlc++) {
        can::Frame tx;
        tx.id = list[i];
        tx.dlc = dlc;
        tx.extended = pass == 1;
        tx.self = true;
        for (uint8_t b = 0; b < dlc; b++) tx.data[b] = (uint8_t)(0x10 * (b + 1) + dlc + i);

        sent++;
        can::Frame rx;
        can::Result r = can::transmit(tx, 100);
        if (r == can::Result::Ok) r = can::receive(rx, 100);
        if (r != can::Result::Ok || !sameFrame(tx, rx)) {
          failed++;
          Serial.printf("FAIL id=0x%lX dlc=%u: %s\n", (unsigned long)tx.id, dlc, can::resultName(r));
        }
      }
    }
  }

  can::Status s;
  bool clean = can::status(s) == can::Result::Ok && s.bus_errors == 0 && s.tx_failed == 0 &&
               s.tx_errors == 0 && s.rx_errors == 0 && s.rx_missed == 0 && s.rx_overrun == 0;
  printStatus();
  Serial.printf("LOOPBACK %s: %d/%d frames ok, error counters %s\n", (failed == 0 && clean) ? "PASS" : "FAIL",
                sent - failed, sent, clean ? "zero" : "NONZERO");
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // let the host attach to native USB

  can::Config cfg;
  cfg.mode = can::Mode::Loopback;
  cfg.bitrate_bps = CAN_BITRATE;
  can::Result r = can::begin(cfg);
  if (r != can::Result::Ok) {
    Serial.printf("can::begin failed: %s (err 0x%x)\n", can::resultName(r), can::lastError());
    return;
  }
  Serial.println("# loopback self-test");
  runLoopbackTest();
}

void loop() { delay(1000); }

#else

void setup() {
  Serial.begin(115200);
  delay(2000);  // let the host attach to native USB

  can::Config cfg;
  cfg.mode = can::Mode::ListenOnly;
  cfg.bitrate_bps = CAN_BITRATE;
  can::Result r = can::begin(cfg);
  if (r != can::Result::Ok) {
    Serial.printf("can::begin failed: %s (err 0x%x)\n", can::resultName(r), can::lastError());
    return;
  }
  Serial.printf("# listen-only sniffer, %lu bps\n", (unsigned long)CAN_BITRATE);
}

void loop() {
  static uint32_t last_status = 0;

  if (!can::running()) {
    delay(1000);
    return;
  }

  can::Frame f;
  if (can::receive(f, 50) == can::Result::Ok) printFrame(f);

  if (millis() - last_status >= STATUS_INTERVAL_MS) {
    last_status = millis();
    printStatus();
  }
}

#endif
