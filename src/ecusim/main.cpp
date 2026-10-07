// Bench ECU simulator for the Adafruit Feather RP2040 CAN (MCP25625). Issue 59.
// Answers OBD-II requests from the RoadNode device under test; see docs/ecusim.md.
// Serial control at 115200: type `status` (commands in ecu_commands.h).
//
// Build flags:
//   CAN_BITRATE          bits per second (default 500000)
//   ECUSIM_CAN_CLOCK_HZ  MCP25625 oscillator (default 16000000, UNCONFIRMED for this board:
//                        a wrong value gives the wrong bit rate and no traffic)
#include <Adafruit_MCP2515.h>
#include <Arduino.h>

#include "ecusim/ecu_commands.h"
#include "ecusim/ecu_sim.h"

using namespace roadnode;

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif
#ifndef ECUSIM_CAN_CLOCK_HZ
#define ECUSIM_CAN_CLOCK_HZ 16000000
#endif

static Adafruit_MCP2515 mcp(PIN_CAN_CS);
static ecusim::EcuSim ecu;
static char line[96];
static size_t line_len = 0;

static void sendFrame(const obd::CanFrame& f) {
  mcp.beginPacket(f.id);
  for (uint8_t i = 0; i < f.dlc; i++) mcp.write(f.data[i]);
  mcp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);  // transceiver out of standby, else nothing reaches the bus
  pinMode(PIN_CAN_RESET, OUTPUT);
  digitalWrite(PIN_CAN_RESET, LOW);  // /RESET asserted briefly
  delay(10);
  digitalWrite(PIN_CAN_RESET, HIGH);
  delay(10);

  mcp.setClockFrequency(ECUSIM_CAN_CLOCK_HZ);
  if (!mcp.begin(CAN_BITRATE)) {
    Serial.println("MCP25625 init failed (check crystal setting and wiring)");
    while (1) delay(1000);
  }
  Serial.println("ECU simulator ready. Commands: status, cycle on|off, dtc 03 0301, vin none, silent on ...");
}

void loop() {
  ecu.update(millis());

  int size = mcp.parsePacket();
  if (size > 0 && !mcp.packetExtended() && !mcp.packetRtr()) {
    obd::CanFrame f;
    f.id = (uint32_t)mcp.packetId();
    f.dlc = (uint8_t)(size > 8 ? 8 : size);
    for (uint8_t i = 0; i < f.dlc; i++) f.data[i] = (uint8_t)mcp.read();
    ecu.onFrame(f);
    if (f.id == 0x7DF || f.id == 0x7E0) {
      if (f.dlc > 1 && (f.data[0] >> 4) == 0) {
        Serial.printf("# req mode %02X%s\n", f.data[1], f.data[1] == 0x04 ? "  <-- FORBIDDEN (clear codes)" : "");
      }
    }
  } else if (size > 0) {
    while (mcp.available()) mcp.read();  // drop extended/RTR frames
  }

  obd::CanFrame out;
  while (ecu.nextFrame(out)) sendFrame(out);

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (line_len) {
        line[line_len] = 0;
        char reply[160];
        if (!ecusim::handleEcuCommand(ecu, line, reply, sizeof(reply))) snprintf(reply, sizeof(reply), "? unknown command");
        Serial.println(reply);
        line_len = 0;
      }
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    }
  }
}
