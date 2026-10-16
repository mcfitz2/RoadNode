#pragma once

#include <Arduino.h>

// The carrier board powers the CAN transceiver from Heltec VEXT (PROJ-685). The MeshCore node
// switches it through HeltecV4Board::periph_power; the standalone tools (sniffer, loopback,
// monitor) have no board object, so they call this before can::begin(). Harmless on a
// breadboard where the transceiver runs from 3V3.
#ifndef PIN_VEXT_EN
#define PIN_VEXT_EN 36
#endif
#ifndef PIN_VEXT_EN_ACTIVE
#define PIN_VEXT_EN_ACTIVE HIGH  // as MeshCore's heltec_v4 variant
#endif

namespace roadnode {
namespace can {

inline void powerOn() {
  pinMode(PIN_VEXT_EN, OUTPUT);
  digitalWrite(PIN_VEXT_EN, PIN_VEXT_EN_ACTIVE);
  delay(5);  // let the rail and transceiver settle before TWAI drives TX
}

}  // namespace can
}  // namespace roadnode
