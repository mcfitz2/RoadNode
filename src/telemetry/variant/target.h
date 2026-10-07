#pragma once

// Derived from MeshCore (MIT, see THIRD_PARTY_NOTICES.md).
// RoadNode copy of vendor/MeshCore/variants/heltec_v4/target.h. The only change
// is the type of `sensors` (which lets RoadNode add vehicle telemetry without editing the
// submodule) and of `board` (which checkpoints the odometer before reboot/poweroff). It shadows the vendor header via include order.
// Re-diff against the vendor file on every MeshCore bump (docs/meshcore.md).
#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <HeltecV4Board.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>
#include "telemetry/meshcore_telemetry.h"
#include "telemetry/roadnode_board.h"
#ifdef DISPLAY_CLASS
#ifdef HELTEC_LORA_V4_OLED
    #include <helpers/ui/SSD1306Display.h>
#elif defined(HELTEC_LORA_V4_TFT)
    #include <helpers/ui/ST7789LCDDisplay.h>
#endif
  #include <helpers/ui/MomentaryButton.h>
#endif

extern roadnode::RoadNodeBoard board;
extern WRAPPER_CLASS radio_driver;
extern AutoDiscoverRTCClock rtc_clock;
extern roadnode::RoadNodeSensorManager sensors;

#ifdef DISPLAY_CLASS
  extern DISPLAY_CLASS display;
  extern MomentaryButton user_btn;
#endif

bool radio_init();
mesh::LocalIdentity radio_new_identity();
