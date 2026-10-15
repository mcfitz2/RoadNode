#!/bin/sh
# Downloads the Heltec WiFi LoRa 32 V4 STEP model (14 MB, kept out of git) used
# for the 3D fit check. Source: github.com/The1TrueJoe/heltec-lora32-kicad (MIT).
set -e
cd "$(dirname "$0")/../lib/rn_carrier.3dshapes"
curl -fsSLo Heltec_WiFi_LoRa32_V4.step \
  https://raw.githubusercontent.com/The1TrueJoe/heltec-lora32-kicad/main/Heltec_LoRa32.3dshapes/Heltec_WiFi_LoRa32_V4.step
