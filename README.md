# RoadNode

Low-power OBD-II vehicle telemetry node. Reads standard OBD-II over CAN, integrates vehicle speed into distance, and reports over [MeshCore](https://github.com/meshcore-dev/MeshCore) as a sensor node. See [`plan.md`](plan.md) for the full plan and [`docs/meshcore.md`](docs/meshcore.md) for the MeshCore integration notes.

## Layout

- `vendor/MeshCore`: unmodified git submodule, pinned. Never edit it.
- `src/`: RoadNode code (`can/ obd/ vehicle/ storage/ gps/ telemetry/`). Only `telemetry/` includes MeshCore headers.
- `platformio.ini`: env `heltec_v4_roadnode`.

```sh
git clone --recurse-submodules https://github.com/mcfitz2/RoadNode.git
pio run -e heltec_v4_roadnode
```

## Hardware

- **Board:** Heltec WiFi LoRa 32 **V4.3**, no OLED (KCT8103L PA, 2 MB PSRAM). Uses MeshCore's plain `heltec_v4` pin map, not `heltec_v4_r8`. Verify the "V4.3" silkscreen on arrival.
- **CAN transceiver:** SN65HVD230 (3.3 V). No 120 ohm termination: we attach to an existing bus.

### CAN wiring

| Function | Heltec V4 GPIO | SN65HVD230 | OBD-II pin |
| --- | --- | --- | --- |
| CAN TX | GPIO3 | TXD | |
| CAN RX | GPIO4 | RXD | |
| Ground | GND | GND | 4 or 5 |
| Logic power | 3.3 V | VCC | |
| CAN High | | CANH | 6 |
| CAN Low | | CANL | 14 |

Do not connect OBD pin 16 (+12 V) to anything yet. Power the board from USB-C or Li-ion.

The env does not set MeshCore's `ENV_PIN_SDA/SCL`, which the stock `heltec_v4_sensor` env points at GPIO3/4. Keep it that way.

### Pins in use by the board (V4 / V4.3, per MeshCore's `heltec_v4` variant)

| GPIO | Function |
| --- | --- |
| 0 | User button |
| 1 | Battery voltage read |
| 2, 5, 7, 46 | LoRa PA / FEM control |
| 8-14 | SX1262 (NSS 8, SCLK 9, MOSI 10, MISO 11, RST 12, BUSY 13, DIO1 14) |
| 17, 18, 21 | OLED SDA, SCL, RST (free on the no-OLED board, but avoid) |
| 35 | TX LED |
| 36 | VEXT enable (active HIGH) |
| 37 | Battery ADC control |
| 34, 38, 39, 42 | GNSS enable (active LOW), RX, TX, reset (active LOW). Reserved for GPS |
| 40, 41 | Plan reserved these for GNSS wake/PPS; unconfirmed against the Heltec schematic |

R8 boards differ (VEXT=40, GNSS enable=42, LED=46, octal PSRAM on GPIO33-37). Not our board.

## Design principles

1. Do not depend on proprietary CAN decoding for basic mileage.
2. Use standard OBD-II wherever possible.
3. Calculate distance locally.
4. Persist mileage independently of MeshCore.
5. Do not transmit raw CAN traffic over MeshCore.
6. Keep vehicle-specific behavior in profiles.
7. Keep DTC reading separate from DTC clearing.
8. Design parked power consumption as a first-class requirement.
9. Reserve the V4 GNSS pins.
10. Keep the CAN hardware electrically independent from the LoRa hardware.
11. Start with listen-only CAN monitoring before transmitting anything to a vehicle.
12. Test the 2006 RAV4 first; it is the reference implementation.
