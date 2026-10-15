# Bench ECU simulator (issue 59)

An Adafruit Feather RP2040 CAN (MCP25625) that pretends to be the vehicle's ECU, so the real
RoadNode firmware can be tested on a bench without the car. Code: `src/ecusim/`. Env: `rp2040_ecusim`.
Logic is MeshCore- and hardware-free and unit tested on the host against the real `ObdManager`
(`test/test_ecusim`).

## Wiring

```
 Heltec V4.3 + SN65HVD230          Feather RP2040 CAN (simulator)
   CANH  ------------------------  CAN H
   CANL  ------------------------  CAN L
   GND   ------------------------  GND (terminal block middle)
```

- Bench bus needs exactly two 120 ohm terminators. The Feather has one on by default (`Term` jumper).
  The SN65HVD230 board needs its own at the other end. **In the car the node must have none** (README).
- The SN65HVD230 board has two SMD resistors marked 151 (150 ohm) and 103 (10 kohm). Whether 151 is a
  fixed terminator is unverified: measure CANH-CANL unpowered when the board arrives. If it is, it must be
  removed before the car.
- Power the node from the bench supply or USB. Power the Feather from USB (also the serial console).

## Board facts (Adafruit guide and arduino-pico variant)

SPI SCK 14, MOSI 15, MISO 8; CS 19, RESET 18, INT 22, STANDBY 16 (driven low or the transceiver never
transmits). **MCP25625 crystal frequency is unconfirmed**; the firmware assumes 16 MHz
(`ECUSIM_CAN_CLOCK_HZ`). If nothing is seen on the bus, check this first.

## Build

```
pio run -e rp2040_ecusim -t upload
pio device monitor -e rp2040_ecusim
```

## What it answers (500 kbit/s, 11-bit, requests 0x7DF/0x7E0, replies 0x7E8)

- Mode 01: supported-PID ranges (00/20/40, stops after 40), PIDs 0C, 0D, 42. Others get negative response 0x12.
- Mode 09: PID 00 and 02 (VIN, 17 chars, multi-frame with flow control) or negative response when VIN is switched off.
- Anything outside 01/09 is counted as **forbidden** and answered 0x11. The node must never send mode 04.

## Serial commands (115200, one per line)

| Command | Effect |
|---|---|
| `status` | Current scenario, request count, forbidden count, driven distance |
| `cycle on` / `cycle off` | Drive cycle: 10 s stopped, 20 s ramp to 60 km/h, 60 s cruise, 20 s ramp down, 20 s stopped; repeats. `on` resets the known distance |
| `speed N`, `rpm N`, `volts V` | Set a value (speed/rpm are overwritten while the cycle runs) |
| `vin <17 chars>` / `vin none` | Set the VIN, or make mode 09 unsupported |
| `silent on` / `silent off` | Answer nothing |
| `delay <ms>` | Hold every reply back this long (the generic profile times out at 200 ms) |
| `pending <0-8>` | Send this many NRC 0x78 "response pending" frames before the real reply |
| `corrupt none\|short\|skipcf` | `short`: single-frame reply with a DLC too small for its length byte; `skipcf`: drop the first consecutive frame of a multi-frame reply (the VIN) |
| `ignore 09` / `unignore 09` | Never answer one mode (some real ECUs ignore 09 behind a gateway) |

The driven distance is the exact integral of the simulated speed, so odometer drift can be checked:
after one full cycle the truth is about 1,333 m (`status` prints it).

## What the bench can and cannot show

Can: real firmware end to end against a known ECU: discovery, the speed/RPM/voltage poll, VIN timing and
deferral while moving, LPP bytes, alerts, mileage checkpoints, NVS persistence over power cycles, admin
commands over the mesh, odometer accuracy against the known integral, power-off checkpointing with the bench supply.

Cannot: GPS (#41; bench is in a basement), real vehicle quirks.
The simulator speaks the standard; a real ECU may differ (response timing, 0x78 pending, extra frames).

## Not yet implemented

Bus-off. The simulator cannot force it: the node's CAN controller has to be driven into it, e.g. by shorting
CANH to CANL or running the bus at a mismatched bitrate. Tracked in #59.

## Verification status

Host tests (all pass, including delay/pending/corrupt fault injection) and the `rp2040_ecusim` env compiles (CI builds it). Not run on the RP2040; the crystal, library behaviour
(`Adafruit_MCP2515` receive-all, 11-bit frames) and wiring are unverified on hardware.
