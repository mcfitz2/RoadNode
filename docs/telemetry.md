# Vehicle telemetry format

Spec for issue #31. Implementation: `src/telemetry/vehicle_lpp.h`. Tests: `test/test_telemetry`.

## Transport

Vehicle data is appended to MeshCore's standard sensor telemetry as
[Cayenne LPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp), so stock
MeshCore clients can already decode and display it. No custom packet type.

- MeshCore answers telemetry requests only for ACL clients, encrypted.
- SensorMesh's telemetry buffer is `MAX_PACKET_PAYLOAD - 4` = 180 bytes.
  A full vehicle record is 23 bytes (plus MeshCore's own channel 1 battery),
  leaving large headroom.
- LPP entry: `[channel][type][value]`, value is `uint32(float * multiplier)`, big-endian.

## Channels

| Ch | Field | LPP type | Unit / multiplier | Size | Present |
|----|-------|----------|-------------------|------|---------|
| 1 | MeshCore device battery | (MeshCore) | | | MeshCore, not RoadNode |
| 2 | Odometer | generic sensor | 0.1 km per count | 6 | always |
| 3 | Trip distance | distance | metres (0.001 resolution) | 6 | always |
| 4 | Speed | analog input | km/h, 0.01 signed | 4 | only if speed known |
| 6 | State flags | digital input | bit flags | 3 | always |
| 7 | Vehicle battery | voltage | volts, 0.01 | 4 | only if known |

Channels 5 (RPM), 8-9 (DTCs), 10-13 (coolant, load, intake air, fuel), 14-16 (GPS trip, speed,
heading) and 17 (trip GPS/OBD ratio) were removed in PROJ-680 and are not reused. RPM is still polled:
it feeds the engine-running and OBD-connected state, it is just not sent.

Fields with no fresh value are **omitted, never sent as zero**.

## Read-only OBD

**Nothing that reads or clears trouble codes or changes ECU state can be transmitted:** `ObdManager`
refuses every mode outside 01 (live data) and 09 (VIN) with `Status::Forbidden` before building a frame.

## State flags (channel 6)

| Bit | Meaning |
|-----|---------|
| 0x01 | vehicle active |
| 0x02 | engine running |
| 0x04 | trip active |
| 0x08 | OBD connected |

## Ranges

LPP carries values through a `float`, exact only to 2^24.

- Odometer: 0.1 km units, capped at 16,777,216 (about 1.68 million km). A raw 32-bit
  LPP distance (0.001 m) would overflow at about 4,295 km, hence the generic sensor.
- Trip: capped at 4,294,000 m (LPP distance limit), resolution 1 m after encode.

## Privacy

Live GPS is reported on MeshCore's channel 1 (stock behaviour), **only in replies to
telemetry requests**. Those are encrypted and answered only for clients in the node's
ACL, so location is private to authorised clients. It is never in adverts: they carry
the fixed `ADVERT_LAT/LON` prefs (0,0) while `adv_loc` stays at its default `prefs`.
Do not set `adv_loc share`.

Anyone with the admin/guest password can join the ACL and read location. The repo only
holds the placeholder `password` (so it can be public); **no real secret is ever committed**.
At deployment, set the real admin password on each unit over the admin CLI (`password <new>`,
stored in the node's flash prefs, not in the build), before the unit goes in a vehicle (#51).
The vehicle id is not in the payload.

## Alerts

Besides pull telemetry, the node pushes encrypted text alerts to ACL clients that
opted in (`PERM_RECV_ALERTS_LO` / `_HI`), via MeshCore's alert queue. Position is the
last valid GPS fix, or "(no GPS fix)" if none since boot.

| Alert | Priority | Condition |
|-------|----------|-----------|
| `Vehicle started at <lat>,<lon>` | low (one attempt) | operating: trip active or engine running |
| `Vehicle parked at <lat>,<lon>` | high (retries until ACK) | operated since boot, engine off, no trip, ECU no longer serving data |
| `Vehicle moving at <lat>,<lon>` | low (one attempt) | operating and speed >= 5 km/h, every 5 min |

The periodic report first fires one interval after engine start, then every interval
while moving (set `-D PERIODIC_ALERT_MINUTES=n`). If due while stopped it waits for the
next moving check. MeshCore cancels a queued alert when its condition drops, so the
condition is held true for a 2 minute window (clamped to half the interval) to cover the
send, then released so it can re-arm. Checks run on MeshCore's 60 s sensor read.

Each fires once per false->true edge; it is not a position stream. Logic:
`src/vehicle/alert_logic.h`; wiring: `src/node/main.cpp` (copy of the vendor
`simple_sensor/main.cpp`, re-diff on MeshCore bumps). Delivery is not durable: with no path it floods and gives up after retries.

### Private channel

Stock MeshCore apps show sensor nodes without a chat view, so the direct alerts above
are delivered and ACKed but not displayed. The node can also post each alert (same text,
same edges, battery alerts included) to a private group channel as `<node name>: <text>`,
which every client holding the channel secret displays.

Create a private channel in the app, copy its secret (32 or 64 hex chars), then on the
node's admin CLI (serial or remote):

| Command | Effect |
|---------|--------|
| `alert channel <hex secret>` | enable, persisted across reboots |
| `alert channel` | show on/off and key size; never echoes the secret |
| `alert channel off` | disable and erase the stored secret |

Channel posts are one flood each, not ACKed or retried. Anyone with the secret can read
them, including the position in the started/parked/moving texts; share the channel only
with people allowed to see where the vehicle is. Never commit the secret or paste it in an issue.

## Verification status

- Encode/decode round trips and size are unit tested on the host against a model of the
  CayenneLPP wire rules, not the real library.
- Firmware build instantiates the encoder with the real `CayenneLPP`, so it compiles
  against the real API, but real wire bytes have not been compared on a device or in a MeshCore client.

## Send policy (#32)

Speed is sampled locally about once a second and is never transmitted per sample. Over the mesh:

| What | When | Configurable |
|------|------|--------------|
| Telemetry reply | on request from an ACL client | n/a (pull) |
| `Vehicle started` | engine starts | no |
| `Vehicle moving` | every N minutes while moving | admin `alert periodic <min>` (0 = off, max 1440), persisted; default 5; `-D PERIODIC_ALERT_MINUTES` sets the first-boot default |
| `Vehicle parked` | trip ends | no |

`alert` shows the current interval. Admin commands only: they are not reachable by plain ACL clients.

**Raw CAN never goes on the mesh.** MeshCore-facing code (`src/telemetry`, `src/node`) only sees
`VehicleSnapshot`. `scripts/check_no_raw_can.sh` (run in CI) fails if either
directory includes the `can/` or OBD transport headers or names `CanFrame`/`CanBus`/`ObdManager`.

### Airtime estimate

Estimated, not measured. Time on air from the Semtech SX126x formula (8 symbol preamble, explicit
header, CRC, CR 4/5) for these packet sizes: telemetry reply at most 54 bytes (header 2 + hashes 2 + MAC 2 + padded cipher of
timestamp + LPP incl. channel 1 GPS; worked out when the vehicle record was 29 bytes, so now an upper
bound); alert 54 bytes.
Direct path assumed; each flood hop adds path bytes (1 per hop) and a retransmission by that hop.

| Radio setting | Reply / alert |
|---------------|---------------|
| SF7 BW62.5 | 0.21 s |
| SF8 BW62.5 | 0.37 s |
| SF10 BW250 | 0.31 s |
| SF11 BW250 | 0.58 s |

Radio settings are set per deployment through MeshCore, so check which row applies. Driving with the
default 5 minute report: 12 reports per hour, plus start and park, about 3 to 7 s of airtime per hour
per opted-in client (0.1 to 0.2 % duty cycle); high-priority alerts (parked) retry until ACKed,
so a poor link can multiply their share. At about 120 mA transmit current (SX1262 at +22 dBm, from
the datasheet, not measured on this board) 7 s per hour is roughly 0.25 mAh. Receive listening, which
MeshCore does continuously, and the advert interval are unchanged by this project and dominate battery use.

## GPS (#41)

**GNSS (#41)** reuses MeshCore's `MicroNMEALocationProvider` through `EnvironmentSensorManager`: it
parses NMEA, updates `node_lat/lon/altitude` once a second and sets the RTC from GPS time. No
second parser. `-D PERSISTANT_GPS` turns GNSS on at boot when the module is detected (stock default
is off until the admin `gps 1` setting). Pins are MeshCore's heltec_v4 map, not the original plan table:
RX 38, TX 39, reset 42 (active low), enable 34 (active low); see `docs/meshcore.md`. Plan's wake 40 and PPS 41 are not used.

GPS is used only for the position in alert texts (and channel 1, stock) and to set the RTC, which
stamps trip ends. GPS motion (#42), the GPS odometer gap fill and the OBD vs GPS trip comparison (#43)
were removed in PROJ-680; the odometer counts OBD speed only.

Fix, NMEA parsing and the RTC set from GPS time are hardware checks: `docs/car-test.md` checks 1-2.
