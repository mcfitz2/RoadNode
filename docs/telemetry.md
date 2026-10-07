# Vehicle telemetry format

Spec for issue #31. Implementation: `src/telemetry/vehicle_lpp.h`. Tests: `test/test_telemetry`.

## Transport

Vehicle data is appended to MeshCore's standard sensor telemetry as
[Cayenne LPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp), so stock
MeshCore clients can already decode and display it. No custom packet type.

- MeshCore answers telemetry requests only for ACL clients, encrypted.
- SensorMesh's telemetry buffer is `MAX_PACKET_PAYLOAD - 4` = 180 bytes.
  A full vehicle record is 29 bytes (plus MeshCore's own channel 1 battery),
  leaving large headroom.
- LPP entry: `[channel][type][value]`, value is `uint32(float * multiplier)`, big-endian.

## Channels

| Ch | Field | LPP type | Unit / multiplier | Size | Present |
|----|-------|----------|-------------------|------|---------|
| 1 | MeshCore device battery | (MeshCore) | | | MeshCore, not RoadNode |
| 2 | Odometer | generic sensor | 0.1 km per count | 6 | always |
| 3 | Trip distance | distance | metres (0.001 resolution) | 6 | always |
| 4 | Speed | analog input | km/h, 0.01 signed | 4 | only if speed known |
| 5 | Engine RPM | generic sensor | rpm | 6 | only if RPM known |
| 6 | State flags | digital input | bit flags | 3 | always |
| 7 | Vehicle battery | voltage | volts, 0.01 | 4 | only if known |
| 8 | DTC count | digital input | total codes the ECU reported, capped 255 | 3 | only after a successful DTC read (0 = none) |
| 9 | DTC entry (repeated) | generic sensor | `kind << 16 \| raw`, up to 12 entries | 6 each | one per code |
| 10 | Coolant temp | temperature | 0.1 C | 4 | only if PID 05 supported and read within 30 s |
| 11 | Engine load | percentage | 0-100 % | 3 | PID 04, same rule |
| 12 | Intake air temp | temperature | 0.1 C | 4 | PID 0F, same rule |
| 13 | Fuel level | percentage | 0-100 % | 3 | PID 2F, same rule |
| 14 | GPS trip distance | distance | metres since trip start, cap 4,294,000 | 6 | needs a current GNSS fix |
| 15 | GPS speed | analog input | km/h from position change | 4 | needs a fix and two accepted steps |
| 16 | GPS heading | direction | degrees 0-359 | 4 | after the first accepted step |
| 17 | Last trip GPS/OBD ratio | analog input | percent, 0.1 resolution (98.3 = GPS saw 98.3% of the OBD distance) | 4 | after a finished trip with a valid comparison; location-permission requesters only |

Fields with no fresh value are **omitted, never sent as zero**.

## DTCs (channels 8 and 9)

Read-only: modes 03 (stored), 07 (pending) and 0A (permanent), polled every 30 s while the
ECU answers, but only when the vehicle is stopped (under 1 km/h): a DTC read can block the poll task
for a few request timeouts and would stretch the gap between speed samples. If the vehicle does not stop
for 10 minutes a read is forced. New codes can therefore be reported late on a long drive. Each entry on channel 9 is `kind << 16 | raw`, where kind is 1 stored,
2 pending, 3 permanent and raw is the two wire bytes. Decode: top 2 bits of the high byte
pick P/C/B/U, next 2 bits the first digit, then three hex digits (`0x0301` is `P0301`).
Worst case (12 codes) is 29 + 3 + 72 = 104 bytes (120 with the four slow values), plus MeshCore's channel 1, within the 180 limit.
If the ECU reports more than 12, the count (channel 8) still shows the real total.
A silent mode (many ECUs ignore 0A) is skipped; DTCs are omitted until stored (03) has answered.
**Nothing that clears codes can be transmitted:** `ObdManager` refuses every mode outside
01/03/07/09/0A with `Status::Forbidden` before building a frame.

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

Besides pull telemetry, the node pushes two encrypted text alerts to ACL clients that
opted in (`PERM_RECV_ALERTS_LO` / `_HI`), via MeshCore's alert queue. Position is the
last valid GPS fix, or "(no GPS fix)" if none since boot.

| Alert | Priority | Condition |
|-------|----------|-----------|
| `Vehicle started at <lat>,<lon>` | low (one attempt) | engine running |
| `Vehicle parked at <lat>,<lon>` | high (retries until ACK) | driven, engine off, CAN bus quiet |
| `Vehicle moving at <lat>,<lon>` | low (one attempt) | engine running and speed >= 5 km/h, every 5 min |
| `Vehicle DTC: P0301 P0420 +n` | high (retries until ACK) | a code not seen before appears |

The DTC alert lists up to 6 current codes (`+n` for the rest) and does not include position.
The first successful read after boot is the baseline and does not alert, so a code that was
already stored does not re-alert every ignition cycle; a code that disappears and returns alerts
again. The consequence: a pre-existing code is only visible via pull telemetry, not an alert.

The periodic report first fires one interval after engine start, then every interval
while moving (set `-D PERIODIC_ALERT_MINUTES=n`). If due while stopped it waits for the
next moving check. MeshCore cancels a queued alert when its condition drops, so the
condition is held true for a 2 minute window (clamped to half the interval) to cover the
send, then released so it can re-arm. Checks run on MeshCore's 60 s sensor read.

Each fires once per false->true edge; it is not a position stream. Logic:
`src/vehicle/alert_logic.h`; wiring: `src/node/main.cpp` (copy of the vendor
`simple_sensor/main.cpp`, re-diff on MeshCore bumps). Delivery is not durable: with no path it floods and gives up after retries.

## Verification status

- Encode/decode round trips and size are unit tested on the host against a model of the
  CayenneLPP wire rules, not the real library.
- Firmware build instantiates the encoder with the real `CayenneLPP`, so it compiles
  against the real API, but real wire bytes have not been compared on a device or in a MeshCore client.

## Slow engine values (#17)

Channels 10-13 are polled one PID per 2.5 s, round-robin (each refreshed about every 10 s), so
speed sampling keeps its normal spacing; the poller test checks distance stays within 1.25 % with
them enabled. A value is sent only while its last read is under 30 s old and the ECU advertised the
PID. Values are checked against decoders with byte vectors in `test_obd`; not yet verified against a
scan tool on the RAV4.

## Send policy (#32)

Speed is sampled locally about once a second and is never transmitted per sample. Over the mesh:

| What | When | Configurable |
|------|------|--------------|
| Telemetry reply | on request from an ACL client | n/a (pull) |
| `Vehicle started` | engine starts | no |
| `Vehicle moving` | every N minutes while moving | admin `alert periodic <min>` (0 = off, max 1440), persisted; default 5; `-D PERIODIC_ALERT_MINUTES` sets the first-boot default |
| `Vehicle parked` | trip ends | no |
| `Vehicle DTC` | new code | no |

`alert` shows the current interval. Admin commands only: they are not reachable by plain ACL clients.

**Raw CAN never goes on the mesh.** MeshCore-facing code (`src/telemetry`, `src/node`) only sees
`VehicleSnapshot` and DTC decoding. `scripts/check_no_raw_can.sh` (run in CI) fails if either
directory includes the `can/` or OBD transport headers or names `CanFrame`/`CanBus`/`ObdManager`.

### Airtime estimate

Estimated, not measured. Time on air from the Semtech SX126x formula (8 symbol preamble, explicit
header, CRC, CR 4/5) for these packet sizes: telemetry reply 54 bytes (header 2 + hashes 2 + MAC 2 +
64 byte padded cipher of timestamp + 44 byte LPP incl. GPS); with 12 DTCs 134 bytes; alert 54 bytes.
Direct path assumed; each flood hop adds path bytes (1 per hop) and a retransmission by that hop.

| Radio setting | Reply / alert | Reply with 12 DTCs |
|---------------|---------------|--------------------|
| SF7 BW62.5 | 0.21 s | 0.44 s |
| SF8 BW62.5 | 0.37 s | 0.78 s |
| SF10 BW250 | 0.31 s | 0.64 s |
| SF11 BW250 | 0.58 s | 1.19 s |

Radio settings are set per deployment through MeshCore, so check which row applies. Driving with the
default 5 minute report: 12 reports per hour, plus start and park, about 3 to 7 s of airtime per hour
per opted-in client (0.1 to 0.2 % duty cycle); high-priority alerts (parked, DTC) retry until ACKed,
so a poor link can multiply their share. At about 120 mA transmit current (SX1262 at +22 dBm, from
the datasheet, not measured on this board) 7 s per hour is roughly 0.25 mAh. Receive listening, which
MeshCore does continuously, and the advert interval are unchanged by this project and dominate battery use.

## GPS (#41, #42)

**GNSS (#41)** reuses MeshCore's `MicroNMEALocationProvider` through `EnvironmentSensorManager`: it
parses NMEA, updates `node_lat/lon/altitude` once a second and sets the RTC from GPS time. No
second parser. `-D PERSISTANT_GPS` turns GNSS on at boot when the module is detected (stock default
is off until the admin `gps 1` setting). Pins are MeshCore's heltec_v4 map, not the original plan table:
RX 38, TX 39, reset 42 (active low), enable 34 (active low); see `docs/meshcore.md`. Plan's wake 40 and PPS 41 are not used.

**GPS motion (#42)** is computed by `src/gps/gps_track.{h,cpp}` from successive fixes, sampled at
about 1 Hz: trip distance, speed and heading, in channels 14-16 above. Position (lat, lon, altitude)
stays on MeshCore's channel 1.

- Separate from OBD: its own accumulator (`GpsTrack`), reset when the OBD trip starts. The OBD
  odometer, trip and speed (channels 2-4) never read it and it never replaces them.
- A fix counts as a step only if it is 8 m from the last accepted one, so stationary GNSS wander adds
  nothing; speeds under about 30 km/h are resolved over several seconds. Speed reads 0 after 5 s with no step.
- No fix for 10 s: fields are omitted, the gap is not integrated. A step implying over 300 km/h is a jump and ignored.
- Chord distance between fixes underestimates curves a little; the OBD distance stays the reference.
- Privacy: channels 14-16 are sent only to requesters with location permission and only while GNSS is
  active, the same rule as channel 1. Adverts still never carry live position.
- Compatibility: appended channels only; the existing bytes are unchanged (tested). Worst case with
  everything present is about 138 bytes plus channel 1, within the 180 byte limit.

**OBD vs GPS distance (#43)** is computed at each trip end by `src/gps/trip_compare.{h,cpp}`:
ratio = GPS trip distance / OBD trip distance. The GPS side is frozen at the trip-end edge (it keeps
accumulating afterwards) and is a lower bound (chords cut curves, no-fix gaps are not integrated), so a
result is only valid when the OBD trip is at least 1 km and GPS had a fix for at least 90% of the trip;
otherwise it is "n/a", never zero. Outside 95-105% is flagged SUSPECT. Those thresholds are guesses until
real drives. Logged as one serial line (`# trip end: obd .. m gps .. m cover ..% ratio ..% ok|SUSPECT`, no
coordinates) and kept in RAM as channel 17 until the next trip ends; it is lost on reboot. A persistent
per-trip log waits on #52.

Verified by host tests only (synthetic fixes). Not verified on hardware: a real fix and NMEA parsing,
RTC set from GPS time, pins, how stock clients display channels 14-16.
