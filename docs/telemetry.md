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

Fields with no fresh value are **omitted, never sent as zero**.

## DTCs (channels 8 and 9)

Read-only: modes 03 (stored), 07 (pending) and 0A (permanent), polled every 30 s while the
ECU answers. Each entry on channel 9 is `kind << 16 | raw`, where kind is 1 stored,
2 pending, 3 permanent and raw is the two wire bytes. Decode: top 2 bits of the high byte
pick P/C/B/U, next 2 bits the first digit, then three hex digits (`0x0301` is `P0301`).
Worst case (12 codes) is 29 + 3 + 72 = 104 bytes, plus MeshCore's channel 1, within the 180 limit.
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

Anyone with the admin/guest password can join the ACL and read location, so the default
password must be changed before in-car use (#51). The vehicle id is not in the payload.

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
