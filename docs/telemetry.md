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

Fields with no fresh value are **omitted, never sent as zero**.

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
| `Vehicle moving at <lat>,<lon>` | low (one attempt) | engine running and speed >= 5 km/h, every 15 min |

The periodic report first fires one interval after engine start, then every interval
while moving (set `-D PERIODIC_ALERT_MINUTES=n`). If due while stopped it waits for the
next moving check. MeshCore cancels a queued alert when its condition drops, so the
condition is held true for a 5 minute window to cover send and retries, then released
so it can re-arm. Checks run on MeshCore's 60 s sensor read.

Each fires once per false->true edge; it is not a position stream. Logic:
`src/vehicle/alert_logic.h`; wiring: `src/node/main.cpp` (copy of the vendor
`simple_sensor/main.cpp`, re-diff on MeshCore bumps). No theft/movement-while-off
alert yet. Delivery is not durable: with no path it floods and gives up after retries.

## Verification status

- Encode/decode round trips and size are unit tested on the host against a model of the
  CayenneLPP wire rules, not the real library.
- Firmware build instantiates the encoder with the real `CayenneLPP`, so it compiles
  against the real API, but real wire bytes have not been compared on a device or in a MeshCore client.
