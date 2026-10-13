# Car test checklist

First in-vehicle test of the prototype (Heltec V4.3 + SN65HVD230 breakout + GT-U7 GPS),
powered from the laptop's USB, not OBD pin 16. Everything here was already verified on
the bench against the Feather ECU simulator (`docs/ecusim.md`); this run covers what the
bench cannot: a GPS fix, a real ECU, and a scan tool to compare against.

Do not write VINs, serial numbers, keys, channel secrets or position fixes into issues or
the repo.

## Laptop setup

1. `git pull`, then `pio test -d host` (all pass).
2. Python with pyserial for scripted serial access (`pip install pyserial`), and `gh auth login`.
3. Plug in the Heltec, find its port with `pio device list`.
4. Flash: `pio run -e heltec_v4_roadnode -t upload --upload-port <port>`. Settings in NVS
   (odometer, alert period, alert channel) survive a reflash.
5. Serial CLI at 115200: send `<command>\r\n`, the reply line starts with `  -> `.

## Wiring

Per the README CAN wiring table: CANH to OBD pin 6, CANL to pin 14, GND to pin 4 or 5,
nothing on pin 16.

The car bus is already terminated. Before plugging in, check the breakout for an onboard
120 ohm resistor (meter across CANH/CANL on the module alone reads ~120 ohm). If present,
remove or disconnect it for the car. With the module on the OBD port and the car off,
CANH-CANL should read ~60 ohm.

GPS antenna on the dash or roof with a sky view.

## Checks

| # | Issue | Do | Pass |
|---|-------|----|------|
| 1 | #41 | `sensor list` shows gps, then `gps` | `on, active, fix, N sats` within a few minutes outdoors |
| 2 | #41 | `clock` after the fix | correct UTC time (GPS sets the RTC) |
| 3 | - | ignition on, engine running | channel alert `Vehicle started at <lat>,<lon>` |
| 4 | #17 | engine idling, request telemetry in the app; read the same PIDs on the scan tool | RPM (ch 5), coolant (ch 10), load (ch 11), intake air (ch 12), fuel level (ch 13) agree with the scan tool |
| 5 | #44 | compare DTCs: app ch 8 count and ch 9 entries, `Vehicle DTC` channel alert if a code appears, vs. scan tool Mode 03 | same codes. With no stored codes, ch 8 = 0 and no ch 9 entries. Never clear codes from RoadNode (it has no Mode 04) |
| 6 | - | drive a few minutes | `Vehicle moving` alert every 5 min (`alert periodic`); speed ch 4 matches the speedometer within a few km/h |
| 7 | #60 | note `odo` before and after the drive; compare with the trip meter | difference matches within ~2 % (`odo scale` adjusts) |
| 8 | #43 | after parking, request telemetry | ch 17 (last trip GPS/OBD distance %) near 100 |
| 9 | - | park, engine off, wait ~2-3 min | `Vehicle parked at <lat>,<lon>` alert |
| 10 | #55 | trip end with GPS time set | trip-end timestamp stored. No command shows it yet; host-tested only, add a readout before claiming it on device |
| 11 | #60 | GPS fill: needs OBD speed unavailable while moving (e.g. unplug CAN briefly while a passenger drives) | `odo` reports `gps filled` > 0 and no double count. Only if it can be done safely; otherwise leave for later |

## Privacy while testing

Position appears only in the alert texts sent to the private channel and ACL clients.
Adverts stay at 0,0: `gps advert` (show) should reply `> none`. Never run `gps advert share`,
`gps advert prefs` or `gps setloc`; they put the position in public adverts.

## Afterwards

Post results per issue (text only, no positions or identifiers) and close only issues whose
criteria are actually met. Unplug the node when parked: pin 16 power and sleep drain are
not tested yet.
