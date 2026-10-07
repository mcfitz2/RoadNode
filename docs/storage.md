# Mileage persistence

Implements plan section 12 (#25). Code: `src/storage/`.

## Design

- **Record** (`VehicleRecord`): total distance (mm), trip distance (mm), last trip timestamp, short vehicle ID. Encoded to a fixed 52-byte little-endian blob: magic, version, sequence, fields, CRC-32.
- **A/B slots.** `VehicleStorage::save()` always writes to the slot that does *not* hold the newest valid record. A power loss mid-write can only damage the older copy. `load()` returns the valid record with the highest sequence number (wrap-safe compare). A record with a bad magic, version, length or CRC is ignored; if both slots are invalid the device is treated as fresh.
- **Backend.** `SlotStore` interface; `NvsSlotStore` (ESP32 `Preferences`, namespace `roadnode`, keys `slot0`/`slot1`) on the device, an in-memory fake in host tests.
- **Checkpoint policy** (`CheckpointPolicy`, pure logic). Mileage is accumulated in RAM; a checkpoint is due when the distance has changed since the last save **and** either at least 0.5 mile has been covered or 2 minutes have passed. Nothing is due while parked. The caller must also checkpoint on trip end and before any intentional shutdown/sleep, then call `markSaved()`. Both limits are configurable (`CheckpointConfig`).
- No flash write per speed sample: the policy is the only thing that triggers a save.

## Verification (host tests, `pio test -d host`)

- Round trip, alternation of slots, newest-wins, sequence continuity across reboot.
- **Power loss:** a torn write is injected at every byte offset (0-51) of the third save; the second record always loads intact.
- Corrupt newest record falls back to the older one; both corrupt means fresh; bad magic rejected; sequence wraparound; failed save can be retried.
- Policy: distance trigger, time trigger, no trigger while parked, reset on save, clock wraparound.

Not verified on hardware: `NvsSlotStore` compiles in the `heltec_v4_roadnode` env but has not run on a device, and nothing calls it yet (wiring is #26).

## Wear estimate

**Estimate, not measured.** Assumptions:

- NVS partition on this board: `default_16MB.csv`, 20 KB = 5 flash sectors (4 KB), wear-leveled by NVS.
- A 52-byte blob uses 3 NVS entries (1 header + 2 data, 32 bytes each) = 96 bytes. A 4 KB page holds 126 entries, so about 42 writes fill a page, after which NVS erases one. Updating a key marks the old entries erased and appends new ones.
- Upper-bound writes per year for a heavy user: 12,000 miles at 2 checkpoints/mile = 24,000; time-based checkpoints while driving slowly add at most ~12,000 (400 h at 30/h); trip ends ~1,500. Total about 40,000 writes/year.
- 40,000 / 42 = about 950 page erases/year, spread over 5 pages = about 190 erases per page per year.
- ESP32 flash is typically rated for 100,000 erase cycles, so roughly 500 years. Even if the real figure is 10x worse, wear is not a concern at these intervals.

Re-check if the intervals are shortened substantially.

## Open items

- Wire into the mileage engine and trip events (checkpoint on `TripEvent::Ended`, before sleep): #26.
- Store/restore `last_trip_timestamp` needs a time source (RTC or GPS time): until then it is 0.
- Vehicle ID is stored here but the VIN-based identity flow is #18/#34.

## Vehicle identity and VIN (#18, #34)

Stored in NVS namespace `roadnode_cfg` (`NvsKvStore`), separate from the mileage slots.

- `vid`: short vehicle ID (1-15 of `A-Z 0-9 - _`, stored uppercase). The persisted value wins over the
  `VEHICLE_ID` build flag, which is only the first-boot default. This is the only identifier in telemetry
  and alerts. Admin commands: `vid`, `vid set <ID>`.
- `vin`: read once per boot from Mode 09 PID 02 while stopped (retried up to 5 times, 30 s apart; a
  negative response stops retries). The first well-formed VIN is stored; a failing check digit is
  recorded but not rejected (check digit is only mandatory in North America). A later different VIN
  does not overwrite: it sets a mismatch flag. `vin clear` forgets it for a new vehicle.
- The VIN never enters `VehicleSnapshot`, LPP telemetry, alerts or command replies (`vin` only reports
  stored / check digit / mismatch). Admin command replies travel over the mesh, so they stay VIN-free.

Unverified on hardware: NVS persistence across power cycles, real ECU VIN reads.

## Checkpoint on reboot / power off, and the trip timestamp (#54, #55)

- **Reboot/power off (#54).** `RoadNodeBoard` (`src/telemetry/roadnode_board.h`, used as `board` in the shadowed
  `target.{h,cpp}`) overrides MeshCore's `reboot()` and `powerOff()`, so the admin `reboot`, `poweroff`, `shutdown`
  and `clkreboot` commands call `VehicleRuntime::shutdown()` first. That waits up to 2 s for a poll step in progress
  (mutex shared with the poll task) and skips the save if it cannot get it, rather than write from two tasks.
  Not covered: OTA (the restart happens inside the update handler) and a power cut, which still lose up to
  0.5 mile / 2 minutes. Trip end already checkpoints, so normal parking is unaffected.
- **Timestamp (#55).** `last_trip_timestamp` is set from the MeshCore RTC at trip end, but only once a GNSS fix has been
  seen this boot (the RTC is then GPS-set). A stock RTC with no sync holds an arbitrary value, so before a fix the
  timestamp stays as it was. An admin `clock sync` alone does not enable it.

Verified: the tracker's use of the timestamp (host tests). Not verified: the board overrides running on the ESP32,
and the GPS-set RTC (no fix on the bench).
