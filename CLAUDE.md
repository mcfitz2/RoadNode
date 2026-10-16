# RoadNode: agent rules

OBD-II telemetry node on MeshCore. Background and design principles are in [README.md](README.md),
the full plan is in [plan.md](plan.md), and the integration notes are under [docs/](docs/). This file
lists only the rules that are easy to break, with links to where each one is defined.

## Hard rules

- **Never edit `vendor/MeshCore`.** It is a pinned, unmodified submodule ([README](README.md#layout)).
  To change MeshCore behaviour, use a RoadNode copy that shadows the vendor file:
  `src/node/main.cpp` (from `examples/simple_sensor/main.cpp`) and `src/telemetry/variant/target.{h,cpp}`.
  If you change one of those copies, update its header comment to list the deviation. Run
  `scripts/check_vendor_drift.sh` after any submodule bump (`--update` once the copies are re-diffed).
- **MeshCore boundary.** Only `src/telemetry` and `src/node` may include MeshCore headers. They see the
  `VehicleSnapshot`, never CAN frames or the OBD transport. Enforced by
  [scripts/check_no_raw_can.sh](scripts/check_no_raw_can.sh) (README principle 5).
- **OBD is read-only: modes 01 and 09 only.** `modeAllowed()` in
  [src/obd/obd_manager.cpp](src/obd/obd_manager.cpp) refuses any other mode before sending (README
  principle 7). Never send mode 04. Widening the allowlist (e.g. Mode 22 for fuel, PROJ-681) needs an
  explicit decision from the owner, not a drive-by change.
- **No per-vehicle profiles.** The same generic OBD settings apply to every car (README principle 6,
  PROJ-682). `VEHICLE_ID` is only a label.
- **Location stays private.** Adverts carry 0,0 and `adv_loc` stays at `prefs`. Vehicle LPP channels
  never encode position ([src/telemetry/vehicle_lpp.h](src/telemetry/vehicle_lpp.h),
  [docs/telemetry.md#privacy](docs/telemetry.md#privacy), #51).
- **No secrets in the repo.** `ADMIN_PASSWORD` in `platformio.ini` stays the placeholder. Real
  passwords are set per unit over the admin CLI ([docs/telemetry.md#privacy](docs/telemetry.md#privacy)).
- **CAN pins are GPIO3/4.** Do not set MeshCore's `ENV_PIN_SDA/SCL` or a display in the node env
  ([README](README.md#can-wiring)). The carrier powers the transceiver from VEXT (GPIO36), which is
  claimed in [src/telemetry/roadnode_board.h](src/telemetry/roadnode_board.h).

## Where code goes

- MeshCore-free logic goes in `src/can`, `src/obd`, `src/vehicle` and `src/storage`. Give it host tests
  in `test/test_<module>/` (Unity), using the fakes in `test/support/` (`fake_bus.h`, `fake_kv.h`,
  `fake_store.h`). See `test/test_poller/` for a rig-style example and `test/test_stack/` for an
  end-to-end example against the ECU simulator.
- Standalone tools have their own `setup()`/`loop()`: `src/sniffer`, `src/monitor` and `src/ecusim`
  (RP2040). Each is excluded from the node env in [platformio.ini](platformio.ini).
- Carrier PCB: [hardware/carrier/](hardware/carrier/). Run `circuit.py` (skidl) first, then
  `build_pcb.py` (see its docstring).

## Verification gates

These are the same gates CI runs ([.github/workflows/build.yml](.github/workflows/build.yml)). Run all of them
before you call a change done:

```sh
scripts/check_no_raw_can.sh
scripts/check_vendor_drift.sh
pio test -d host
pio run -e heltec_v4_roadnode -e heltec_v4_cansniffer -e heltec_v4_canloopback -e heltec_v4_obdmonitor -e rp2040_ecusim
```

For a bug fix, add a test that fails without the fix.

## Hardware testing

- Bench: the RP2040 ECU simulator and its serial scenarios are described in [docs/ecusim.md](docs/ecusim.md).
  Add a scenario there when a test needs one.
- Car and carrier tests wait for the carrier PCB. Put hardware-only acceptance criteria on the
  carrier testing ticket (PROJ-684) instead of blocking code tickets on them. The car procedure is in
  [docs/car-test.md](docs/car-test.md).

## Style

- 2-space indent, `roadnode::<area>` namespaces, `_member` fields, `s_` file-local statics.
- Comments explain *why* (constraints, measurements, ticket numbers), not what.
- Admin/serial command replies start with `OK ...` or `Err: ...`.
- Keep `#ifdef ARDUINO` / host splits as in [src/vehicle/vehicle_state.h](src/vehicle/vehicle_state.h).
