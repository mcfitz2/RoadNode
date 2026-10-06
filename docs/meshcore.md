# MeshCore build system and telemetry integration

Findings for issues #1 (framework choice) and #29 (MeshCore telemetry API).
Investigated against `meshcore-dev/MeshCore` at commit `a366955` (2026-09-30), firmware version `v1.17.1`.

## 1. Build system

MeshCore uses **PlatformIO with the Arduino framework** (not ESP-IDF, not Arduino IDE).

| Item | Value |
| --- | --- |
| Build tool | PlatformIO (`pio run -e <env>`), wrapped by `build.sh` |
| Framework | `arduino` |
| ESP32 platform | `platformio/espressif32@6.11.0` (Arduino-ESP32 2.x / ESP-IDF 4.4 based) |
| Radio lib | RadioLib, pinned to a git commit; `RADIOLIB_STATIC_ONLY`, `RADIOLIB_GODMODE` |
| Other libs | rweather/Crypto, adafruit/RTClib, CayenneLPP 1.6.1 |
| Env inheritance | `arduino_base` -> `esp32_base` -> `Heltec_lora32_v4` -> `heltec_v4_oled` -> `env:heltec_v4_sensor` |
| Variant config | `variants/<board>/platformio.ini`, pulled in via `extra_configs = variants/*/platformio.ini` |
| Local overrides | `platformio.local.ini` (listed in MeshCore's `.gitignore`) |
| Role selection | each env picks an app via `build_src_filter` (`+<../examples/simple_sensor>`) |
| Feature flags | `-D` build flags (`ENV_INCLUDE_*`, `ENV_PIN_SDA/SCL`, `PIN_GPS_*`, ...) |
| CI | GitHub Actions: `pr-build-check`, per-role firmware builders, unit tests |

### Target for RoadNode

Heltec V4 is already supported. Relevant existing envs in `variants/heltec_v4/platformio.ini`:

- `heltec_v4_sensor`: **closest starting point** (SensorMesh app, SSD1306 OLED, `ENV_PIN_SDA=3`, `ENV_PIN_SCL=4`)
- `heltec_v4_tft_sensor`: TFT variant of the same
- `Heltec_lora32_v4`: shared base (SX1262 pins, FEM, VEXT, GPS pins, ADC battery)

## 2. Decision (resolves #1)

**Use PlatformIO + Arduino framework on `platformio/espressif32@6.11.0`, matching MeshCore exactly.**

Rationale:
- MeshCore firmware is the base; a different framework means a different MeshCore build.
- ESP32 core 2.x ships the legacy `driver/twai.h` TWAI driver (`twai_driver_install`, `twai_start`, ...), so no ESP-IDF is needed for CAN. Confirmed in section 6 (Arduino-ESP32 2.0.17, `driver/twai.h` present).
- RoadNode-specific code can sit alongside MeshCore as additional source files selected by `build_src_filter`.

Consequence: use the legacy TWAI API (IDF 4.4), not the node-based `esp_twai_*` API from IDF 6.

## 3. How RoadNode plugs in (decided)

**RoadNode is its own repo. MeshCore is a git submodule (e.g. `lib/MeshCore`, upstream `meshcore-dev/MeshCore`) and is never modified.** All RoadNode code lives in this repo and builds on top of MeshCore's classes. Rationale: upstream updates are a submodule bump, and there is no patch set to maintain. Only fork upstream if we ever need to upstream a change.

Rules:
- No edits inside the submodule. If something needs changing, subclass or wrap it, override via build flags, or use `platformio.local.ini`-style config. Last resort: upstream a PR.
- Pin the submodule to a release tag or commit, and bump deliberately.
- RoadNode's own `platformio.ini` defines the `heltec_v4_roadnode` env. It reuses MeshCore's base env config and sources from the submodule.
- Keep `can/`, `obd/`, `vehicle/`, `storage/` free of MeshCore includes. Only `telemetry/` (a `SensorManager` subclass plus a small `main.cpp` modeled on `examples/simple_sensor`) touches MeshCore. This preserves plan §6 and §26.

**Unvalidated: build wiring (spike in #2).** MeshCore's `platformio.ini` uses paths relative to its own project root (`variants/*/platformio.ini`, `-I variants/heltec_v4`, `file://arch/esp32/AsyncElegantOTA`, `build_src_filter` entries like `+<../variants/heltec_v4>`, and the `merge-bin.py` extra script). Including it from a parent project may not resolve these. Candidate approaches to try, in order:
1. `extra_configs = lib/MeshCore/platformio.ini` with `extends = Heltec_lora32_v4`, then fix up paths via `-I lib/MeshCore/...` and `build_src_filter` with `+<../lib/MeshCore/src/...>`.
2. Treat MeshCore as a library (`lib_deps = symlink://lib/MeshCore`; it ships `library.json` and `build_as_lib.py`) and copy only the Heltec V4 `variants/heltec_v4` board/target glue into RoadNode as our own files (copying, not editing; track upstream changes).
3. Fall back to a thin fork carrying only the new env and no code changes, if neither works.

Note `SensorMesh` and the stock `main.cpp` live in `examples/`, which is not part of the library. We will compile them directly from the submodule path via `build_src_filter` where possible; otherwise RoadNode carries its own sensor-node app (a copy with a different `SensorManager`).

## 4. MeshCore telemetry model

- Telemetry is **Cayenne LPP** (`CayenneLPP` lib), a channel/type/value TLV encoding. It is already compact binary; plan §13's "compact binary rather than verbose text" is satisfied by using it.
- Entry point: `SensorManager` (`src/helpers/SensorManager.h`):
  - `virtual bool begin()`
  - `virtual bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry)`
  - `virtual void loop()`
- `SensorMesh` (`examples/simple_sensor/`) is the sensor-node app. On a telemetry request it resets the LPP buffer, adds battery voltage on channel `TELEM_CHANNEL_SELF` (1), calls `sensors.querySensors(...)`, and returns the buffer (`SensorMesh.cpp` ~L179-186).
- `EnvironmentSensorManager` is the stock implementation (I2C env sensors, INA, GPS via `MicroNMEALocationProvider`). It uses per-sensor `query_*` functions that call `lpp.addTemperature(ch, ...)` etc.
- Existing LPP types cover our fields without custom packets:

| RoadNode field | LPP type |
| --- | --- |
| speed | `addGenericSensor` / `addAnalogInput` |
| total distance | `addDistance` / `addEnergy` / `addGenericSensor` (check range and resolution, see below) |
| trip distance | same as above |
| engine rpm | `addFrequency` or `addGenericSensor` |
| device battery voltage | `addVoltage` (already added by SensorMesh) |
| engine running / vehicle active | `addDigitalInput`/`addPresence` |
| GPS (later) | `addGPS` |

- Alerts: `SensorMesh` has `alertIf(condition, Trigger&, priority, text)` with `PERM_RECV_ALERTS_LO/HI` for push messages to subscribed clients. Useful for trip start/end and DTC events (#32, #46).
- Time series: `TimeSeriesData` in `examples/simple_sensor/` provides min/max/avg history.
- `docs/payloads.md` says the telemetry request payload is "sensor- and application-specific" and not defined in `BaseChatMesh`, so telemetry request/response is handled by the `SensorMesh` app code, not by a protocol-level definition.
- Companion apps display LPP telemetry directly.

### Implication for #31 (packet format)

Default plan: **do not invent a packet**. Implement a `RoadNodeSensorManager : SensorManager` that emits LPP. Fall back to a custom binary payload only if LPP lacks the range/resolution (e.g. 6+ digit odometer in miles at 0.01 resolution; LPP distance/generic have limited range, so verify; total mileage may need splitting over channels or a custom field).

## 5. Pin conflicts and corrections to the plan

The plan's pin table (plan §3) conflicts with MeshCore's Heltec V4 config.

| Finding | Detail | Action |
| --- | --- | --- |
| **GPIO3/4 are I2C in MeshCore** | `heltec_v4_sensor` sets `ENV_PIN_SDA=3`, `ENV_PIN_SCL=4`; the TFT variant sets `PIN_BOARD_SDA=4`, `PIN_BOARD_SCL=3` | Don't use the stock `heltec_v4_sensor` env with CAN on 3/4. Either define our own env that does not set `ENV_PIN_*` for these pins, or move CAN. Decide in #4 |
| OLED I2C is GPIO17/18 | `heltec_v4_oled` | Matches plan |
| LoRa pins | NSS=8, SCLK=9, MOSI=10, MISO=11, RST=12, BUSY=13, DIO1=14, plus PA power=7, GC1109/KCT8103L PA control=2/5/46, TX LED=35 | Plan said "GPIO8-14" only; also avoid 2, 5, 7, 35, 46 |
| Plan says GPIO10 is user button | MeshCore: `PIN_USER_BTN=0`; GPIO10 is LoRa MOSI | Fix plan |
| GNSS pins differ from plan | MeshCore: `PIN_GPS_RX=38`, `PIN_GPS_TX=39`, `PIN_GPS_RESET=42` (active LOW), `PIN_GPS_EN=34` (active LOW). Plan reserved 38-42 (wake 40, PPS 41) | Reserve 34 and 38-42; confirm against Heltec schematic |
| Other board pins | `PIN_VEXT_EN=36`, `PIN_ADC_CTRL=37`, `PIN_VBAT_READ=1` | Avoid |
| `ENV_INCLUDE_GPS=1` is already set in base flags | GPS support already in `EnvironmentSensorManager` | Reuse for #41 instead of writing a new NMEA parser |
| Hardware revisions | `heltec_v4`, `heltec_v4_r8` variants exist | Confirm which revision we own before finalizing pins |

Candidate CAN pins to evaluate in #4: any currently unused exposed GPIO (not in the lists above), routed via the GPIO matrix. Needs a check against the V4 pinout and the actual board revision. Alternatively keep GPIO3/4 for CAN and drop the I2C env sensors (we don't need them).

## 6. Build verification

Test build of stock `heltec_v4_sensor` on macOS (PlatformIO CLI): **SUCCESS** in ~6 min (cold, includes toolchain download).

- Platform: Espressif 32 6.11.0, board `heltec_wifi_lora_32 v4` (16 MB flash, 2 MB PSRAM)
- Framework: `framework-arduinoespressif32` 3.20017.241212 (= Arduino-ESP32 2.0.17)
- RAM 2.7% (56 KB of 2 MB), flash 17.9% (1.17 MB of 6.5 MB), so plenty of headroom for CAN/OBD code
- `driver/twai.h` present in the ESP32-S3 SDK include path, so the legacy TWAI driver is available with no framework change
- Build command: `pio run -e heltec_v4_sensor` from the MeshCore checkout

## 7. Other findings relevant to the plan

- **Sleep**: `mesh::MainBoard` has `virtual void sleep(uint32_t secs)`; `ESP32Board` implements it (`src/helpers/ESP32Board.h`). Use as the hook for #37; wake source (CAN RX / ignition) will need custom code.
- **Storage**: ESP32 builds use SPIFFS (`FILESYSTEM`) for prefs and identity, so mileage storage can use NVS (`Preferences`) separately or the same filesystem. Decide in #25.
- **No existing TWAI/CAN code** anywhere in MeshCore (grepped `src/`, `examples/`, `variants/`, `lib/`); no conflict.
- **Debug flags** `MESH_DEBUG` and `MESH_PACKET_LOGGING` are off by default and the companion env warns not to enable them (they corrupt the serial protocol). Our CAN sniffer's serial output (#9) must not run in an env that also uses the USB companion interface.
- **Contribution policy**: larger changes want an issue and a nod from maintainers, and PRs go to the `dev` branch. Matters only if we upstream a vehicle sensor.

## 8. Recommended follow-ups

1. Close #1 and #29 (done).
2. Update affected issues (done).
3. Update #2 (scaffold) to the submodule layout above, including the build-wiring spike.
4. Update #4 and README: resolve GPIO3/4 conflict.
5. Update #31: default to LPP; verify range for mileage.
6. Update #41: reuse `EnvironmentSensorManager` GPS support and the real GNSS pin map.
