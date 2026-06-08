# u-blox HPE Driver Design Notes

## Current Goal

`ubx_hpe` starts as a PX4-specific passive recorder for u-blox HPE receivers such as ZED-X20P and ZED-F9P. The first revision intentionally avoids receiver configuration and message parsing. It records raw receiver bytes so we can inspect real traffic, validate firmware behavior, and add parsing/configuration from evidence rather than guesses.

## Current Shape

The driver is a `ModuleBase` task with CLI lifecycle commands (`start`, `stop`, `status`). It reads raw bytes from one UART and sends those bytes to two optional sinks:

- A `.ubx` file on SD card when `UBX_HPE_FILE=1`. The default filename mode is incrementing, such as `/fs/microsd/ubx_hpe_000.ubx`, to avoid overwriting earlier bench captures.
- PX4's existing `gps_dump` uORB topic when `UBX_HPE_DUMP=1`.

`gps_dump` is reused instead of adding a new uORB topic because PX4 already uses it for raw GNSS communication dumps. Septentrio's GNSS driver is the closest precedent for this behavior.

## Why A Dedicated Module Task First

A dedicated task is the safer initial design for raw GNSS capture because this driver may block or spend variable time in operations that should not run on a shared work queue:

- Waiting for serial bytes.
- Coalescing high-rate UART data.
- Writing to SD card.
- Future parsing or capture bookkeeping that may become bursty.

PX4 work queues are best for short, bounded `Run()` callbacks. Blocking serial reads, sleeps, long parsing, and filesystem writes can delay other drivers sharing the same queue. For this initial recorder, isolation matters more than minimizing task overhead.

A future `ModuleBase + ScheduledWorkItem` design could make sense if the driver becomes a small nonblocking parser with no direct SD-card writes.

## Serial I/O

The driver uses `device::Serial` from `px4_platform_common/Serial.hpp` instead of opening and configuring the UART directly.

Reasons:

- It matches the GPS, Septentrio, CRSF, and other modern PX4 serial-driver patterns.
- It handles NuttX, POSIX, and QURT platform differences in one place.
- It opens the port nonblocking and configures raw 8N1/no-flow-control behavior consistently.
- It supports `readAtLeast()`, which coalesces UART bytes and avoids waking for every tiny fragment.
- It avoids local baud-rate constant portability issues.

Current read pattern:

```cpp
device::Serial _uart;

_uart.setPort(_device);
_uart.setBaudrate(_baudrate);
_uart.open();
_uart.flush();

const ssize_t nread = _uart.readAtLeast(buffer, sizeof(buffer), min_read_bytes, timeout_ms);
```

For a raw recorder, a reasonable starting point is `min_read_bytes = 32` and `timeout_ms = 50`, mirroring the GNSS driver convention. These values should be adjusted after looking at real X20P/F9P output rates and SD write behavior.

## PX4 Serial Port Configuration

`module.yaml` defines `UBX_HPE_CFG` as the PX4 serial-port assignment parameter. The generated startup command passes the selected device and baud parameter to the module:

```sh
ubx_hpe start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
```

The `-b` option accepts both literal baud rates, such as `230400`, and PX4 parameter references, such as `p:${BAUD_PARAM}`.

The receiver-side u-blox port selection (`UART1` vs `UART2`) is defined as a parameter now, but it has no effect while the driver is passive-only. The first receiver auto-configuration patch should use it to select the appropriate UBX `CFG_UART1_*` or `CFG_UART2_*` keys.

Auto-configuration parameters are already defined for the future config implementation:

- `UBX_HPE_PORT`: receiver-side u-blox UART port, `UART1` or `UART2`.
- `UBX_HPE_DUMP`: enable publishing raw bytes to `gps_dump`.
- `UBX_HPE_FILE`: enable direct SD-card `.ubx` capture.
- `UBX_HPE_FNAME`: choose fixed, incrementing, or realtime-based `.ubx` filenames.
- `UBX_HPE_DUMP_I`: `gps_dump.instance` value used for raw byte publication.
- `UBX_HPE_RATE`: receiver measurement period in milliseconds.
- `UBX_HPE_RAW`: raw measurement message divisor.
- `UBX_HPE_SOL`: solution message divisor.
- `UBX_HPE_STAT`: status, timing, signal, correction-state, and RF-health message divisor.
- `UBX_HPE_SPAN`: `MON-SPAN` spectrum monitor divisor.
- `UBX_HPE_SFRBX`: `RXM-SFRBX` navigation-bit message divisor.

These parameters are not applied until receiver auto-configuration sends UBX `CFG-VALSET` messages.

## Receiver Auto-Configuration Direction

When auto-configuration is added, start with UBX-only receiver I/O. NMEA should be explicitly disabled because this driver does not need it and mixed ASCII/binary output makes raw capture and parser bring-up noisier than necessary.

Initial protocol policy:

- Enable UBX input/output on the selected receiver UART.
- Disable NMEA input/output on the selected receiver UART.
- Leave RTCM disabled until there is a concrete moving-base, correction-input, or correction-output requirement.
- Make RTCM a deliberate later option, not part of the default bring-up profile.

Rate configuration is grouped by message purpose instead of one flat output-rate knob:

- Raw measurements: high-rate raw observables needed for HPE/post-processing workflows.
- Solution messages: PVT and relative-position solution messages at the primary navigation/estimation rate.
- Status/health messages: lower-rate monitoring such as RF, spectrum/span, signal, jamming/spoofing, satellite, and receiver health/status messages.

This gives us a clean way to tune bandwidth: raw and solution traffic can run fast, while diagnostic traffic stays informative without crowding the UART or SD-card recorder. The first implementation should define these groups in driver code as typed UBX `CFG-VALSET` recipes, using the `UBX_HPE_*` parameters above to select the receiver port, base measurement period, and message divisors.

## gps_dump Notes

`gps_dump` messages hold up to 79 bytes. Incoming receiver bytes should be chunked into that payload size and published with:

- `instance = UBX_HPE_DUMP_I`, defaulting to `2` so it can coexist with conventional main/secondary GNSS dump instances `0` and `1`.
- `device_id = 0` until the driver has a real PX4 device ID.
- The high bit of `len` clear for receiver-to-PX4 bytes.

If the driver later writes configuration to the receiver and dumps outgoing bytes too, follow the Septentrio convention and set the high bit of `len` for PX4-to-receiver bytes.

## File Logging Notes

The SD-card sink is useful for early bring-up, but it is the highest-risk part of the driver from a timing perspective. Things to check before relying on it in flight:

- Whether direct `write()` calls can stall long enough to lose UART bytes.
- Whether the current periodic `fsync()` interval is appropriate for the target SD card and data rate.
- Filename mode is parameterized now: fixed overwrites `/fs/microsd/ubx_hpe.ubx`, incrementing chooses the first free `/fs/microsd/ubx_hpe_NNN.ubx`, and realtime uses `CLOCK_REALTIME` when valid with incrementing fallback.
- Whether logging should stop gracefully when the SD card is missing or full.
- Whether raw capture belongs in PX4 logger/ULog instead of direct files once the topic design is stable.

## Monitoring And Perf

The baseline recorder includes lightweight runtime observability:

- Perf counters for cycle, UART read, file write, fsync, read errors, write errors, partial writes, and fsync errors.
- Average read/write byte rates and `gps_dump` message rate in `status`.
- Max read chunk, max write chunk, max write latency, and max fsync latency.
- Periodic fsync for dirty log files instead of fsyncing every write.
- Stale-data warnings if no receiver bytes arrive for more than five seconds.
- `status` reports whether the UART is open, whether file logging is enabled, and which `gps_dump` instance is used.

This should make the first hardware captures easier to debug without committing to parser/config behavior yet.

## Next Things To Check

1. Rebuild the intended hardware target, not only SITL.
2. Test capture on the target UART with an X20P/F9P receiver at the expected baud rates.
3. Confirm whether `/fs/microsd/ubx_hpe.ubx` write bandwidth is enough at the receiver's highest configured output rate.
4. Tune `min_read_bytes` and `timeout_ms` using real receiver output and missed-byte observations.
5. Watch write/fsync latency on hardware captures and adjust fsync cadence if needed.
6. Check generated parameter metadata for `UBX_HPE_CFG` and verify startup on the actual board.
7. Validate `UBX_HPE_FILE`, `UBX_HPE_DUMP`, and `UBX_HPE_FNAME` behavior on real FMU boot, especially whether realtime is valid early enough to be useful.
8. Add a real PX4 device ID if/when this should appear as a normal GNSS device source.
9. Capture sample raw UBX/HPE traffic before adding parser logic.
10. Add UBX frame synchronization and checksum validation as the first parsing layer.
11. Add optional receiver configuration only after the passive capture path is proven.
12. Start receiver configuration as UBX-only, with NMEA disabled and RTCM deferred until needed.
13. Add grouped rate parameters for raw measurements, PVT/relative-position solutions, and lower-rate status/health messages.
14. Add a receiver-side UBX port parameter (`UART1` or `UART2`) when auto-configuration needs it.
15. Decide whether this should coexist with the generic PX4 GPS driver on a separate UART or replace it for HPE-specific missions.
16. Check boot/startup integration and board config for the actual x20p/F9P vehicle target.

## References In The PX4 Tree

Useful examples to compare against:

- `src/drivers/gps/gps.cpp`: generic GPS serial read loop using `device::Serial::readAtLeast()`.
- `src/drivers/gnss/septentrio/septentrio.cpp`: GNSS driver with `gps_dump` support for incoming and outgoing receiver bytes.
- `platforms/common/include/px4_platform_common/Serial.hpp`: platform-independent serial abstraction.
- `platforms/nuttx/src/px4/common/SerialImpl.cpp`: NuttX serial implementation.
- `platforms/posix/src/px4/common/SerialImpl.cpp`: POSIX serial implementation.

## Upstream/Fresh Fork Porting Note

The first `ubx_hpe` baseline was developed in a private PX4 fork that may lag current upstream PX4 and ARK board support. Treat the baseline recorder commit as a portable checkpoint, not necessarily the long-term integration base.

Before adding DroneCAN provisioning, receiver auto-configuration, or new uORB messages, re-apply the baseline driver on a current upstream PX4 or actively maintained ARK fork. The current baseline should be easy to move because it is mostly a new driver directory plus one board config enable line:

```sh
git cherry-pick a8788fa08d
```

Keeping the DroneCAN/config work on an up-to-date base should reduce churn in these likely conflict areas:

- DroneCAN/UAVCAN APIs and node-firmware conventions.
- Board Kconfig and generated parameter metadata.
- Serial-driver startup scripts and `module.yaml` behavior.
- uORB message definitions and logger topic selection.
- Existing GNSS parsing/config code that may have changed upstream.

## UART1-Captive DroneCAN Provisioning Case

A common vehicle integration may wire receiver UART1 to a DroneCAN/DroneCAN-adjacent MCU, leaving receiver UART2 as the only port available to PX4. This is important for ZED-X20P-class receivers because UART1 is the reliable factory-default UBX control port, while UART2 may default to correction-oriented protocols and may not accept UBX configuration commands until it has been provisioned.

Recommended split:

- PX4 `ubx_hpe` remains passive-first on UART2 and records whatever is present.
- PX4 detects stream type from incoming bytes where possible: UBX sync, RTCM3 preamble, NMEA `$`, SPARTN-like traffic, or unknown.
- If UART2 does not respond to UBX probes, PX4 should report that UART2 UBX input may not be enabled rather than repeatedly sending blind config.
- A DroneCAN MCU connected to receiver UART1 should provide a factory-provisioning path for UART2.
- After UART2 is provisioned for UBX input/output, PX4 can own normal operation over UART2 and apply later `CFG-VALSET` updates directly.

Minimum DroneCAN-side provisioning recipe:

- Probe UART1 at expected baud rates and confirm receiver identity with `UBX-MON-VER`.
- Configure selected receiver UART2 baud for the PX4 link.
- Enable `CFG_UART2INPROT_UBX` and `CFG_UART2OUTPROT_UBX`.
- Disable NMEA on UART2.
- Decide deliberately whether RTCM/SPARTN remain enabled on UART2; do not leave them on by accident if bandwidth is tight.
- Configure the desired UBX message groups and rates on UART2.
- Save to nonvolatile receiver config only when the integration wants persistent UART2 behavior.

This should become an explicit driver state later: `passive`, `probe`, `configure-local-port`, and `requires-external-provisioning`.

## Constellation, Band, And Signal-Health Parameters

Constellation and signal-band selection should be parameters once auto-configuration exists. They directly affect bandwidth, solver behavior, and spoof/jam diagnostics, so they should not be hidden constants.

Recommended parameter groups:

- `UBX_HPE_CONST`: constellation enable mask for GPS, Galileo, BeiDou, GLONASS, QZSS, and SBAS as supported by the receiver/firmware. This parameter is now defined in `module.yaml` but not applied until auto-configuration exists.
- `UBX_HPE_BANDS`: signal-band enable mask, starting with L1/E1/B1/C1 and L5/E5/B2-class signals. This parameter is now defined in `module.yaml`; extra band bits are placeholders until mapped to concrete u-blox `CFG-SIGNAL` keys.
- `UBX_HPE_L5_HLT`: L5 health handling policy. This parameter is now defined in `module.yaml` but not applied by the passive recorder.

The L5 health policy is useful because some receivers, firmware revisions, constellations, or test environments can mark L5-family signals unhealthy or partially available even when those measurements are useful for diagnostics/post-processing. Suggested policy values:

- `0`: strict, trust receiver signal health and do not use unhealthy signals in derived topics.
- `1`: ignore L5 health for raw capture/parsing summaries only, but keep health flags published so downstream code can choose.
- `2`: ignore L5 health for driver-level measurement selection where supported.

Default should be strict (`0`) until we have receiver logs showing that a relaxed policy is needed. Even when ignoring L5 health for selection, publish the original health/quality flags so post-flight analysis can see what the receiver reported.

## Message Parsing And uORB Publishing Plan

Add parsing in layers so each step is testable against captured `.ubx` files:

1. UBX frame synchronization and checksum validation.
2. Raw class/id counters and per-message drop/error stats.
3. Timestamp extraction and receiver time mapping.
4. Typed parsing for solution messages: `NAV-PVT`, `NAV-RELPOSNED`, and any HPE-specific solution/status messages.
5. Typed parsing for raw measurements: `RXM-RAWX` and optionally `RXM-SFRBX`.
6. Typed parsing for signal/status/health: `NAV-SIG`, `NAV-SAT`, `MON-RF`, `MON-SPAN`, `MON-COMMS`, `MON-SYS`, `RXM-COR`.
7. uORB publication and logger integration.
8. Receiver auto-configuration using the same message definitions and rate groups.

Potential existing uORB mappings:

- `sensor_gnss_relative`: good fit for `NAV-RELPOSNED`-style relative position output.
- `sensor_gps` / `vehicle_gps_position` path: possible fit for PVT, but using the generic GPS pipeline may require more integration than a first typed HPE status publisher.
- `sensor_gnss_observables`: partial fit for raw observations, but it is capped at 20 observations and may not represent multi-band/multi-constellation X20P output without loss.
- `satellite_info`: partial fit for `NAV-SAT`, but it is capped at 20 satellites and has limited signal/band detail.
- `gps_dump`: best fit for raw UBX payload retention and full-fidelity logging when typed topics are lossy.

If full multi-band raw observables matter inside PX4, add a new bounded uORB topic rather than overloading `sensor_gnss_observables`. Candidate shape:

- One message per epoch chunk, not necessarily one message per epoch.
- Fixed maximum observation count per message.
- Fields for constellation, signal id/band, svid, pseudorange, carrier phase, doppler, C/N0, lock time, standard deviations, tracking status, health/quality flags, and chunk sequencing.

## MON-SPAN, NAV-SIG, And uORB Constraints

uORB messages are fixed-size C structs with bounded arrays. They do not support arbitrary-length UBX payloads as native variable-length messages. This affects large or variable-entry GNSS messages:

- `NAV-SIG` can fit uORB if represented as a bounded, possibly chunked signal-summary topic. It should include signal count, constellation, signal id, svid, C/N0, quality indicators, health flags, correction/lock flags, and chunk sequence fields. A hard cap is required, and overflow/drop counters should be published or reported.
- `NAV-SAT` can fit similarly, but existing `satellite_info` is probably too small and too generic for full HPE signal analysis.
- `MON-RF` is small enough for a typed RF-health topic.
- `RXM-COR`, `MON-COMMS`, and `MON-SYS` are good candidates for compact typed status topics or aggregated diagnostics.
- `MON-SPAN` is the awkward one. Full spectrum data is comparatively large and naturally array-like, so a single typed uORB message may either be too large or too lossy. Prefer one of these approaches:
  - Keep full `MON-SPAN` payloads in `.ubx`/`gps_dump` and publish only a compact summary topic.
  - Publish bounded chunks with frequency-bin offset, bin count, amplitude bins, span id, and sequence fields.
  - Defer typed `MON-SPAN` publishing until we know the exact payload size/rate from X20P logs.

For first parser bring-up, publish small typed summaries and keep raw `.ubx` capture enabled. That preserves full evidence while avoiding large uORB messages that increase RAM, logger bandwidth, and review risk.

## UBX Tunnel To DDS/Zenoh For Offboard Analysis

Onboard uORB topics should stay small enough for real-time use: signal-quality summaries, jammer/spoofer indicators, RF health, correction state, and bounded per-signal/per-satellite summaries. Full-fidelity UBX payloads for messages such as `MON-SPAN`, dense `NAV-SIG`, raw measurements, and future HPE diagnostics can be too large or too high-rate for convenient onboard typed topics.

A better split is:

- PX4 publishes compact typed uORB summaries for onboard monitoring, failsafe logic, and logs.
- PX4 optionally publishes a chunked UBX tunnel topic for offboard analysis by a larger computer over DDS/Zenoh.
- SD `.ubx` and `gps_dump` remain the local forensic capture path.

This should be a dedicated UBX/HPE tunnel topic rather than overloading solution/status topics. It can still use the normal uORB-to-Zenoh or uORB-to-DDS bridge once the topic is included in the generated bridge configuration.

Candidate topic shape:

- `timestamp`: PX4 publication time.
- `timestamp_sample`: receiver/sample time if known, otherwise zero.
- `device_id` and `instance`.
- `stream_id`: distinguish receiver input, outgoing config, parsed UBX frames, or selected message classes.
- `ubx_class` and `ubx_id` when the chunk is associated with a decoded UBX frame.
- `sequence`: monotonically increasing stream sequence for loss detection.
- `frame_sequence`: UBX-frame sequence for reassembling multi-chunk frames.
- `chunk_index` and `chunk_count`.
- `payload_length`.
- fixed-size `payload[N]` bytes.
- flags for direction, checksum-valid, truncated, parser-resync, and rate-limited/drop state.

The payload size should be chosen with bridge and logger cost in mind. Existing PX4 precedents are `mavlink_tunnel` with 128 bytes and `ulog_stream` with 249 bytes. A UBX tunnel could reasonably start in the 128-256 byte range, but should be tested on the target bridge. Larger chunks reduce per-message overhead but increase uORB/log/bridge memory pressure.

Do not assume DDS/Zenoh can carry arbitrary variable-length UBX packets directly from uORB. PX4 uORB messages are fixed-size C structs with bounded arrays, and the bridge generated topics follow that constraint. Large UBX messages must be chunked or summarized.

Potential modes:

- `off`: no tunnel publication.
- `selected`: tunnel only selected large/diagnostic messages such as `MON-SPAN`, `NAV-SIG`, `RXM-RAWX`, or unknown HPE messages.
- `all_valid_ubx`: tunnel every checksum-valid UBX frame after parser sync.
- `raw_bytes`: tunnel raw UART bytes in chunks before parsing, mainly for debugging bridge transport.

Recommended default is `off` or `selected`, with SD `.ubx` capture still available. The tunnel is for live offboard analysis, not a replacement for robust local capture until bridge bandwidth, loss behavior, and CPU cost are measured.

This also gives a clean analysis pipeline:

1. `ubx_hpe` parses and summarizes enough for onboard health.
2. `ubx_hpe` emits selected full UBX frames as chunked uORB tunnel messages.
3. Zenoh/DDS bridge forwards the tunnel stream to a companion or ground computer.
4. Offboard tools reassemble frames and run heavier spectrum, signal-quality, spoof/jam, and raw-measurement analysis.

## First Spoof/Jam And False-Time Feature Set

The first compact parser/status implementation should prioritize these UBX messages:

- `MON-RF`: receiver RF/jamming/interference state and RF block health.
- `NAV-SIG`: per-signal tracking, health, quality, C/N0, and used/not-used state.
- `MON-SPAN`: spectrum summary for narrowband/wideband interference detection.
- `NAV-CLOCK`: receiver clock bias, drift, and timing-quality anomalies.

This priority is driven by real-world false solution time observations. Position plausibility alone is not enough; the driver should surface time-quality features early so downstream logic can distinguish a position-only degradation from a time/clock integrity problem.

Recommended compact clock/time fields:

- Receiver time-of-week and UTC-valid state where available.
- `NAV-CLOCK` clock bias and drift.
- Clock accuracy estimate.
- Time accuracy estimate from PVT/time messages when available.
- Delta between consecutive receiver time samples and PX4 monotonic elapsed time.
- Count of backward receiver-time jumps.
- Count of large forward receiver-time jumps.
- Count of UTC validity changes.
- Count of leap-second/UTC parameter changes if exposed.
- `false_time_suspect` or `time_integrity_score` derived from jumps, drift, invalid UTC, and mismatch to PX4 monotonic time.

Suggested first summary topics/features:

- `ubx_hpe_signal_summary`: `NAV-SIG` counts, C/N0 distributions, constellation/band masks, used ratios, strong-not-used counts, unhealthy counts.
- `ubx_hpe_rf_summary`: `MON-RF` status plus `MON-SPAN` compact floor/peak/occupied-bandwidth features.
- `ubx_hpe_time_summary`: `NAV-CLOCK` and receiver-time consistency metrics.

For the first implementation, publish raw values and simple counters before inventing a hard classifier. The classifier can initially live offboard using the UBX tunnel and logs, then move into PX4 once the false-time and RF signatures are understood.

## Full-Fidelity Message Tunnel Selection

The first full-fidelity UBX tunnel should prioritize messages that are too large, too variable, or too analysis-heavy for normal onboard uORB topics:

- `NAV-SIG`: dense per-signal quality, health, correction, and tracking state.
- `MON-SPAN`: spectrum bins and RF-interference detail.
- `RXM-SFRBX`: raw navigation subframes/bits.
- `RXM-RAWX`: raw pseudorange, carrier phase, Doppler, C/N0, and measurement quality.

These messages should not be forced into normal typed uORB summaries. They either exceed existing message caps, require chunking, or are primarily useful to offboard analysis/post-processing. The onboard driver should parse enough to validate frames, maintain counters, publish compact summaries, and decide which frames to tunnel.

Recommended split:

- `gps_dump` and `.ubx` file: local raw forensic capture.
- UBX tunnel over DDS/Zenoh: live offboard analysis for large diagnostic/raw messages.
- Compact uORB summaries: onboard health, warning, and logging signals derived from selected fields.

For the first tunnel implementation, use a `selected` mode that forwards checksum-valid `NAV-SIG`, `MON-SPAN`, `RXM-SFRBX`, and `RXM-RAWX` frames. Add counters for tunneled frames, dropped frames, chunk count, bytes tunneled, and tunnel rate limiting. Avoid enabling `all_valid_ubx` by default until bridge bandwidth and companion CPU cost are measured.

## UART vs DroneCAN Driver Placement Tradeoff

There are two plausible places to run the HPE-specific UBX logic:

- On PX4 over a directly connected UART.
- On a DroneCAN node that has direct access to the receiver UARTs.

The direct-UART PX4 driver is simplest when PX4 can reach a receiver port that already has UBX enabled. It gives PX4 direct access to raw bytes, direct `.ubx` capture, direct `gps_dump`, and eventually direct receiver configuration. It is less attractive when receiver UART1 is captive behind another MCU and UART2 may not have UBX input enabled from factory defaults.

The DroneCAN-node driver is attractive when a node MCU is already wired to receiver UART1 and can provision UART2 or own all UBX communication. In that architecture, the node becomes the receiver-specific UBX/config/parser endpoint, and PX4 receives products over CAN.

### Existing PX4 DroneCAN GNSS Path

The current PX4 `uavcan` GNSS bridge already receives common DroneCAN GNSS products:

- `uavcan.equipment.gnss.Fix` / `Fix2` -> PX4 `sensor_gps`.
- `uavcan.equipment.gnss.Auxiliary` -> HDOP/VDOP support for `sensor_gps`.
- `ardupilot.gnss.RelPosHeading` -> PX4 `sensor_gnss_relative`.
- RTCM and moving-baseline data through existing GNSS bridge paths where enabled.

If the DroneCAN node runs the UBX driver and publishes only these standard messages, PX4 can still navigate normally, but it will not receive rich HPE diagnostics such as `NAV-SIG`, `MON-SPAN`, `RXM-RAWX`, `RXM-SFRBX`, or false-time/clock-integrity details unless additional messages are added.

### Benefits Of Running UBX On The DroneCAN Node

- Solves the UART1-captive integration case cleanly.
- Lets the node configure receiver UART2 from the reliable UART1 control path.
- Keeps baud probing, receiver provisioning, and factory recovery close to the receiver wiring.
- Allows PX4 to continue using existing DroneCAN GNSS `Fix2`/`Auxiliary`/relative-heading ingestion for normal navigation.
- Can keep large raw UBX diagnostics off the flight controller unless explicitly tunneled.
- May be a better place for receiver-specific firmware/config quirks when the node is part of the GNSS assembly.

### Costs And Risks Of Running UBX On The DroneCAN Node

- Existing DroneCAN GNSS messages are too generic for HPE signal/RF/raw-measurement diagnostics.
- PX4 loses direct raw UART observability unless the node forwards raw/tunneled UBX data.
- Custom DroneCAN DSDL or a vendor-specific tunnel may be needed for HPE summaries and raw chunks.
- PX4's `uavcan` module would need new subscribers/bridge code for those custom messages.
- CAN bandwidth can become the bottleneck if `RXM-RAWX`, `RXM-SFRBX`, `NAV-SIG`, or `MON-SPAN` are streamed at high rate.
- More logic moves into node firmware, so versioning, testing, and field updates become part of the GNSS integration story.

### Recommended Split For DroneCAN-Based HPE

Use standard DroneCAN GNSS messages for navigation and add a separate HPE diagnostics/tunnel path:

```text
DroneCAN node
  standard GNSS Fix2 / Auxiliary / RelPosHeading
    -> existing PX4 uavcan GNSS bridge
    -> sensor_gps / sensor_gnss_relative

  custom HPE summary and tunnel messages
    -> new PX4 uavcan HPE bridge
    -> ubx_hpe_signal_summary / ubx_hpe_rf_summary / ubx_hpe_time_summary / ubx_hpe_tunnel
```

The node-side UBX driver should publish:

- Standard `Fix2` and `Auxiliary` for normal PX4 GNSS use.
- `RelPosHeading` if moving-baseline heading is needed.
- Compact HPE status messages for `MON-RF`, `NAV-SIG`, `MON-SPAN` summaries, and `NAV-CLOCK` false-time metrics.
- Optional chunked UBX tunnel messages for full `NAV-SIG`, `MON-SPAN`, `RXM-SFRBX`, and `RXM-RAWX` frames.

The PX4-side `uavcan` module should keep the existing GNSS bridge intact and add a new HPE bridge for the custom diagnostic/tunnel messages. This avoids disrupting normal GNSS navigation while still exposing the HPE-specific data path.

### Practical Recommendation

If PX4 has direct access to a UBX-enabled receiver UART, start with the PX4 UART driver because it is easiest to debug and preserves raw evidence. If UART1 is captive and UART2 is not guaranteed to accept UBX config, use the DroneCAN node as the provisioning and UBX endpoint, then bridge standard navigation plus compact HPE diagnostics and selected raw UBX chunks back to PX4.
