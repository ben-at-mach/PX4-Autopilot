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
