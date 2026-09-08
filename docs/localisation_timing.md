# Localization timing and GIL comparison

Normal builds release the Python GIL while runtime localization calls compute or
wait for the localization mutex. Particle scoring still owns the same mutex, so
these calls can still delay their caller. Initialization and shutdown retain their
previous serialization. No particle algorithm or I2C schedule was changed.

## Build and verify

Run from the repository root. If the SDK archive is absent, first run
`make -C rplidar_sdk/sdk`.

```bash
.venv/bin/python lib/setup.py build_ext --inplace --force
.venv/bin/python -c 'from lib import lidar; print(lidar.__file__, lidar.runtime_gil_released)'
.venv/bin/python -m pytest tests/localisation_timing_test.py tests/timing_test.py -q
.venv/bin/ruff check
```

The normal extension prints `True`. The concurrency tests intentionally hold the
native mutex for 200 ms in another native thread and check Python progress during
the wait for each runtime binding. Each contention test runs in a subprocess with
a five-second timeout. Other tests cover synthetic localization, partial scans,
prediction, rotation gating, and concurrent getters. These tests require no robot.

To demonstrate the GIL mechanism on a desktop, run this once for each build:

```bash
.venv/bin/python -m scripts.benchmark_localisation_gil logs/gil-demo.json
```

It injects five 200 ms mutex holds and writes the same raw/summary/trace outputs.
The pose calls should still take approximately 200 ms with either build; heartbeat
pauses should shrink in the fixed build. This artificial workload demonstrates
isolation, **not** a prediction of real scan duration or robot speedup.

## Capture on the Pi

```bash
.venv/bin/python main.py --fps --timing-stationary --timing-output logs/fixed-stationary-1.json
```

Activate the normal run switch. The timing window is armed on the **first active
control iteration**, after hardware initialization. It waits 10 seconds, then
captures 60 seconds. After at least 70 seconds, use the normal Enter shutdown so
motor cleanup runs and the capture is saved. Recording ending does not stop or
change robot behavior. `--timing-warmup` and `--timing-duration` override the window.
Pausing does not restart the window; repeat the run if the pause overlaps it.

`--timing-stationary` still executes strategy, camera, localization, IMU and motor
I/O, but replaces strategy outputs with zero translation/dribbler commands and
suppresses kicks. The target heading equals the supplied yaw, giving zero requested
yaw correction. Omit it for normal movement. It also works without timing capture
for an instrumentation-disabled stationary comparison. It does not physically lock
the wheels or measure motor response.

Use these workloads, repeating each three times per build and alternating builds:

1. Stationary, with normal active control and sensor updates.
2. Repeatable translation and gentle turns with scan updates enabled.
3. The recording/streaming settings actually used during games; append their usual
   CLI options. Do not combine session recording and camera MJPEG preview.

Keep field position, scene, camera resolution/rate, power settings and other load
consistent. Fast rotation deliberately pauses scan updates; separate those periods
using the state observations. Repeat a diagnostics-disabled run with `--fps` to
check instrumentation overhead. A 60-second experiment is a screening test, not
proof of long-term real-time performance.

## Instrumented baseline with old GIL behavior

A build-time switch makes an A/B comparison possible from the **same source**, with
identical telemetry and workloads. This is a benchmark baseline, not a production
configuration. `--force` is required because changing compiler macros alone does
not necessarily invalidate setuptools' object cache. Stop the Python process before
rebuilding and start a fresh one afterward.

```bash
SOCCER_LIDAR_HOLD_GIL=1 .venv/bin/python lib/setup.py build_ext --inplace --force
.venv/bin/python -c 'from lib import lidar; print(lidar.__file__, lidar.runtime_gil_released)'
.venv/bin/python -m pytest tests/localisation_timing_test.py tests/timing_test.py -q
.venv/bin/python main.py --fps --timing-stationary --timing-output logs/baseline-stationary-1.json
```

The baseline prints `False`. The concurrency tests assert **no Python progress**
during the controlled wait for this build, and assert progress for the normal
build. A passing baseline test therefore demonstrates the old behavior; it is not
an endorsement of that behavior.

Restore the fixed build after baseline measurements, even if a run is interrupted:

```bash
SOCCER_LIDAR_HOLD_GIL=0 .venv/bin/python lib/setup.py build_ext --inplace --force
.venv/bin/python -c 'from lib import lidar; assert lidar.runtime_gil_released'
```

## Outputs and interpretation

Each capture creates:

- `run.json`: raw per-thread events, state observations, clock alignment check,
  source revision/dirty status, loaded extension and build mode, Python/GIL status,
  configuration, and available temperature/throttling readings at start/end.
- `run.json.summary.json`: timing percentiles, sample counts, event rates, overflow
  counts and percentages of drive intervals above 25 ms and 40 ms.
- `run.json.trace.json`: Chrome Trace Event JSON, loadable in a compatible timeline
  viewer such as Perfetto. Overlay Python drive/heartbeat lanes with native scan
  and mutex intervals. Native lanes have stable per-capture numeric names; event
  names identify the owning operation.

```bash
.venv/bin/python scripts/compare_timing.py \
  logs/baseline-stationary-1.json.summary.json \
  logs/fixed-stationary-1.json.summary.json
```

Buffers keep the first 100,000 events/observations per thread in the window and
count subsequent drops. No per-event file I/O occurs. Drop counts must be zero for
an unbiased full-window comparison. If samples overflow, shorten `--timing-duration`
by the same amount for both builds and repeat. Save occurs on orderly shutdown, so abrupt
power loss does not preserve this diagnostic capture. Summary intervals crossing
the window boundary are excluded; raw traces retain them for context. Percentiles
use the nearest-rank convention. A short capture reports its actual elapsed window.

| Events | Interpretation |
| --- | --- |
| `drive.interval`, `drive.lateness` | Actual uncapped tick interval and lateness against the pre-adjustment schedule; target period 20 ms |
| `drive.writes.interval`, `.failure` | Time between successful four-wheel write batches, plus failed batches; failed writes do not count as completion |
| `drive.command_age`, `drive.yaw_age` | Age of target and its associated host IMU reading when consumed by drive |
| `i2c.drive/imu/odometry.wait/hold` | Time waiting for and occupying the shared bus lock, including driver/Python work inside it |
| `binding.*`, `loc_*.wait/hold` | Python call latency versus native mutex wait/occupancy; Python latency includes GIL reacquisition |
| `scan.accepted`, `scan.skipped` | Native update duration and count; skip includes rotation gate, unavailable state and insufficient scan geometry |
| `logic.interval`, `imu.interval`, `imu.consume_age` | Strategy cadence, successful IMU read cadence and host sample age |
| `camera.*` | Colour conversion, preprocessing, synchronous Hailo call, merge/postprocessing, capture count and source-frame age at publication/consumption |
| `heartbeat.delay` | Excess interval beyond a requested 5 ms sleep; indicates Python scheduling gaps, not definitive GIL attribution |

Stage spans can nest: `camera.detect_scene` includes inference and postprocessing.
Do not add nested spans as if they were independent costs. Camera capture timestamps
are host monotonic callback timestamps, not the sensor's exposure timestamp. IMU
ages similarly use host read completion; motor write completion means the driver
returned, not that physical motion completed.

State observations sample pose/confidence, gate state and counters at about 10 Hz.
Confidence percentage and gate percentage are percentages of these samples, not
exact time-weighted values. Known field positions are needed to measure absolute
pose accuracy. Stationary pose variation alone measures repeatability.

The existing `get_mcl_update_count()` counts recorded confident scan corrections,
not every scan update. Use `scan.accepted` for accepted processing throughput.

Interpret changes across repeated runs, especially p99/max and long-interval counts:

- Smaller heartbeat/drive gaps but similarly slow localization calls: the GIL fix
  worked; consider separately published pose snapshots and queued odometry only if
  the main-loop wait remains unacceptable.
- Better heartbeat but poor wheel writes correlated with bus holds: investigate I2C
  scheduling and transaction cost.
- Pauses correlated with Hailo calls: inspect the installed runtime binding's GIL
  behavior before choosing a fix.
- Remaining Python scheduling delays after native-call/bus issues are addressed:
  prototype a native motor/IMU owner that consumes fresh IMU data independently of
  Python strategy. Rebenchmark before migrating the rest of the application.

The 25/40 ms cutoffs are diagnostic markers, not certified control requirements.
No amount of native code alone guarantees hard real-time deadlines on this system.
