# litegrip-python

[English](README.md) | [简体中文](readme_zn.md)

Python SDK (primary public interface) for the **LiteGrip lightweight robotic
gripper series**. It drives the gripper's Damiao DM4310 motor over classic CAN
using the MIT control protocol.

## Scope

| | |
| --- | --- |
| Product | LiteGrip lightweight robotic gripper series |
| Repository role | Python SDK (primary public interface) |
| Status | Active — source, packaging and tests are in place |
| Motor | Damiao DM4310, default CAN ID `0x08`, MIT mode, classic CAN at 1 Mbit/s |
| Platform | Linux only (SocketCAN) |
| Python | 3.8 or newer |
| Runtime dependencies | none, standard library only |

## Installation

The package is not on PyPI yet. Install it from a checkout:

```bash
git clone https://github.com/nexform-tech/litegrip-python.git
cd litegrip-python
python3 -m pip install .
```

Or skip installation entirely and point `PYTHONPATH` at the source tree:

```bash
PYTHONPATH=/path/to/litegrip-python/src python3 your_script.py
```

Bring the CAN interface up before running anything:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## Quick start

```python
from litegrip import LiteGrip

with LiteGrip(channel="can0", can_id=0x08) as gripper:
    gripper.load_calibration()   # site calibration, falling back to the factory one
    gripper.enable()             # retries until the status frame reports err == 1

    gripper.open()                                     # 50 mm/s to the open-side stop
    gripper.close()                                    # 50 mm/s to the closed-side stop
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # close until gripped, then hold 20 N
    print(result.reached, result.stalled, result.cycles)
```

`with` calls `disconnect()` on exit, which disables the motor by default. To leave
the gripper energised after the block, pass `LiteGrip(..., disable_on_disconnect=False)`
or set `gripper.disable_on_disconnect = False`.

## The six actions

These are the supported entry points for moving the gripper. Each one verifies its own
outcome before reporting success, so callers do not re-implement ramps or stall detection.

| Method | Behaviour | Returns |
| --- | --- | --- |
| `open(speed_mm_s=None)` | Ramps to the calibrated open-side stop, backed off by `margin`. | `MoveResult` |
| `close(speed_mm_s=None)` | Same, toward the closed side. | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | Closes until it stalls (i.e. grips), then holds `force_n`. `hold_s=0` holds forever. | `GraspResult` |
| `zero()` | Full calibration: probes both mechanical stops, derives travel and `rad_to_mm`, saves to disk. | `CalibrationData` |
| `enable(retries=None)` | Sends enable and re-reads the status frame, retrying until it reports `err == 1`. | `EnableResult` |
| `disable()` | Disables the motor (zero torque, back-drivable by hand). | `bool` |

Each one is also reachable through `gripper.actions` (a `GripperActions` instance), which is
what the high-level methods forward to. Use `gripper.actions` directly when building another
driver layer, such as a ROS 2 node.

### Progress reporting

The motion actions never print; they report through a `progress` callback, invoked with a
`MoveProgress` snapshot at the sampling rate. `zero()` is the exception — it delegates to
`calibrate()`, which prints its own probe progress to stdout.

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

For `grasp`, `p.phase` is `"hold"` while force is being held, and `p.total_steps` is `0`.

## Tuning: MotionConfig

Every tunable lives on one dataclass, `MotionConfig`. The defaults are the values validated on
real hardware. Override them per instance through `gripper.motion_config`, or per call.

```python
from litegrip import LiteGrip, MotionConfig

with LiteGrip("can0") as gripper:
    gripper.motion_config = MotionConfig(speed_mm_s=25.0, force_n=10.0)
    gripper.close()

    gripper.motion_config.force_n = 5.0     # also writable field by field
    gripper.grasp()
```

| Field | Default | Meaning |
| --- | --- | --- |
| `speed_mm_s` | `50.0` | `open` / `close` jaw speed |
| `grasp_speed_mm_s` | `50.0` | closing speed of the `grasp` approach |
| `margin` | `0.05` | fraction of travel kept in reserve from each stop |
| `frame_interval` | `0.005` | ramp frame period, seconds (200 Hz) |
| `sample_interval` | `0.05` | stall sampling period, seconds (20 Hz) |
| `settle_s` | `0.3` | time spent holding the target after the ramp (not stall-checked) |
| `reach_tol` | `0.02` | position tolerance for `reached`, radians |
| `stall_cycles` | `5` | samples per stall window |
| `stall_ratio` | `0.2` | window movement below this fraction of the expected distance counts as stalled |
| `stall_delta` | `0.0015` | floor for the stall threshold, radians |
| `max_lead_mm` | `4.0` | how far the command may lead the measured position |
| `force_n` | `20.0` | default `grasp` force |
| `hold_interval` | `0.2` | force-hold slice length, seconds |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | gains used while holding force |
| `enable_retries` / `enable_retry_interval` | `3` / `0.2` | enable retry count and gap |
| `calib_kp` / `calib_kd` | `60.0` / `2.0` | probe stiffness used by `zero()` |
| `calib_step_rad` | `0.1` | probe increment used by `zero()` |
| `calib_stall_delta` / `calib_stall_cycles` / `calib_max_iter` | `0.0015` / `5` / `80` | probe stall criteria |
| `sleep_fn` / `monotonic_fn` | `time.sleep` / `time.monotonic` | seams for tests and simulation |

`sleep_fn` and `monotonic_fn` are the supported way to simulate the gripper: the engine calls
only those two, so passing `sleep_fn=lambda _: None` collapses a whole ramp to no wall-clock
time. The tests under `tests/` use exactly this to run a fake CAN with no hardware attached.

## Results

The result types are dataclasses that also define `__bool__`, so the old `if gripper.open():`
idiom keeps working.

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `reached`, `stalled`, `state`, `target_rad`, `final_cmd_rad`, `steps` | `reached and not stalled` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles` | `ok` |
| `EnableResult` | `ok`, `state`, `tries` | `ok` |

Two combinations look alarming but are correct:

- **`grasp` on a real object** returns `stalled=True, reached=False` while `ok=True`. Stopping
  short of the empty-jaw target is the point of gripping, so read `ok`, not `reached`.
- **`close` on an empty gripper** can report `reached=True` while the jaw sits about `0.010 rad`
  short of the target. That is the close-side stick-slip dead band, which is why `reach_tol`
  defaults to `0.02` — a tighter tolerance would report a false failure.

## How the motion works

Worth reading if a gripper is behaving oddly.

- **Continuous ramp, not `goto_rad`.** `goto_rad` streams one constant target for a whole
  duration, so the servo snaps onto it in tens of milliseconds and then idles — at low speed
  that reads as step-and-stop. These actions push a linear ramp at `frame_interval` with joint
  velocity feed-forward, the same technique the SDK's own `move_at_speed` uses.
- **Command lead is capped.** The command is an absolute ramp, but the part that leads the
  measured position is clamped to `max_lead_mm`. Uncapped, a blocked jaw accumulates error until
  torque reaches a dangerous value; capped, static friction is still broken at full torque while
  the torque itself stays bounded at roughly `kp × lead`, about 5 Nm by default. Making the
  command relative to the measured position is not an option: the command then freezes with the
  jaw, the error never grows, and the stall test misfires.
- **Stall detection is windowed software logic.** The DM4310 has no stall protection, so the
  engine samples position every `sample_interval` and declares a stall when the net movement over
  `stall_cycles` samples falls below `max(stall_delta, stall_ratio × expected distance)`. A
  single-sample test would misfire on the close-side stick-slip dead band. The settle phase after
  the ramp is not checked, because a stationary jaw is the expected outcome there.
- **`enable` is a verified one-way command.** Enabling sends a CAN frame with no
  acknowledgement, so a dropped frame goes unnoticed and the motor silently stays disabled.
  `enable()` therefore sends it, re-reads the status frame, and reports success only when
  `err == 1`, retrying up to `enable_retries` times and clearing genuine faults first.

## Torque, force and safety

- **The N-to-Nm conversion is approximate.** `UnitConversion.N_TO_NM` is `0.1`, which the SDK
  marks as approximate; real clamping force depends on finger geometry. Treat `force_n` as a
  repeatable setting, not a calibrated measurement.
- **DM4310 limits** are a 3 Nm rating, a 7 Nm peak, and a 10 Nm protocol/firmware ceiling. The
  default 20 N (`2.0 Nm` feed-forward) sits inside the rating.
- **Direction is data, not a switch.** The SDK assumes increasing radians means closing, and
  stores both extremes in the calibration file. If `pos_open_rad >= pos_closed_rad` the
  configuration is rejected with `CommandError`, because the SDK's clamping would be wrong.
- **`open`, `close`, `grasp` and `zero` drive the jaws into mechanical stops or apply sustained
  force.** Keep hands and objects out of the travel range unless you intend to grip them, and
  support the jaws before running `zero()`.
- **Disabling on disconnect is the default**, so a crashed process does not leave the motor
  holding torque. Set `disable_on_disconnect=False` only when something else keeps the gripper
  under control.

## Migrating from the 2.2.0 API

The motion methods were rewritten, so their signatures changed. Results are now dataclasses
instead of bare `bool`s; `__bool__` preserves truthiness, but old positional calls raise
`TypeError` rather than silently doing something different.

| 2.2.0 | Current |
| --- | --- |
| `open(kp=None, kd=None, duration=1.0)` | `open(speed_mm_s=None)` |
| `close(kp=None, kd=None, force_n=None, duration=1.0)` | `close(speed_mm_s=None)`, with force moved to `grasp()` |
| `grasp(force_n=10.0, kp=150.0, kd=2.0, duration=3.0, stall_threshold=0.001, stall_cycles=5)` | `grasp(force_n=None, hold_s=0.0)`, default force now 20 N |
| `enable()`, returning `True` when `err` was `0` **or** `1` | `enable()`, returning `EnableResult`, ok only when `err == 1` |
| `disable() -> bool` | `disable() -> bool` (unchanged) |
| gains passed per call, e.g. `close(kp=150.0)` | gains belong to `GripperConfig`; motion tunables to `MotionConfig` |
| — | `zero()`, `gripper.actions`, `gripper.motion_config`, `disable_on_disconnect` are new |

Two behaviour changes matter beyond the signatures:

- `enable()` now reports failure honestly. It used to return `True` whenever the status frame
  held `0` or `1`, so a motor that never energised looked enabled. Code that ignored the return
  value and carried on will now see a `HardwareError` at startup instead.
- `zero()` is not `calibrate()`. It probes with the `MotionConfig.calib_*` values and saves the
  result, whereas `calibrate()` keeps its own older defaults and does not save.

## Development

The test suite is hardware-free: it drives the engine through a kinematic fake CAN in
`tests/fake_can.py`, so it runs anywhere without a gripper or a CAN interface.

```bash
python3 -m unittest discover -s tests -t tests -v
```

`litegrip.__version__` is read from the installed distribution metadata, so an installed wheel
reports the version that was actually released. A checkout that was never `pip install`ed has
no metadata to read and reports `0.0.0+source`; that is expected, not a broken build.

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
