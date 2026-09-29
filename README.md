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
    gripper.load_calibration()   # this channel's own file, then the factory one
    gripper.enable()             # retries until the status frame reports err == 1

    gripper.open()                                     # 50 mm/s to the open-side stop
    gripper.close()                                    # 50 mm/s to the closed-side stop
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # close until gripped, then hold 20 N
    print(result.reached, result.stalled, result.cycles)
```

`with` calls `disconnect()` on exit, which disables the motor by default. To leave
the gripper energised after the block, pass `LiteGrip(..., disable_on_disconnect=False)`
or set `gripper.disable_on_disconnect = False`.

## Reverse mounts and two grippers on one machine

Which way the motor counts when the jaws close is not fixed: a gripper whose motor
is mounted the other way round is a *reverse mount*, and there closing means
decreasing radians. The SDK does not assume either; it derives the direction from
the ordering of the two calibrated limits, so both mounting work (see
`GripperConfig.close_sign`). What it *does* refuse to do is guess: until a
calibration is loaded, the motion actions raise `CommandError`.

Declare the direction by name. `list_templates()` returns the choices, in order,
for a UI to offer; `mount=` loads the picked one:

```python
from litegrip import LiteGrip, list_templates

list_templates()                       # ["normal", "reverse"] — for a UI

with LiteGrip("can1", mount="reverse") as gripper:
    print(gripper.mount)               # "reverse" — read back from the limits
    gripper.enable()
    gripper.zero()                     # optional: measure the real travel, save it
```

The same selection is reachable four ways: `LiteGrip(..., mount="reverse")`,
`gripper.load_template("reverse")`, `gripper.load_calibration(template="reverse")`,
and — if you already hold the path — `load_calibration(CALIB_TEMPLATES["reverse"])`.
All four load the identical file. Both templates are nominal: they label the
direction and give a plausible stroke, which `zero()` then replaces with the
measurement. A name outside `list_templates()` raises `CommandError` listing the
valid ones, and a template that cannot be read raises rather than falling back —
the fallback would be the factory file, and that is a *normal* mount, so answering
a request for reverse with normal is the one failure the name exists to prevent.

Deciding which is which takes one look: with the jaws visible, run a small move
and see which way they travel. Choosing the wrong mount is not silent — the
derived direction is logged on load, and the first `close()` heads the wrong way.
Read it back any time from `gripper.mount` (`"normal"` / `"reverse"`, or `None`
until calibrated).

Every LiteGrip shares the same CAN ID (`0x08`), so **the channel is the only
identity key** when two sit on one machine. Calibrations are stored one file per
channel — `~/.litegrip/<channel>_calibration.json` — so the two never overwrite
each other, and a no-argument `load_calibration()` reads this channel's own file,
then the legacy single-file location, then the factory one. A candidate that names
a *different* channel is skipped there, so a `can1` unit with no calibration of
its own fails loudly (`False`, then `CommandError` from the motions) instead of
silently adopting `can0`'s direction. Set `LITEGRIP_CALIB` to pin one explicit
path for every channel instead.

## Leader/follower teleoperation

Two grippers can be linked so one follows the other. The **master** (leader) motor goes slack —
you push its jaws by hand — and it publishes how far open it is at the loop rate. The **slave**
(follower) receives that and drives its own jaws to match. What travels over the wire is a
normalized opening in `[0, 1]`, not an angle, so the two ends do not need the same calibration,
mount, or zero point.

```python
from litegrip import LiteGrip

# Leader: publish this gripper's opening to the follower at 192.168.1.20.
with LiteGrip("can0") as master:
    master.load_calibration()
    master.enable()
    master.teleop_start("master", host="192.168.1.20")

# Follower: bind, align to the first frame, then follow.
with LiteGrip("can0") as slave:
    slave.load_calibration()
    slave.enable()
    slave.teleop_start("slave", host="0.0.0.0")
    while True:
        print(slave.teleop_status())   # frames, openness, loop_hz, stale, ...
```

`examples/teleop.py` runs one end from the command line:

```bash
# Machine A — the leader you push by hand:
python3 examples/teleop.py --mode master --channel can0 --host 192.168.1.20
# Machine B — the follower:
python3 examples/teleop.py --mode slave  --channel can0 --host 0.0.0.0
```

Both ends must share `master_id` (default `master`) and be connected and enabled first. Teleop is
exclusive: the background loop owns the CAN I/O, so do not drive the gripper from the caller until
`teleop_stop()`. `teleop_start` returns the initial `teleop_status()` snapshot; `teleop_status()`
reports `active`, `mode`, `topic`, `frames`, `last_frame_age_ms`, `stale`, `openness`, `loop_hz`.

- **The transport is plain UDP**, with no authentication or encryption. Use it only on a trusted
  network. Pass `transport=` a `TeleopTransport` to supply your own; an injected one is never closed
  by the SDK.
- **A follower that loses the leader holds its position, it does not go slack.** After
  `watchdog_s` (default `0.2`) without a fresh frame it keeps commanding its last target under the
  follow gains, so `stale` goes true but the jaws stay put — and can hold whatever is between them.
- **The follower clamps the incoming opening to `[0, 1]`**, i.e. to its own calibrated travel, so a
  bad frame cannot command it past a limit.
- **Stopping leaves the gripper holding**, not slack: the master leaves zero-gravity mode on
  `teleop_stop()`, so its jaws hold under the configured gains.
- Follow gains default to `kp=100.0`, `kd=2.0`; override with `kp=` / `kd=`.

## The six actions

These are the supported entry points for moving the gripper. Each one verifies its own
outcome before reporting success, so callers do not re-implement ramps or stall detection.

| Method | Behaviour | Returns |
| --- | --- | --- |
| `open(speed_mm_s=None)` | Ramps *past* the calibrated open-side stop and lets the mechanical stop end the move. | `MoveResult` |
| `close(speed_mm_s=None)` | Same, toward the closed side. | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | Closes until it stalls (i.e. grips), then holds `force_n`. `hold_s=0` holds forever. | `GraspResult` |
| `zero()` | Full calibration: probes both mechanical stops, derives travel and `rad_to_mm`, saves to disk. It preserves the direction already declared by the loaded calibration; a stall cannot tell one stop from the other. | `CalibrationData` |
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
| `margin` | `0.05` | fraction of travel kept in reserve from the stop, used by `grasp` only |
| `frame_interval` | `0.005` | ramp frame period, seconds (200 Hz) |
| `sample_interval` | `0.05` | stall sampling period, seconds (20 Hz) |
| `settle_s` | `0.3` | time spent holding the target after the ramp (not stall-checked) |
| `reach_tol` | `0.02` | position tolerance for `reached`, radians |
| `stall_cycles` | `5` | samples per stall window |
| `stall_ratio` | `0.2` | window movement below this fraction of the expected distance counts as stalled |
| `stall_delta` | `0.0015` | floor for the stall threshold, radians |
| `max_lead_mm` | `4.0` | travel-phase cap on how far the command may lead the measured position |
| `press_overshoot` | `0.05` | fraction of travel the `open` / `close` command aims *past* the stop |
| `press_zone_mm` | `2.0` | within this distance of the stop, the lead cap drops to `stop_lead_mm` |
| `stop_lead_mm` | `0.7` | lead cap while pressing, so pressing torque is about `kp × stop_lead_mm` |
| `stop_tol` | `0.02` | how close to the calibrated stop the jaw must park to count as pressed home, radians |
| `force_n` | `20.0` | default `grasp` force |
| `hold_interval` | `0.2` | force-hold slice length, seconds |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | gains used while holding force |
| `enable_retries` / `enable_retry_interval` | `3` / `0.2` | enable retry count and gap |
| `calib_kp` / `calib_kd` | `20.0` / `2.0` | probe stiffness used by `zero()` |
| `calib_step_rad` | `0.05` | probe increment used by `zero()`, and the cap on how far the command may lead the measured position |
| `calib_tau_limit` | `2.0` | probe torque ceiling, Nm — the probe freezes as soon as `\|tau\|` reaches it |
| `calib_stall_delta` / `calib_stall_cycles` / `calib_max_iter` | `0.0015` / `5` / `200` | probe stall criteria |
| `sleep_fn` / `monotonic_fn` | `time.sleep` / `time.monotonic` | seams for tests and simulation |

`sleep_fn` and `monotonic_fn` are the supported way to simulate the gripper: the engine calls
only those two, so passing `sleep_fn=lambda _: None` collapses a whole ramp to no wall-clock
time. The tests under `tests/` use exactly this to run a fake CAN with no hardware attached.

## Results

The result types are dataclasses that also define `__bool__`, so the old `if gripper.open():`
idiom still compiles. What it *means* changed for `open` and `close` — see below.

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `limit_rad`, `final_cmd_rad`, `steps` | `ok` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles` | `ok` |
| `EnableResult` | `ok`, `state`, `tries` | `ok` |

`MoveResult.ok` means different things depending on who produced it:

- From `open()` / `close()`: success is *pressing onto the mechanical stop*, so a successful move
  reports `ok=True`, `stalled=True`, and `reached=False` — the target is deliberately past the
  stop, so the jaw never gets there. `reached` is essentially always `False` here; read `ok`.
  Stalling far from the stop means something blocked the travel, and gives `ok=False`.
- From `grasp()`'s closing phase: success is `reached and not stalled`, unchanged from before —
  gripping an object stops short of the empty-jaw target by design.

Two combinations look alarming but are correct:

- **`grasp` on a real object** returns `stalled=True, reached=False` while `ok=True`. Stopping
  short of the empty-jaw target is the point of gripping, so read `ok`, not `reached`.
- **`close` on an empty gripper** reports `ok=True, stalled=True, reached=False` — the jaw has
  parked on the mechanical stop. Before, the same call reported `reached=True, stalled=False`.
- **`grasp`'s closing phase on an empty gripper** can report `reached=True` while the jaw sits
  about `0.010 rad` short of `target_rad`. That is the close-side stick-slip dead band, which is
  why `reach_tol` defaults to `0.02` — a tighter tolerance would report a false failure.

## How the motion works

Worth reading if a gripper is behaving oddly.

- **Continuous ramp, not `goto_rad`.** `goto_rad` streams one constant target for a whole
  duration, so the servo snaps onto it in tens of milliseconds and then idles — at low speed
  that reads as step-and-stop. These actions push a linear ramp at `frame_interval` with joint
  velocity feed-forward, the same technique the SDK's own `move_at_speed` uses.
- **Command lead is capped, in two tiers.** The command is an absolute ramp, but the part that
  leads the measured position is clamped. While travelling it is `max_lead_mm`, enough to break
  static friction; within `press_zone_mm` of the stop it narrows to `stop_lead_mm`, so pressing
  torque stays around `kp × stop_lead_mm` rather than climbing to `kp × press_overshoot`. The
  tier switch applies to the settle phase too — there the command sits past the stop, and an
  uncapped lead would push to about 8 Nm. Uncapped anywhere, a blocked jaw accumulates error until
  torque reaches a dangerous value; capped, the torque stays bounded at roughly `kp × lead`.
  Narrowing the cap globally is not an option either: too small a lead cannot break static
  friction, so the jaw would report a false stall mid-travel. Making the command relative to the
  measured position is not an option: the command then freezes with the jaw, the error never
  grows, and the stall test misfires.
- **`open` and `close` aim past the stop.** Their target is the calibrated stop plus
  `press_overshoot` of travel, and the move ends when stall detection fires on the mechanical
  stop. The calibrated extreme therefore only decides *which way* to travel and what the mm display
  reads — a slightly off calibration no longer moves the endpoint. That also removes the reliance
  on `margin` guessing correctly and on fighting the close-side stick-slip dead band.
  `grasp` is different: it must stop on the *object*, so its closing phase still targets
  `margin` inside the stop.
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
- **Direction is data, not a switch.** Both extreme positions live in the calibration file, and
  which of the two is numerically larger is what says which way closing runs
  (`GripperConfig.close_sign`). A reverse-mounted gripper is therefore a perfectly ordinary
  configuration, not an error. What is rejected — with `CommandError` — is a configuration that
  has never been calibrated (`GripperConfig.calibrated` still `False`) or whose two limits are
  equal, because then every direction would be a guess.
- **Pressing torque is bounded by design.** The travel-phase cap of `max_lead_mm` is about 5 Nm,
  and the pressing cap of `stop_lead_mm` is about `kp × stop_lead_mm / rad_to_mm` — roughly
  `0.94 Nm` at the defaults (`kp=100`, `stop_lead_mm=0.7`, `rad_to_mm≈74`), about a third of the
  3 Nm rating. The cap cannot usefully go below one frame of travel
  (`speed_mm_s × frame_interval`, `0.25 mm` at the defaults), otherwise the ramp's own step gets
  clipped. If it does not press home reliably — the jaw coasts in and settles further than
  `stop_tol`, so `open`/`close` report `ok=False` — raise `stop_lead_mm` until it does without
  audible impact; the default is arithmetic, not a hardware measurement, and needs confirming on
  the real unit.
- **Sustained pressing heats the coil.** `open` and `close` now hold against the stop for the
  settle phase every time, so check the coil temperature if they run back to back.
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
| closing assumed to increase radians | either ordering works — `close_sign` is derived; `CALIB_TEMPLATES` declares the mount |
| `pos_open_rad >= pos_closed_rad` raised `CommandError` | that ordering is a reverse mount; a *never-calibrated* config raises instead (`GripperConfig.calibrated`) |

These behaviour changes matter beyond the signatures:

- `enable()` now reports failure honestly. It used to return `True` whenever the status frame
  held `0` or `1`, so a motor that never energised looked enabled. Code that ignored the return
  value and carried on will now see a `HardwareError` at startup instead.
- `zero()` is not `calibrate()`. Both now probe with the same defaults and both bound the command
  lead to one step and stop at `tau_limit`; the difference is that `zero()` probes with the
  `MotionConfig.calib_*` values and **saves** the result, while `calibrate()` takes its arguments
  directly and does not save.
- The probe is safe at a hard stop. It used to advance its target unconditionally, so once the
  jaws reached a stop the command kept leading further every cycle and `kp × error` kept growing
  until the structure gave way. The command is now re-derived from the measured position each
  cycle (lead ≤ `calib_step_rad`), and the probe aborts the instant `|tau|` reaches
  `calib_tau_limit` — a guard that does not depend on the position-based stall test, which cannot
  fire while the structure is still yielding.
- `MoveResult.__bool__` used to be `reached and not stalled`; it is now `ok`. For `grasp`'s
  closing phase the two agree. For `open` and `close` they are opposite: a successful press onto
  the stop is `stalled=True, reached=False`, so `if gripper.close():` means something different
  from before even though the type never changed.
- The motion actions now refuse to run on a configuration that was never calibrated. Code that
  relied on the placeholder defaults in `GripperConfig` moving the jaws will now raise
  `CommandError` and has to `load_calibration()` first. Those defaults also swapped round, so
  they read as a normal mount rather than a reverse one.
- The pressing lead cap `stop_lead_mm` dropped from `1.0` to `0.7` mm, so `open` and `close` press
  onto the stop with roughly a third of the rated torque instead of half. Raise it back if a unit
  fails to press home.
- Default `save_calibration()` / `load_calibration()` paths are now per channel —
  `~/.litegrip/<channel>_calibration.json`, not the single `litegrip_calibration.json`. That old
  file is still read as a fallback and `LITEGRIP_CALIB` still overrides everything, but a `can0`
  calibration no longer answers a `can1` load.
- The automatic load chain skips a candidate whose `channel` names another interface. A `can1`
  unit with no calibration of its own used to load `can0`'s silently; it now returns `False` and
  the motions raise `CommandError`. Failing loudly beats moving the wrong way.
- Loading by name (`mount=` / `load_template`) never falls back to the factory file. An unreadable
  template raises `CommandError`, because the factory file is a normal mount and falling back
  would answer "reverse" with "normal".
- The two templates now carry only direction and geometry (limits, `rad_to_mm`, motor type), not
  `can_id` / `mst_id` / gains. Loading a mount can no longer rewrite the CAN IDs you passed or a
  tuned `kp`/`kd`. `CALIB_TEMPLATES` and its keys are unchanged.

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
