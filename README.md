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
| Optional extra | `litegrip[zenoh]` — the point-to-point zenoh teleoperation link |

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
All four load the identical file. Both templates carry the shipped unit's geometry and differ
only in which limit they call closed: they declare the direction, and `zero()` then replaces the
limits and the scale with your own measurement. A name outside `list_templates()` raises
`CommandError` listing the valid ones, and a template that cannot be read raises rather than
falling back — the fallback would be the factory file, and that is a *normal* mount, so answering
a request for reverse with normal is the one failure the name exists to prevent.

`zero()` is the only place the SDK asks for a number it cannot measure itself: the jaws' travel,
in millimetres, as read off calipers. Put it in `GripperConfig.max_stroke_mm` — 85 mm by default,
which is the shipped hardware:

```python
gripper.config.max_stroke_mm = 85.0     # your caliper measurement
gripper.zero()                          # probes both stops, derives rad_to_mm, saves to disk
```

The scale it derives is that travel **plus the probe's inset**, over the span the probe recorded:

```text
rad_to_mm = (max_stroke_mm + STOP_INSET_MM) / recorded_travel_rad
```

The inset is not a fudge factor. The probe finds the open stop by pressing about a millimetre
*into* it, so the two recorded extremes span that much more than the jaws actually travel
(`GripperGeometry.SPAN_MM`, 86 mm, against `JAW_TRAVEL_MM`, 85 mm). Two errors follow from getting
it wrong, and they are equal and opposite: dividing the recorded span by the caliper measurement
alone leaves every mm-based move 1.2% short — 1 mm lost per full stroke — while putting the
recorded span into `max_stroke_mm` adds the same millimetre a second time. On the shipped unit the
defaults reproduce the factory file's own scale exactly: `86 / 1.409552 = 61.0123 mm/rad`.

Set `max_stroke_mm` only when the jaws you measured are not 85 mm. It is the one per-unit geometry
the SDK asks for, and no calibration file carries it — a file's own `rad_to_mm` is read as-is, so
the number matters only when you re-probe.

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

The leader does not go slack the instant it starts: it first holds its jaws **under gain**, and
publishes the opening from its very first cycle, until the follower reports it has arrived. That
way the operator cannot push the target out from under a follower that is still travelling.

### Transport

The link is a **point-to-point zenoh** session — the same structure the field teleoperation runs
on. Both ends use `mode="peer"` with all discovery **off** (no multicast, no gossip), so the only
way they find each other is an explicit endpoint: the leader listens on a TCP port, the follower
connects to the leader's address. The topic is the shared litearm namespace,
`litearm/v4/{grip_id}/gripper_teleop`, and the frame is byte-identical to the litearm stack's, so
the two interoperate.

The readiness handshake rides a second, sibling topic on the same session —
`litearm/v4/{grip_id}/gripper_ready`, carrying a single byte — so the teleop frame stays
byte-identical to the litearm stack's. It is the follower that publishes there and the leader that
subscribes: the leader's endpoint listens, so the reverse direction needs no new port.

zenoh is an optional dependency — the base SDK stays stdlib + SocketCAN:

```bash
pip install 'litegrip[zenoh]'
```

`link="udp"` selects a plain-UDP fallback for a trusted LAN; it has no authentication or
encryption. Pass `transport=` a `TeleopTransport` to supply your own; an injected one is never
closed by the SDK.

```python
from litegrip import LiteGrip

# Leader: listen and publish this gripper's opening.
with LiteGrip("can0") as master:
    master.load_calibration()
    master.enable()
    master.teleop_start("master")                      # zenoh, gripA, port 17448

# Follower: connect to the leader, ramp to the first frame, then follow.
with LiteGrip("can0") as slave:
    slave.load_calibration()
    slave.enable()
    slave.teleop_start("slave", host="192.168.1.20")
    while True:
        print(slave.teleop_status())   # frames, openness, loop_hz, stale, ...
```

`examples/teleop.py` runs one end from the command line:

```bash
# Machine A — the leader you push by hand:
python3 examples/teleop.py --mode master --channel can0
# Machine B — the follower:
python3 examples/teleop.py --mode slave  --channel can0 --host 192.168.1.20
```

To bench a **single** gripper, `--fake-leader` replaces the leader with a synthetic one on an
in-process bus: it sweeps its opening open → closed → open, so the follower can be driven into a
hard stop and its torque guard watched tripping and re-arming with no second gripper and no
network. Put a rigid object between the jaws first — with nothing to press against, the follower
closes freely and the guard has nothing to demonstrate.

```bash
python3 examples/teleop.py --mode slave --channel can0 --fake-leader --torque-limit 1.0
```

The script follows the same handshake: the leader holds under gain until the follower announces
itself, `--ready-timeout` bounds that wait (`0` waits indefinitely), and `--no-require-ready`
restores the immediate hand-back.

Both ends must share `grip_id` (default `gripA`) and be connected and enabled first. Teleop is
exclusive: the background loop owns the CAN I/O, so do not drive the gripper from the caller until
`teleop_stop()`. `teleop_start` returns the initial `teleop_status()` snapshot; `teleop_status()`
reports `active`, `mode`, `topic`, `frames`, `last_frame_age_ms`, `stale`, `openness`,
`position_mm`, `force_n`, `dq_cmd`, `loop_hz`, `rejected`, `send_failed`, `fault`, `torque_nm`,
`over_torque`, `torque_trips`, `ready`, `ready_rx`, `ready_pubs`, `ready_timed_out`, and (master)
`matching`. `position_mm` and `force_n` are the
master's own state, or the leader's values from the frame the slave followed — a caller can show
the jaws without opening a second CAN reader.

- **The align move is a bounded-speed ramp, and no align command ever leads the measurement by more
  than `lead_cap_mm`.** The align used to be a single `goto_rad(..., duration=1.0)`. The `duration`
  reads like a ramp, but the CAN layer sends `q = q_target` from the very first frame and merely
  holds it until the deadline, so that first frame demanded `kp` times the whole error — at the
  shipped `kp` of `100.0` Nm/rad any error past ~0.1 rad saturates the DM4310, which is how a
  follower drove into its closed hard stop hard enough to break the printed limit. The align is now
  a constant-speed schedule at `align_speed_mm_s` (default `50` mm/s of jaw travel) with that same
  speed fed forward as `dq`, and each of its frames stays within a lead cap: the commanded position
  may lead the measured one by at most `lead_cap_mm` (default `4` mm). Torque is
  `kp * (q_cmd - q_measured)`, so that bounds the align's commanded torque by construction — about
  5.30 Nm at the config defaults (`kp = 100`, `rad_to_mm = 75.44`). Capping does not make the move
  slower for free: the jaws still travel, they
  just press with a bounded torque while they catch up. The **follow loop is deliberately not
  capped** — it commands the leader's opening outright so the follower stays responsive, and
  `torque_limit_nm` is what protects it under load. `lead_cap_mm=0` disables the align's cap;
  `align_speed_mm_s` must be > 0. On the command line these are `--align-speed` and `--lead-cap`.
  The align also now runs through the loop's own send path, so `torque_limit_nm` covers it — a jam
  during the align releases in place instead of pressing until the move ends.
- **The leader holds its jaws under gain until the follower reports ready.** The follower announces
  itself on `litearm/v4/{grip_id}/gripper_ready` once it has aligned to a frame and is no longer
  stale, not tripped, and within `ready_tolerance_mm` (default `2.0`) of the frame's target; the
  leader only then relaxes into zero-gravity. It **keeps publishing the opening the whole time** —
  the frames are what the follower aligns to, so withholding them would deadlock the handshake —
  only the hand-back is gated. `require_ready=False` restores the immediate relax, and
  `ready_timeout_s` (default `10.0`; `0` waits indefinitely) bounds the wait so an older follower
  that never announces cannot stall the leader: on timeout it warns and relaxes anyway. A leader
  whose transport cannot subscribe disables the gate and goes slack as before, and plain UDP has no
  reverse path, so the gate is inert there. `ready`, `ready_rx`, `ready_pubs`, and
  `ready_timed_out` in `teleop_status()` report the state.
- **The follower feeds the leader's velocity forward.** The wire frame carries only the opening, so
  the follower recovers a velocity by differencing successive frames and sends it as the motor's
  `dq` target — the arm teleoperation sends `dq` outright. Without it the follower biases on
  position error alone and trails a moving leader (the lag scales with speed / `kp`). Because
  `kd * dq` is a real torque term the estimate is bounded: the first frame (nothing to difference
  against), a degenerate interval, and a gap longer than `MAX_FRAME_GAP_S` all yield `dq = 0`, and
  the result is clamped to `dq_max` (default `10.0` rad/s; `dq_max=0` disables the feedforward).
- **A follower that loses the leader holds its position, it does not go slack.** After
  `watchdog_s` (default `0.2`) without a fresh frame it keeps commanding its last target under the
  follow gains, so `stale` goes true but the jaws stay put — and can hold whatever is between them.
- **An optional torque guard releases a follower that is pressing too hard.** Set
  `torque_limit_nm` and the follower watches its own torque (the motor reports no raw current, and
  torque is derived from coil current) every cycle; held at or over the limit for
  `TORQUE_TRIP_CYCLES` (3, i.e. 60 ms at 50 Hz) it goes to zero stiffness and damping **in place**
  — the jaws stop pushing without the loop stopping, and keep streaming so the motor does not latch
  a comm-loss fault. It re-arms only once the leader has reopened by `TORQUE_REARM_OPENNESS` (0.05),
  so it lets go instead of chattering against the same obstruction. A trip within 0.05 of full open
  has no travel left to reopen into, so there the target is the open stop itself; without that cap
  such a trip latched the guard off for the rest of the session. The default is `0` — the guard
  is **off** unless you ask for it. Pick the value per machine: it depends on how fragile the part
  between the jaws is, and because the follow gain is in Nm/rad (`kp` is `100.0` by default) a low
  limit corresponds to a very small position error — watch `torque_nm` under a real press before
  trusting a number. `over_torque` and `torque_trips` in `teleop_status()` report the state.
- **Non-finite frames are dropped, never clamped.** A NaN opening would pass a `[0, 1]` clamp and
  then fold onto a hard stop, silently driving the follower closed. Both ends reject NaN / ±inf at
  the wire boundary — including the first frame used for the align — count them in `rejected`, and
  hold position instead.
- **The follower clamps the target into its own calibrated travel every cycle**, and checks what
  the SDK tells it: a `send_mit_frame` that returns `False` bumps `send_failed`, and a gripper
  `error_code` other than "enabled" is reported in `fault` — neither is swallowed.
- **Stopping leaves the gripper holding**, not slack: the master leaves zero-gravity mode on
  `teleop_stop()` and the follower sends one final frame at its current angle, so both hold under
  the configured gains and neither disables. The one exception is a follower that has tripped the
  torque guard: its final frame keeps the zero gains, because re-applying them would press the very
  thing the guard just let go of.
- Teleop refuses to start on an uncalibrated gripper, a zero-travel one, or one with
  `rad_to_mm == 0` (`TeleopNotReady`), before anything is enabled or driven.
- Follow gains default to the calibration's `kp` / `kd` (`100.0` / `2.0` out of the box); override
  with `kp=` / `kd=`.

## Trajectory record and replay

A motion you teach by hand can be captured once and repeated later. Recording puts the motor into
zero-gravity so you can push the jaws through the motion; replay streams the captured openings back
as MIT command frames. What is stored is the normalized opening in `[0, 1]`, exactly as teleop
sends it, so a trajectory taught on one gripper replays on another with a different mount or
calibration.

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:
    gripper.load_calibration()
    gripper.enable()

    taught = gripper.record(5.0)   # 5 s of hand-teaching; the jaws are slack
    taught.save("pick")            # ~/.litegrip/trajectories/pick.lgt
    gripper.play(taught)           # repeat it
```

| Method | Behaviour |
| --- | --- |
| `record(duration_s, rate_hz=100.0, zero_gravity=True)` | Blocking hand-teach. Returns the `Trajectory`. |
| `record_start(rate_hz=100.0, zero_gravity=True, max_samples=None)` | Background recording; returns the status snapshot. |
| `record_stop(allow_empty=False)` | Stops and returns the captured `Trajectory`. |
| `play(trajectory, speed=1.0, kp=None, kd=None, align=True)` | Blocking replay. `loop` must be `False`. |
| `play_start(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` | Background replay. |
| `play_stop(timeout=2.0)` | Stops a replay and leaves the gripper holding. |
| `trajectory_status()` | One snapshot for both directions. `active`, `kind`, `samples` and `error` are always there; a recording adds `rate_hz`, `zero_gravity` and `loop_hz`, a replay adds `frames`, `speed`, `openness` and `completed`. |

`examples/trajectory.py` runs the same thing from the command line:

```bash
python3 examples/trajectory.py --record 5 --save pick   # hand-teach, then save
python3 examples/trajectory.py --list                   # no hardware needed
python3 examples/trajectory.py --play pick --repeat 3
```

`Trajectory.save("pick")` writes `~/.litegrip/trajectories/pick.lgt`; a name with a path separator
in it is used as written. Set `LITEGRIP_TRAJ_DIR` to move that directory. `Trajectory.load("pick")`
reads it back, and `--list` prints one line per file. The format is compact binary with an 8-byte
magic header, and a file whose length does not match the sample count in its header is rejected
rather than parsed into half a trajectory.

- **Replay commands position, not force.** The recorded torque is stored for diagnostics and never
  fed forward, so a squeeze recorded against an object repeats as a position path that presses with
  whatever `kp` yields. The grip force you taught is not preserved — follow the replay with
  `grasp(force_n=...)` if it matters.
- **`record()` is exclusive and the jaws are slack for its whole duration.** It streams zero-torque
  frames itself, so do not drive the gripper from the caller while it runs, and keep a hand on it:
  nothing is holding the jaws.
- **Record without zero-gravity when something else drives.** `record_start(zero_gravity=False)`
  only *reads* state, so the caller may run a `grasp()` or a move sequence from another thread and
  capture it. That is the way to record a programmatic motion.
- **A capture that did not fill raises.** `record()` reports how many samples it got instead of
  returning a short recording as if it were whole, and a sampling loop that died is never reported
  as a good capture.
- **A blocking `play()` returns with only one hold frame sent.** The motor self-locks a
  communication-loss fault about 100 ms after the frames stop, so call the next action promptly —
  or use `play_start(loop=True)` with `play_stop()` for a hold that lasts. A trajectory of one
  sample is a pose with nothing to repeat, so looping it holds that opening.
- **Recording, replay and teleop are mutually exclusive.** All three own the CAN I/O, and starting
  a second one raises `TeleopBusyError` or `TrajectoryBusyError`. `disconnect()` stops whichever is
  running.
- Record and play both require a loaded calibration: without one the normalized opening is a guess.

## The six actions

These are the supported entry points for moving the gripper. Each one verifies its own
outcome before reporting success, so callers do not re-implement ramps or stall detection.

| Method | Behaviour | Returns |
| --- | --- | --- |
| `open(speed_mm_s=None)` | Ramps *past* the calibrated open-side stop and lets the mechanical stop end the move. With `GripperConfig.work_stroke_mm` set it stops at that opening instead and never presses the stop — the shipped calibration sets it to `80.0`, so a plain `open()` ends 6 mm inside the recorded open extreme, 5 mm short of the jaws' full travel. | `MoveResult` |
| `close(speed_mm_s=None)` | Same, toward the closed side. `work_stroke_mm` is an open-side number and does not change where a close ends. | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | Closes until it stalls (i.e. grips), then ramps to and holds `force_n`. The closing leg carries `force_n` as its torque budget, so meeting the object presses no harder than the setpoint. `hold_s=0` holds forever. | `GraspResult` |
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
| `max_lead_mm` | `4.0` | travel-phase cap on how far the command may lead the measured position, on moves that carry no force setpoint |
| `press_safety` | `0.9` | fraction of the setpoint a force-carrying approach may spend, so the measured force lands below it |
| `press_overshoot` | `0.05` | fraction of travel the `open` / `close` command aims *past* the stop |
| `press_zone_mm` | `2.0` | within this distance of the stop, the lead cap drops to `stop_lead_mm` |
| `stop_lead_mm` | `0.7` | lead cap while pressing, so pressing torque is about `kp × stop_lead_mm` |
| `stop_tol` | `0.02` | how close to the calibrated stop the jaw must park to count as pressed home, radians |
| `force_n` | `20.0` | default `grasp` force |
| `force_ramp_n_s` | `20.0` | rate the held force climbs to its setpoint, N/s |
| `hold_interval` | `0.2` | force-hold slice length, seconds |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | deprecated — the force hold uses no gains; setting them changes nothing |
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
- **An approach that carries a force setpoint spends that setpoint as its budget.** `grasp`'s
  closing leg is still a position frame, so on meeting the object the drive computes the torque
  itself: `kp × lead + kd × commanded speed` — decided by how fast the leg travels and how far the
  engine lets the command lead, not by the force that was asked for. At the config defaults that is
  `kp × max_lead_mm / rad_to_mm` = 5.30 Nm from the lead cap alone, about 53 N, and it was the same
  53 N whether the grasp asked for
  5 N or 40 N. So this leg hands its budget (`force_n × 0.1 × press_safety`) to the three terms a
  frame can spend it on, in this order: the tick's own step (`kp × v × dt`, which only the speed
  can cover), then the damping (`kd`), then the lead. Meeting the object then presses at about
  `0.9 × force_n` — 4.5 N for a 5 N grasp, 36 N for a 40 N one. Two consequences are worth
  knowing. The lead shrinks with the setpoint: 1.7 mm at a 40 N grasp and 50 mm/s, 0.34 mm at
  20 N. And below `~3.7 N` at the default `grasp_speed_mm_s` the *speed* is the setpoint's to
  decide, because one frame of travel at that speed already presses the whole budget — a 1 N
  grasp closes at 13.4 mm/s, a 0.1 N one at 1.3 mm/s. That is deliberate: there is no way to
  approach at 0.1 N quickly, so do not ask for a force the mechanism's own friction can absorb;
  use `grasp_speed_mm_s` to cap the travel time instead.
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
- **The force hold is a torque command, not a position one.** `grasp`'s hold phase and
  `set_force` stream frames with `kp = kd = 0`, so the drive's output is the feed-forward
  torque `force_n × 0.1 Nm` alone, wherever the jaws are. That is what makes the force
  independent of the workpiece: as a soft object yields, the jaws follow it and the grip
  stays at the setpoint. Do not add a gain to make the hold "stiffer" — `kp × (q - measured)`
  turns into a force error the moment the jaws move, and at `kp = 150` the closed side's
  ~`0.0103 rad` stick-slip step is worth about `15 N`. `hold_kp` / `hold_kd` used to do
  exactly that; they are now ignored.
- **The held force is ramped to its setpoint, not stepped to it.** Entering the hold, the
  torque climbs at `force_ramp_n_s` (20 N/s) from the torque already in flight — the
  closing press, about `10 N` on the bench — one step per frame, and lands exactly on the
  setpoint rather than approaching it. A step into a contact is an impulse through the
  mechanism and the fingers bounce off what they just touched; a rate is what makes the
  force climb evenly. At 20 N/s a hand-over from a ~`10 N` press reaches a `20 N` setpoint
  in half a second, and a setpoint moved to at most the rated `40 N` takes two. Do not
  specify the climb as a *duration* instead: a duration-shaped ramp is at its steepest in
  its first tick, which is a step with a slow tail.
- **`set_force`'s `duration` is the hold time after the climb, not a budget that includes
  it.** The call ramps to the setpoint, holds there for `duration` seconds, then returns,
  so its wall clock is `climb + duration`. From a released grip at the default
  `duration=0.3`, `set_force(20.0)` climbs for about `1.0 s` and then holds `0.3 s`, so it
  blocks for about `1.3 s` where it used to be just `0.3 s`. The force always lands on the
  setpoint, so a short `duration` still gets the full force — it only shortens the hold.
  `duration=0` ramps to the setpoint and returns without holding.
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
- **A held force leaves the jaws compliant.** The hold frames carry no stiffness or damping, so
  an external push back-drives the jaws while the motor keeps pushing with `force_n` — and a
  `grasp()` that finds nothing between the jaws closes onto the mechanical stop at that force.
  That is the price of a force that does not drift when the workpiece moves.
- **Direction is data, not a switch.** Both extreme positions live in the calibration file, and
  which of the two is numerically larger is what says which way closing runs
  (`GripperConfig.close_sign`). A reverse-mounted gripper is therefore a perfectly ordinary
  configuration, not an error. What is rejected — with `CommandError` — is a configuration that
  has never been calibrated (`GripperConfig.calibrated` still `False`) or whose two limits are
  equal, because then every direction would be a guess.
- **A non-finite value is refused, never clamped.** `goto`, `goto_rad`, `move_at_speed*`, `grasp`
  and `set_force` raise `CommandError` when an argument is NaN or ±inf, and the CAN boundary
  refuses any such field of an MIT frame before it can become bytes. Clamping a NaN answers it
  with an end of the range — `q → +12.5 rad`, `kp → 500`, `tau → +10 Nm` — a real command nobody
  asked for, and `set_force(nan)` used to spin in its climb loop without sending a single frame or
  ever returning. A feed-forward torque beyond the motor's own frame range (`±10 Nm` on a DM4310,
  `±28 Nm` on a DM4340) is refused for the same reason: the clamp would have shipped a different
  force. A finite out-of-range value still clamps, exactly as before.
- **Pressing torque is bounded by design.** On `open` and `close` — the two moves that press onto
  a stop with no force setpoint — the travel-phase cap of `max_lead_mm` is about
  `kp × max_lead_mm / rad_to_mm`, and the pressing cap of `stop_lead_mm` about
  `kp × stop_lead_mm / rad_to_mm`. **`kp` is whichever is in effect, and a calibration file that
  carries one overrides the `GripperConfig` default.** At the config defaults (`kp=100`,
  `max_lead_mm=4.0`, `stop_lead_mm=0.7`, `rad_to_mm=75.44`) that is `5.30 Nm` and `0.93 Nm`; with
  the shipped calibration loaded it is `0.33 Nm` and `0.06 Nm`, because that file sets `kp=5.0`.
  `grasp`'s closing leg is bounded by its setpoint instead, as described under *How the motion
  works*. The caps cannot usefully go below one frame of travel
  (`speed_mm_s × frame_interval`, `0.25 mm` at the defaults), otherwise the ramp's own step gets
  clipped. If `open`/`close` does not press home reliably — the jaw coasts in and settles further
  than `stop_tol`, so it reports `ok=False` — raise `stop_lead_mm` until it does without audible
  impact. Every number here is arithmetic from the two formulas above, not a hardware measurement:
  whether a given unit presses home at them has to be confirmed on that unit, and the shipped
  calibration's `kp=5.0` makes its `0.06 Nm` the first thing to check.
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
