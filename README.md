# litegrip-python

[English](README.md) | [简体中文](readme_zn.md)

Python SDK for the **LiteGrip lightweight robotic gripper series**, and the primary public
interface to it. Read this to install the SDK, bring a gripper up, and drive it from your own
program.

You do not need to know CAN or the DM4310 protocol: every move, calibration and force control
is one method call, and each call verifies its own outcome before reporting success.

## What you need

| | |
| --- | --- |
| Motor | Damiao DM4310, CAN ID `0x08`, MIT mode, classic CAN at 1 Mbit/s |
| Host | Linux with SocketCAN — this is the only supported platform |
| Python | 3.8 or newer |
| Dependencies | none: standard library only |
| Optional | `litegrip[zenoh]` — the point-to-point teleoperation link |

## 1. Install

The package is not on PyPI yet. Install it from a checkout:

```bash
git clone https://github.com/nexform-tech/litegrip-python.git
cd litegrip-python
python3 -m pip install .
```

Or skip installation and point `PYTHONPATH` at the source tree:

```bash
PYTHONPATH=/path/to/litegrip-python/src python3 your_script.py
```

## 2. Bring up CAN

The gripper is a CAN device. Bring the interface up at 1 Mbit/s before any SDK call, and do it
again after every reboot:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Both commands need root. If `can0` does not exist at all, the CAN adapter is not loaded yet.
In a product, run these two commands at boot from your init system, so the application never
has to start the interface itself.

## 3. Declare a reverse mount (only if you have one)

The shipped units are **normal mount** — closing increases the motor's rad count — and the
factory calibration the SDK ships describes them. On a normal-mount gripper you declare
nothing: `load_calibration()` in the next section gives you the right direction, and step 5
confirms it.

A gripper whose motor is mounted the other way round is a **reverse mount**, and for it closing
means *decreasing* radians. The SDK assumes neither direction; it derives one from the ordering
of the two calibrated limits. If yours is reverse, say so by name:

```python
from litegrip import LiteGrip

with LiteGrip("can0", mount="reverse") as gripper:
    gripper.enable()
    print(gripper.mount)    # "reverse" — read back from the limits
    gripper.close()         # the jaws should shut, not open
```

If they open instead, the unit is normal after all — drop the argument.

`list_templates()` returns every name the SDK accepts, for a UI to offer. `gripper.mount` reads
back `"normal"`, `"reverse"`, or `None` until calibrated; a name outside `list_templates()`
raises `CommandError` listing the valid ones.

Running a move under a template is safe even though the geometry is not yours yet: `open` and
`close` never send a command outside the calibrated travel, so a wrong direction cannot drive
the jaws past a stop. It only sends them the wrong way, which you see immediately.

## 4. Measure the travel

`zero()` is the one place the SDK asks for a number it cannot measure itself: the jaws' travel
in millimetres, read off calipers. Put it in `GripperConfig.max_stroke_mm` — `85.0` by default,
which is the shipped hardware — and `zero()` probes both mechanical stops, derives the scale,
and saves it:

```python
with LiteGrip("can0") as gripper:
    gripper.enable()
    gripper.config.max_stroke_mm = 87.5     # your caliper measurement
    gripper.zero()                          # probes both stops, saves per channel
```

Set `max_stroke_mm` only when your jaws are not 85 mm. It is the one per-unit measurement no
calibration file carries, and it matters only when you re-probe.

`zero()` drives the jaws onto both real stops, so keep hands and objects out of the travel
range, and support the jaws if the gripper is mounted in a way they would fall.

`zero()` writes what it measured to `~/.litegrip/<channel>_calibration.json`. Nothing reads
that file back on its own: step 5 runs the factory calibration unless you name it with
`--calib ~/.litegrip/can0_calibration.json` or set `LITEGRIP_CALIB`.

### Keep it working

Every LiteGrip shares the same CAN ID (`0x08`), so **the channel is the only identity key**
when two sit on one machine. `zero()` saves one file per channel —
`~/.litegrip/<channel>_calibration.json` — so two units never overwrite each other.

**The factory calibration is the standard value.** A no-argument `load_calibration()` reads
that file and nothing else, so a calibration measured on this machine is never picked up by
accident — not even the one `zero()` just wrote. Name it to use it:

```python
from litegrip import LiteGrip, default_calib_path

with LiteGrip("can0") as gripper:
    gripper.load_calibration(default_calib_path("can0"))    # zero()'s own output
    gripper.enable()
```

`LITEGRIP_CALIB=/path/to/file.json` does the same for every channel without touching the
code. Whenever a no-argument load passes over a local file, the warning names it and repeats
these two ways to use it. **Do not expect it to be picked up automatically.**

The factory file declares `channel: can0`, so a `can1` unit has no implicit source at all and
fails loudly rather than silently adopting `can0`'s direction. Give it one with either
`LiteGrip("can1", mount="normal")` — a template loads by name and ignores the channel — or a
`zero()` of its own, named back as above.

The SDK never guesses a direction: until a calibration is loaded, every motion raises
`CommandError`.

## 5. Check the gripper works

Point this at a freshly wired unit, and again after any calibration change or repair:

```bash
python3 examples/stroke_test.py --channel can0
```

It opens to one limit, closes to the other, prints where each move ended and how hard it
pushed, and exits 0 only if both succeeded.

```bash
python3 examples/stroke_test.py --dry-run                    # the plan, moving nothing
python3 examples/stroke_test.py --cycles 3 --speed 10 --yes   # unattended
```

## Drive it from Python

```python
from litegrip import LiteGrip

with LiteGrip(channel="can0", can_id=0x08) as gripper:
    gripper.load_calibration()
    gripper.enable()                                   # retries until the status frame says err == 1

    gripper.open()                                     # 50 mm/s to the open-side stop
    gripper.close()                                    # 50 mm/s to the closed-side stop
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # close until gripped, then hold 20 N
    print(result.ok, result.stalled, result.cycles)
```

`with` calls `disconnect()` on exit, which disables the motor by default. To leave the gripper
energised after the block, pass `LiteGrip(..., disable_on_disconnect=False)`.

## The six actions

These are the supported entry points for moving the gripper.

| Method | Behaviour | Returns |
| --- | --- | --- |
| `open(speed_mm_s=None)` | Ramps toward the calibrated open-side stop. The command is capped at the calibrated travel, so the jaw stops on the limit and is never driven past it. With `GripperConfig.work_stroke_mm` set it stops at that opening instead — the shipped calibration sets `80.0`, so a plain `open()` ends 5 mm short of the jaws' full travel. | `MoveResult` |
| `close(speed_mm_s=None)` | Same, toward the closed side, with the same command cap. `work_stroke_mm` is an open-side number and does not change where a close ends. | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | Closes until it stalls (i.e. grips), then ramps to and holds `force_n`. The closing leg carries `force_n` as its torque budget, so meeting the object presses no harder than the setpoint. `hold_s=0` holds forever — until a fault, the runaway watch, or Ctrl+C. `force_n=None` uses `MotionConfig.force_n` (20 N). | `GraspResult` |
| `zero()` | Full calibration: probes both mechanical stops, derives travel and `rad_to_mm`, saves to disk. It preserves the direction the loaded calibration declared; a stall cannot tell one stop from the other. | `CalibrationData` |
| `enable(retries=None)` | Sends enable and re-reads the status frame, retrying until it reports `err == 1`. | `EnableResult` |
| `disable()` | Disables the motor — zero torque, back-drivable by hand. | `bool` |

Each one is also reachable through `gripper.actions` (a `GripperActions` instance), which is
what the high-level methods forward to.

### Other ways to calibrate

| Call | When to use it |
| --- | --- |
| `zero()` | The default. Probes both stops under power and saves. |
| `calibrate_guided()` | Guided two-step probe that stops at each limit and waits for you to confirm. |
| `calibrate_manual(duration=30.0)` | Hand-teach: goes slack, you push the jaws through the full stroke, it samples the extremes. |
| `write_zero()` | Declares the *current* encoder angle as the motor's zero, in place. Does not probe the stops and does not touch the calibration file. |

`write_zero()` writes the offset into the motor's flash and **cannot be undone** — the span you
already measured stays valid, but the zero point moves for good. Use it only when you have
deliberately parked the jaws at the closed position, which is what
`examples/zero_closed.py zero` does behind a confirmation prompt.

## Reading the results

The result types are dataclasses that also define `__bool__`, so `if gripper.close():` works.
What it *means*, and what to check, is per action:

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `limit_rad`, `final_cmd_rad`, `steps` | `ok` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles`, `travel_guard` | `ok` |
| `EnableResult` | `ok`, `state`, `tries` | `ok` |

**Read `ok`.** Three combinations look alarming and are correct:

- **`close()` on an empty gripper** returns `ok=True, stalled=True, reached=False`. Success for
  `open`/`close` is *stalling with the command capped at the calibrated limit and the jaw parked
  within `stop_tol` of it*. `reached` is essentially always `False` here, because the target
  sits deliberately past the limit.
- **`grasp()` on an object** returns `stalled=True, reached=False` with `ok=True`. Stopping
  short of the empty-jaw target is the point of gripping.
- **`grasp()` on an empty gripper** can return `reached=True` while the jaw sits about
  `0.010 rad` short of `target_rad` — the close-side stick-slip dead band, which is why
  `reach_tol` defaults to `0.02`.

Stalling far from the limit gives `ok=False`: something blocked the travel.

## Force control

`grasp(force_n, hold_s)` closes, then holds a force. `set_force(force_n, duration=0.3)` holds a
force at wherever the jaws already are, and returns when it is done.

The hold streams frames with `kp = kd = 0`, so the drive's output *is* the feed-forward torque
`force_n × 0.1 Nm`, wherever the jaws are. That is what makes the force independent of the
workpiece: as a soft object yields, the jaws follow it and the grip stays at the setpoint.

Three consequences worth knowing:

- **The force is approximate.** `1 N = 0.1 Nm` is marked approximate in the SDK, and the real
  clamping force also depends on your finger geometry. Treat `force_n` as a repeatable setting,
  not a calibrated measurement.
- **The jaws stay compliant.** An external push back-drives them while the motor keeps pushing
  with `force_n`. An empty `grasp()` closes onto the mechanical stop at that force. That is the
  price of a force that does not drift when the workpiece moves.
- **Below about 3.7 N the approach speed becomes the limit**, because one frame of travel at
  `grasp_speed_mm_s` already spends the whole torque budget. Do not ask for a force the
  mechanism's own friction can absorb; cap the travel time with `grasp_speed_mm_s` instead.

### The runaway watch

The hold frames carry no position error, so no position limit can act on them — the only lever
is removing the torque. So the hold runs a **predicate, not a clamp**: once the measured
position is `hold_runaway_rad` (`0.05 rad`, about 3 mm) past the closed-side calibrated stop,
`grasp()` and `set_force` stream zero-torque release frames and return —
`GraspResult.travel_guard` set, or `False` from `set_force`.

Only the closing side is checked, because that is the side the feed-forward pushes. Crossing
the open limit is pulled back by the same torque.

This is a **runaway detector, not an accuracy bound**. Against an object the jaws stop on the
object; against nothing they stop on the mechanical stop, which sits at the calibrated limit.
Raise `hold_runaway_rad` only if a jaw that is genuinely holding something is being released.
The watch is off until the config is calibrated: the placeholder limits describe no real travel,
and a window built from them is worse than none.

Need more force than the default? `MotionConfig.force_n` is `20.0` N (`2.0 Nm`), inside the
DM4310's `3 Nm` rating. Its peak is `7 Nm` and the protocol ceiling is `10 Nm`.

## Progress reporting

The motion actions never print; they report through a `progress` callback, invoked with a
`MoveProgress` snapshot at the sampling rate.

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

For `grasp`, `p.phase` is `"hold"` while force is being held, and `p.total_steps` is `0`.
`zero()` is the exception to the rule: it prints its own probe progress to stdout.

## Tuning

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
| `force_n` | `20.0` | default `grasp` force |
| `force_ramp_n_s` | `20.0` | rate the held force climbs to its setpoint, N/s |
| `hold_interval` | `0.2` | force-hold slice length, seconds |
| `hold_runaway_rad` | `0.05` | past the closed stop by this much, the force hold drops the torque and stops |
| `margin` | `0.05` | fraction of travel kept in reserve from the stop, used by `grasp` only |
| `max_lead_mm` | `4.0` | cap on how far the command may lead the measured position, on moves that carry no force setpoint |
| `stop_lead_mm` | `0.7` | the same cap on the approach to a stop, so the pressing torque stays bounded |
| `press_safety` | `0.9` | fraction of the setpoint a force-carrying approach may spend |
| `reach_tol` | `0.02` | position tolerance for `reached`, radians |
| `stop_tol` | `0.03` | how close to the calibrated stop the jaw must park to count as pressed home |
| `settle_s` | `0.3` | time spent holding the target after the ramp |

Every other field — the stall window, the probe settings, the frame timing — is documented on
the dataclass itself, with the reasoning for its default.

`sleep_fn` and `monotonic_fn` are the supported seams for simulation: the engine calls only
those two for time, so `sleep_fn=lambda _: None` collapses a whole ramp to no wall-clock time.
The test suite runs a fake CAN with no hardware attached exactly this way.

## Two grippers: leader/follower teleoperation

Two grippers can be linked so one follows the other. The **master** (leader) motor goes slack —
you push its jaws by hand — and publishes how far open it is. The **slave** (follower) drives
its own jaws to match. What travels over the wire is a normalized opening in `[0, 1]`, not an
angle, so the two ends do not need the same calibration, mount, or zero point.

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

The link is a point-to-point **zenoh** session, which is an optional dependency:

```bash
pip install 'litegrip[zenoh]'
```

Both ends use `mode="peer"` with all discovery off, so the only way they find each other is an
explicit endpoint. `link="udp"` selects a plain-UDP fallback for a trusted LAN; it has no
authentication or encryption, so keep either one on a trusted network.

`examples/teleop.py` runs one end from the command line — one process per machine:

```bash
python3 examples/teleop.py --mode master --channel can0
python3 examples/teleop.py --mode slave  --channel can0 --host 192.168.1.20
```

To bench a **single** gripper, `--fake-leader` replaces the leader with a synthetic one on an
in-process bus. Put a rigid object between the jaws first — with nothing to press against, the
follower closes freely and the torque guard has nothing to demonstrate.

```bash
python3 examples/teleop.py --mode slave --channel can0 --fake-leader --torque-limit 1.0
```

Both ends must share `grip_id` (default `gripA`) and be connected and enabled first. Teleop is
exclusive: the background loop owns the CAN I/O, so do not drive the gripper from the caller
until `teleop_stop()`.

- **The leader holds its jaws under gain until the follower reports ready**, so the operator
  cannot push the target out from under a follower that is still travelling. `ready_timeout_s`
  (default `10.0`) bounds the wait; `require_ready=False` restores the immediate hand-back.
- **An optional torque guard releases a follower that is pressing too hard.** Set
  `torque_limit_nm` and it goes to zero stiffness and damping *in place* after 60 ms over the
  limit, then re-arms once the leader has reopened. The default is `0` — **the guard is off
  unless you ask for it**. Pick the value per machine: it depends on how fragile the part
  between the jaws is.
- **A follower that loses the leader holds its position, it does not go slack.** After
  `watchdog_s` (default `0.2`) without a fresh frame, `stale` goes true and the jaws stay put —
  they can hold whatever is between them.
- **The follower feeds the leader's velocity forward** by differencing successive frames, so it
  does not trail a moving leader. `dq_max=0` disables that.
- **Non-finite frames are dropped, never clamped**, on both ends.
- `teleop_status()` reports `active`, `mode`, `frames`, `last_frame_age_ms`, `stale`, `openness`,
  `position_mm`, `force_n`, `loop_hz`, `rejected`, `send_failed`, `fault`, `torque_nm`,
  `over_torque`, `torque_trips`, `ready`, and more. `position_mm` and `force_n` are the master's
  own state, or the leader's values from the frame the slave followed — so a caller can show the
  jaws without opening a second CAN reader.

## Teach and replay a motion

A motion you teach by hand can be captured once and repeated later. Recording puts the motor
into zero-gravity so you can push the jaws through the motion; replay streams the captured
openings back as command frames. What is stored is the normalized opening in `[0, 1]`, exactly
as teleop sends it, so a trajectory taught on one gripper replays on another with a different
mount or calibration.

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
| `trajectory_status()` | One snapshot for both directions: `active`, `kind`, `samples`, `error`, plus `rate_hz` / `loop_hz` while recording and `frames` / `speed` / `openness` / `completed` while playing. |

```bash
python3 examples/trajectory.py --record 5 --save pick   # hand-teach, then save
python3 examples/trajectory.py --list                   # no hardware needed
python3 examples/trajectory.py --play pick --repeat 3
```

- **Replay commands position, not force.** The recorded torque is stored for diagnostics and
  never fed forward, so a squeeze recorded against an object repeats as a position path. Follow
  the replay with `grasp(force_n=...)` if the grip force matters.
- **`record()` is exclusive and the jaws are slack for its whole duration.** Keep a hand on the
  gripper: nothing is holding the jaws.
- **A capture that did not fill raises** rather than returning a short recording as if it were
  whole.
- **Record without zero-gravity when something else drives.**
  `record_start(zero_gravity=False)` only *reads* state, so the caller may run a `grasp()` or a
  move sequence from another thread and capture it. That is the way to record a programmatic
  motion.
- `LITEGRIP_TRAJ_DIR` moves the trajectory directory. Recording, replay and teleop are mutually
  exclusive; `disconnect()` stops whichever is running.

## Build a product on it

- **The stable surface is `LiteGrip` and `gripper.actions`.** The high-level methods forward to
  a `GripperActions` instance, which is the layer to build another driver on — a ROS 2 node, a
  service, a state machine. Nothing above it is required.
- **One owner of the CAN I/O at a time.** Teleop, recording and replay are mutually exclusive
  and raise `TeleopBusyError` / `TrajectoryBusyError`. When your process must stay responsive,
  use the background variants (`teleop_start`, `record_start`, `play_start`) and poll the
  matching `*_status()`.
- **Check `ok`, and read the fields for diagnostics.** `if gripper.close():` compiles and means
  `ok`, but `final_cmd_rad`, `steps` and `state` are what you log when it is false.
- **Errors are typed and never swallowed.** `LiteGripError` and its subclasses — `CommError`,
  `ConnectError`, `CommandError`, `CANTimeoutError`, `HardwareError`, `NotInitializedError` —
  plus `TeleopError` and `TrajectoryError`. `describe_error(code)` turns the motor's own error
  code into text.
- **Crash safety is the default.** `disconnect()` disables the motor, so a process that dies
  does not leave the motor holding torque. Set `disable_on_disconnect=False` only when something
  else keeps the gripper under control.
- **Storage is predictable.** `~/.litegrip/<channel>_calibration.json` and
  `~/.litegrip/trajectories/`. `LITEGRIP_CALIB` and `LITEGRIP_TRAJ_DIR` override them. The
  calibration file is read only when you name it; see step 4.
- **Configure `logging` to see what the SDK decides.** It logs through the standard `logging`
  module: the calibration it loaded, the direction it derived, and every guard that fired.
- **The engine is testable without hardware.** `MotionConfig.sleep_fn` and `monotonic_fn` are
  its only clock seams, which is how the test suite runs a fake CAN in CI.

## Safety and limits

- **Load a calibration before any motion.** Until then, every motion method raises
  `CommandError`. This is the whole gate: on the placeholder defaults a `goto(40.0)` maps to
  `+0.61 rad`, past the real closed stop, and would press there.
- **Keep hands and objects out of the travel range** during `open`, `close`, `grasp` and
  `zero()`. `open` and `close` stop on the calibrated limit; `zero()` probes the real mechanical
  stops under power.
- **A calibration that is wrong limits the travel; it cannot let the jaw run past it.** `open`
  and `close` cap the emitted command at the calibrated extreme, so once the jaw is there the
  residual error is zero and the steady pressing torque is zero.
- **A held force keeps pushing.** Nothing about a hold is position-bounded: the frames carry no
  gains, so the jaws are compliant and the motor pushes with `force_n` wherever they end up.
  The runaway watch is the only thing that stops it, and it is a backstop, not an operating
  limit.
- **`write_zero()` is irreversible.** It writes an encoder offset into the motor's flash.
- **Sustained pressing heats the coil.** `open` and `close` hold against the limit for the
  settle phase every time, so check the coil temperature if they run back to back.
- **A non-finite value is refused, never clamped.** `goto`, `goto_rad`, `move_at_speed*`,
  `grasp` and `set_force` raise `CommandError` for a NaN or ±inf argument, and the CAN boundary
  refuses any such field of a frame before it can become bytes. Clamping a NaN would answer it
  with an end of the range — a real command nobody asked for.

## Troubleshooting

| Symptom | What it means |
| --- | --- |
| `CommandError` naming a calibration | No calibration loaded for this channel. Run step 4. |
| `ConnectError`, or no status frame ever | `can0` is down, or the bitrate is not 1 Mbit/s. See step 2. |
| `HardwareError` from `enable()` | The motor never reported `err == 1`. Check power, then `clear_fault()`. |
| The jaws move the wrong way | The unit is reverse-mounted, not normal. See step 3. |
| `close()` returns `ok=False` | The jaw parked further than `stop_tol` from the limit. Raise `stop_lead_mm` until it presses home. |
| `travel_guard` is set, or `set_force` returns `False` | The runaway watch tripped: the jaw walked past the closed-side stop. Check the calibration describes the real travel, and that nothing has removed the stop. |
| A `can1` unit's `load_calibration()` returns `False` | A no-argument load reads the factory file, which declares `channel: can0`, so this unit has no implicit source. Run `zero()` for it and name the result with `path=` or `LITEGRIP_CALIB`. |
| The held force sags below the setpoint | You changed the hold gains, or something is back-driving the jaws faster than the feed-forward. `hold_kp` / `hold_kd` are deprecated and ignored. |

## Examples

Every script below is runnable from the repository root and prints what it measured.

| Script | What it answers |
| --- | --- |
| `examples/stroke_test.py` | Does this unit work? Both ends of the travel, exit 0 on success. |
| `examples/grasp_test.py` | Does `grasp()` still do what it says — on clear jaws and on an object? |
| `examples/hold_force_test.py` | Does the held grip hold its force, or drift to a smaller one? |
| `examples/zero_closed.py` | Bench tool: `status`, `release`, `goto`, and the irreversible `zero`. |
| `examples/teleop.py` | One end of a leader/follower link. |
| `examples/trajectory.py` | Hand-teach a motion, save it, play it back. |

## Upgrading from 2.2.0

The motion methods were rewritten, so their signatures changed. Results are dataclasses now
instead of bare `bool`s. Old positional calls raise `TypeError` rather than silently doing
something different.

- `grasp(force_n=10.0, kp=150.0, kd=2.0, duration=3.0, ...)` became
  `grasp(force_n=None, hold_s=0.0)`, and the default force is now `20 N`.
- Gains moved out of the calls and onto `GripperConfig` / `MotionConfig`.
- `enable()` now returns an `EnableResult` and reports failure honestly — it used to return
  `True` whenever the status frame held `0` **or** `1`, so a motor that never energised looked
  enabled.
- **`MoveResult.__bool__` changed meaning.** It used to be `reached and not stalled`; it is now
  `ok`. For `grasp`'s closing phase the two agree. For `open` and `close` they are opposite: a
  successful press onto the stop is `stalled=True, reached=False`, so `if gripper.close():`
  means something different from before even though the type never changed.
- `open` and `close` no longer command past the calibrated limit, and the motion actions now
  refuse to run on a configuration that was never calibrated.
- Calibrations are per channel — `~/.litegrip/<channel>_calibration.json`, not the single
  `litegrip_calibration.json`. The old file is still loadable, but only by naming it: a
  no-argument `load_calibration()` reads the factory calibration alone. `LITEGRIP_CALIB` still
  overrides it.

## Development

The test suite is hardware-free: it drives the engine through a kinematic fake CAN in
`tests/fake_can.py`, so it runs anywhere without a gripper or a CAN interface. CI lints both
READMEs and runs this suite:

```bash
python3 -m unittest discover -s tests -t tests -v
```

`litegrip.__version__` is read from the installed distribution metadata, so an installed wheel
reports the version that was actually released. A checkout that was never `pip install`ed
reports `0.0.0+source`; that is expected, not a broken build.

The repository follows Conventional Commits and automated semantic-release versioning on every
merge to `main`; the operating rules for contributors are in [AGENTS.md](AGENTS.md).

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 driver |

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the [Apache License 2.0](LICENSE).
