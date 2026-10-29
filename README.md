# litegrip-python

[English](https://github.com/nexform-tech/litegrip-python/blob/main/README.md) | [简体中文](https://github.com/nexform-tech/litegrip-python/blob/main/README.zh-CN.md)

Python SDK for the **LiteGrip lightweight robotic gripper series**, and the primary public
interface to it. This is the reference: what each interface is, and how to use it.

You do not need to know CAN or the DM4310 protocol: every move, calibration and force control is
one method call, and each call verifies its own outcome before reporting success. Every signature
below is the real one, with its default.

## What you need

| | |
| --- | --- |
| Motor | Damiao DM4310, CAN ID `0x08`, MIT mode, classic CAN at 1 Mbit/s |
| Host | Linux with SocketCAN — this is the only supported platform |
| Python | 3.10 or newer |
| Dependencies | none: standard library only |
| Optional | `litegrip-python[zenoh]` — the point-to-point teleoperation link |

## 1. Install

```bash
pip install litegrip-python
```

Or install from a checkout:

```bash
git clone https://github.com/nexform-tech/litegrip-python.git
cd litegrip-python
python3 -m pip install .
```

Or skip the install and point `PYTHONPATH` at the source tree:

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

Both commands need root. If `can0` does not exist at all, the CAN adapter is not loaded yet. In a
product, run these two commands at boot from your init system, so the application never has to
start the interface itself.

## 3. Quick start

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:            # connect() on entry, disconnect() on exit
    gripper.load_calibration()               # the shipped factory calibration — no arguments
    gripper.enable()                         # retries until the status frame says err == 1

    gripper.open()                           # 50 mm/s toward the open-side stop
    gripper.close()                          # 50 mm/s toward the closed-side stop
    result = gripper.grasp(force_n=20.0, hold_s=3.0)
    print(result.ok, result.stalled, result.cycles)
```

Two things to know before going further:

- **The factory calibration is the default.** A no-argument `load_calibration()` reads the
  calibration file shipped inside the SDK, which describes the shipped units. You name a
  different file only when you have one — see [Calibration](#calibration).
- **Nothing moves before a calibration is loaded.** Every motion method raises `CommandError` on
  an uncalibrated config, because the placeholder limits describe no real travel.

`with` calls `disconnect()` on exit, which disables the motor by default. Pass
`LiteGrip(..., disable_on_disconnect=False)` to leave it energised after the block.

## Interfaces

### LiteGrip

```python
LiteGrip(channel="can0", can_id=0x08, mst_id=None, canfd_mode=None,
         motor_type=MotorType.DM4310, config=None, motion_config=None,
         disable_on_disconnect=True, mount=None)
```

- `channel` — CAN interface name. Every LiteGrip shares CAN ID `0x08`, so the channel is the only
  identity key when two sit on one machine.
- `mount` — **deprecated**, kept only for backward compatibility. Shipped grippers have one
  mounting, and the factory calibration already describes it; leave it unset and use a
  no-argument `load_calibration()`.
- `mst_id` — master ID; `None` auto-detects it.
- `config` / `motion_config` — replace the two dataclasses up front instead of editing them after
  construction.
- `disable_on_disconnect` — `disconnect()` disables the motor (crash safety). Set `False` only
  when something else keeps the gripper under control.

| Attribute | What it is |
| --- | --- |
| `channel`, `can_id`, `mst_id` | The identity this instance drives |
| `config` | `GripperConfig` — limits, scale, gains |
| `motion_config` | `MotionConfig` — every tunable; settable as a whole or field by field |
| `actions` | `GripperActions` — the engine the high-level methods forward to |
| `mount` | **Deprecated**, kept for compatibility; always `"normal"` on a calibrated shipped unit, `None` until calibrated |
| `is_connected`, `is_enabled` | Session state |
| `last_send_error` | Why the last `send_mit_frame` did not send, or `None`. The exception the send reported — an `OSError` from the transport, or the `NotInitializedError` naming the step that was skipped. |
| `session` | The CAN session, for advanced use |

### Connection and state

| Method | What it does |
| --- | --- |
| `connect()` → `bool` | Opens the CAN session. The `with` block does this for you. |
| `disconnect()` → `None` | Closes it, disabling the motor unless `disable_on_disconnect=False`. |
| `poll(timeout_s=0.0)` → `bool` | Reads one frame; `True` if one arrived. |
| `get_state(wait=True)` → `GripperState` | Position, velocity, torque, temperature, error bits, plus how fresh they are. `wait=True` waits up to 50 ms for the frame that updates them — it does not guarantee one. |
| `refresh_status()` → `bool` | Asks the motor for a status frame (0xCC) without moving the jaws. Use it before `get_state()` when the gripper has been idle. |
| `get_position()` / `get_position_rad()` → `float` | Measured opening, in mm / rad. |
| `get_force()` → `float` | `get_torque()` × 10, in N. |
| `get_torque()` → `float` | Measured torque, Nm. |
| `get_temperature()` → `tuple[int, int]` | `(MOS °C, coil °C)`. |
| `get_error()` → `int` | The status code the motor reported: `1` enabled, `0` disabled, `0x9`–`0xC` for faults. `describe_error(code)` names one. |
| `get_info()` → `GripperInfo` | Static device metadata. |
| `is_moving()` / `is_grasped()` → `bool` | Motion and grip predicates. |
| `wait_for_ready(timeout=5.0)` → `bool` | Blocks until the motor accepts commands. |

The readings `get_state()` returns are a cache, and only a status frame updates it. A DM motor
answers the control frames it receives, so a gripper that is being driven reports on its own; one
holding a pose, or disabled, sends nothing, and then `wait=True` waits out its 50 ms and hands back
the previous frame. The snapshot says which happened:

```python
state = gripper.get_state()
if not state.fresh:
    gripper.refresh_status()          # 0xCC: ask for a frame; the jaws do not move
    state = gripper.get_state()

if state.is_stale():                  # older than 0.5 s, or never had a frame
    raise RuntimeError(f"no status frame for {state.data_age_s} s")
print(state.position_mm, state.force_n, state.timestamp)
```

- `fresh` — a frame answered *this* read. False means the numbers are the previous frame's, however
  old that is.
- `data_age_s` — seconds since the frame behind the snapshot arrived, or `None` when none ever has.
- `timestamp` — when that frame arrived, not when you asked. `0.0` means there is no frame at all and
  every reading is a constructor default. **Do not** time a loop with it; it is for a human reading a
  log, and the age arithmetic is what `data_age_s` is for.
- `is_stale(stale_after_s=STALE_AFTER_S)` — age past the threshold (`STALE_AFTER_S` is 0.5 s). It is
  about the snapshot, not the read: a snapshot 20 ms old can still be `fresh=False`.

`fresh` and `is_stale()` answer different questions, and neither substitutes for the other — the wait
window is 50 ms while the staleness threshold is 0.5 s, so a read no frame answered can still look
young. Ask both when the difference matters.

### Enable and faults

| Method | What it does |
| --- | --- |
| `enable(retries=None)` → `EnableResult` | Enables, then re-reads the status frame until it reports `err == 1`. `retries=None` uses `MotionConfig.enable_retries` (3). |
| `disable()` → `bool` | Zero torque; the jaws are back-drivable by hand. |
| `clear_fault()` → `bool` | Clears a latched motor fault. |
| `stop()` → `None` | Sends one zero-torque frame. |

### Motion

| Method | What it does |
| --- | --- |
| `open(speed_mm_s=None, *, progress=None)` → `MoveResult` | Ramps toward the calibrated open-side stop. With `GripperConfig.work_stroke_mm` set it stops at that opening instead — the shipped calibration sets `80.0` against the unit's 85 mm of travel, so a plain `open()` ends 5 mm short of the open end. |
| `close(speed_mm_s=None, *, progress=None)` → `MoveResult` | The same, toward the closed side. `work_stroke_mm` is an open-side number and does not change where a close ends. |
| `home()` → `MoveResult` | Ramps to the calibrated closed position. |
| `grasp(force_n=None, hold_s=0.0, *, progress=None)` → `GraspResult` | Closes onto the object — or onto the closed-side calibrated stop if there is nothing there — then ramps to and holds `force_n` N. The closing leg carries `force_n` as its torque budget, so meeting the object presses no harder than the setpoint. The hold takes over only once the jaw is stopped: it is `stalled=True` (something is between the jaws) or `reached=True` (the jaws closed empty). `hold_s=0` holds until a fault, the runaway watch, Ctrl+C, or a `release()` / `request_release()` from another thread — which ends the grasp with `released=True`. `force_n=None` uses `MotionConfig.force_n` (20 N). |
| `release(clearance_mm=10.0, *, abort=False, timeout_s=None)` → `ReleaseResult` | Ends an in-progress `grasp()` and opens the jaws from the pose where the grip was declared, by `clearance_mm` more, clamped to the calibrated open stop. Safe to call from any thread while `grasp(hold_s=0)` is blocked in another: it queues the request, the grasp thread performs the opening, and the call returns that outcome. With no grasp running it opens from the current position in the calling thread. `abort=True` is the E-stop form: torque off, no opening move, `ok=False`. A second call during the same grasp joins the first. |
| `request_release(clearance_mm=10.0, *, abort=False)` → `bool` | Non-blocking form of `release()`: records the request if a grasp is running and returns `True`, else `False`. Never sends a frame, so it is safe for a UI thread, a signal handler or an E-stop callback. The grasp thread acts on it before its next frame (within 5 ms). |
| `goto(position_mm, kp=None, kd=None, duration=0.5)` → `MoveResult` | Ramp to a position in millimetres. |
| `goto_rad(position_rad, kp=None, kd=None, dq_target=0.0, tau_feedforward=0.0, duration=0.5)` → `MoveResult` | The same in radians. `dq_target` and `tau_feedforward` must be 0: the ramp derives the velocity feed-forward from its own schedule, so both are refused with `CommandError` rather than ignored. |
| `move_to(target_rad, kp=None, kd=None, tau_feedforward=0.0, duration=1.0)` → `MoveResult` | The same as `goto_rad`, with a longer default `duration`. |
| `move_at_speed(target_mm, speed_mm_s=30.0, kp=None, kd=None)` → `MoveResult` | Ramp to a millimetre target at `speed_mm_s`. |
| `move_at_speed_rad(target_rad, speed_rad_s=0.5, kp=None, kd=None)` → `MoveResult` | The same in radians. |

`kp` and `kd` fall back to `GripperConfig.kp` (`100.0`) and `.kd` (`2.0`) on every call that takes
them. Every positioning call — the `goto` family and the `move_at_speed` family alike — runs one
ramp of one frame per `MotionConfig.frame_interval`, and **`duration` is the length of that ramp**,
not a deadline: a 40 mm target with `duration=0.5` travels at 80 mm/s and takes 0.5 s, then holds
the target for `settle_s` (`0.3`) while arrival is decided. The two families differ only in how the
speed is given (a duration, a speed in mm/s, or a speed in rad/s).

The ramp is bounded, and that bound is the point of it. Each frame's command leads the measured
position by at most `MotionConfig.max_lead_mm` (4 mm), which caps the frame's torque near
`kp × 4 mm` — about 6.6 N·m against the shipped `rad_to_mm` of `61.01`, under the DM4310's ±10 N·m
frame limit. A hard step to the target is not bounded: at the shipped `kp` of `100 Nm/rad` any
error past about `0.1 rad` saturates that limit, and a saturated frame overshoots the target
instead of stopping on it. `open` and `close` press onto a calibrated stop on purpose; they cap the
emitted command at the calibrated extreme, so the jaw stops on the limit and is never driven past
it. A NaN or ±inf position is refused with `CommandError`, never clamped.

### Gripping and force

`grasp(force_n, hold_s)` is the only way to press with a force. It closes first and holds second,
and the hold frames carry `kp = kd = 0`, so the drive's output *is* the feed-forward torque
`force_n × 0.1 Nm`. That is what makes the force independent of the workpiece: as a soft object
yields, the jaws follow it and the grip stays at the setpoint.

**The hold is ended by `release()`, and only by it.** A `grasp(hold_s=0)` keeps sending hold frames
until something stops it; there is no separate "stop pressing, stay where you are" call. `release()`
opens from the pose the grip was *declared* at — the end of the force-carrying approach — by
`clearance_mm` more, not from wherever the set force pressed the jaws to: a 20 N grip drives a few
millimetres further in, and opening from there would free that much less. The target is
`max(current, grip pose) + clearance_mm`, clamped to the calibrated open stop and reached by the
same bounded ramp as every other move. Because a grasp may be holding in one thread while the caller
wants it to stop from another, `release()` never sends frames from the caller's thread: it records
the request, the grasp thread sees it before its next frame (5 ms) and performs the opening, and the
caller gets that result. `request_release()` is the same without the wait, for threads that must not
block. `abort=True` drops the torque and sends no opening move — the E-stop form.

**A force is only meaningful against something.** There is no call that applies a force at the
current position, and that is deliberate. Such a call cannot tell whether anything is between the
jaws, and the hold frames give the drive no position or velocity error to work against — with
nothing there, the torque is the only acting force and the jaws accelerate. `grasp()` avoids it
by construction: the closing leg runs on the position ramp, so it arrives at the object (or at
the closed-side stop) and stops there before the hold takes over.

- **The force is approximate.** `1 N = 0.1 Nm` is marked approximate in the SDK, and the real
  clamping force also depends on your finger geometry. Treat `force_n` as a repeatable setting,
  not a calibrated measurement.
- **The jaws stay compliant.** An external push back-drives them while the motor keeps pushing
  with `force_n`, and an empty `grasp()` closes onto the mechanical stop at that force. That is
  the price of a force that does not drift when the workpiece moves.
- **Below about 3.7 N the approach speed becomes the limit**, because one frame of travel at
  `grasp_speed_mm_s` already spends the whole torque budget. Cap the travel time with
  `grasp_speed_mm_s` rather than asking for a force the mechanism's own friction absorbs.

**The runaway watch.** The hold frames carry no position error, so no position limit can act on
them; the only lever is removing the torque. Once the measured position is `hold_runaway_rad`
(`0.05 rad`, about 3 mm) past the closed-side calibrated stop, `grasp()` streams zero-torque
release frames and returns with `GraspResult.travel_guard` set. Only the closing side is checked,
because that is the side the feed-forward pushes.
This is a runaway detector, not an accuracy bound: raise `hold_runaway_rad` only if a jaw that is
genuinely holding something is being released. The watch is off until the config is calibrated.

**An empty `grasp()` is the case that reaches it.** The closing leg presses onto the closed-side
stop and the hold keeps pushing there, so a stop that is not where the calibration says — or one
that has been knocked out — lets the hold walk the jaws past it. The trip leaves them sitting
outside the calibrated travel, and the release frames leave the drive *enabled* but carrying no
torque: it holds nothing, but it is not off either. Get the jaws back inside the travel before
commanding anything else — `disable()` the motor, push them clear of the stop by hand, then
`enable()` and carry on. Do not just call `grasp()` again: the closing leg re-approaches the same
stop and the hold presses it again.

Need more force than the default? `MotionConfig.force_n` is `20.0` N (`2.0 Nm`), inside the
DM4310's `3 Nm` rating. Its peak is `7 Nm` and the protocol ceiling is `10 Nm`.

### Calibration

The factory calibration shipped inside the SDK is the standard value, and a no-argument load reads
it and nothing else:

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:
    gripper.load_calibration()      # the shipped file, describing the shipped unit
    gripper.enable()
```

| Call | What it reads or writes |
| --- | --- |
| `load_calibration()` → `bool` | The shipped factory calibration, plus `LITEGRIP_CALIB` when that is set (the env var wins). |
| `load_calibration(path="/abs/file.json")` → `bool` | That file. Falls back to the factory file only if the path cannot deliver a usable calibration, and warns on a channel mismatch. |
| `load_calibration(template="normal")` / `load_template(name)` → `bool` | **Deprecated**, kept only for backward compatibility; new code uses a no-argument `load_calibration()` or `path=`. |
| `list_templates()` → `List[str]` | **Deprecated**, kept for compatibility; still returns `['normal', 'reverse']`. |
| `default_calib_path(channel="can0")` → `str` | Where a calibration is saved: `~/.litegrip/<channel>_calibration.json`, or `$LITEGRIP_CALIB` when set. |
| `save_calibration(path=None)` → `str` | Writes the current config to that path. |
| `zero()` → `CalibrationData` | Probes both mechanical stops, derives the scale, saves the result. |
| `calibrate_guided(kp=20.0, kd=2.0, step_rad=0.05, ...)` → `CalibrationData` | Two-step probe that stops at each limit and waits for you to confirm. |
| `calibrate_manual(duration=30.0, settle_time=2.0, sample_interval=0.01)` → `CalibrationData` | Hand-teach: goes slack, you push the jaws through the full stroke, it samples the extremes. |
| `write_zero()` → `WriteZeroResult` | Declares the *current* encoder angle as the motor's zero, in place. Writes the motor's flash and **cannot be undone**. |

What follows from that:

- **A calibration measured on your machine is never picked up by accident.** `zero()` writes
  `~/.litegrip/<channel>_calibration.json`, and that file is read only when you name it:
  `load_calibration(path=default_calib_path("can0"))`, or `LITEGRIP_CALIB=/path/to/file.json` for
  every channel at once. **Do not expect it to be picked up automatically.**
- **Only `<channel>_calibration.json` is recognized in that directory.** A calibration file under
  any other name — `can0_08_calibration.json`, say — is not a candidate at all: not tried, not
  merged, not a later fallback stage. Whenever a no-argument load passes over a local file, the
  warning names it and repeats the two ways to use it.
- `zero()` drives the jaws onto both real stops, so keep hands and objects out of the travel
  range, and support the jaws if the gripper is mounted in a way they would fall. Set
  `GripperConfig.max_stroke_mm` (`85.0`, the shipped hardware) when your jaws are not 85 mm; it is
  the one per-unit measurement no calibration file carries, and it matters only when you re-probe.
- The factory file declares `channel: can0`, so a `can1` unit has no implicit source and fails
  loudly — `load_calibration()` returns `False` — rather than silently adopting `can0`'s
  calibration. Name its own `zero()` back as shown in the next section.

### Using your own calibration file

`zero()`, `calibrate_guided()`, and `calibrate_manual()` write their result to
`~/.litegrip/<channel>_calibration.json`. A no-argument `load_calibration()` **does not read
it**; the next run must name it explicitly.

The most direct way is to pass the save location back to `load_calibration()`:

```python
from litegrip import LiteGrip, default_calib_path

path = default_calib_path("can0")        # ~/.litegrip/can0_calibration.json
with LiteGrip("can0") as gripper:
    if not gripper.load_calibration(path=path):
        raise RuntimeError(f"cannot load calibration: {path}")
    gripper.enable()
    gripper.open()
```

For several grippers, or to avoid repeating the path in every script, point an environment
variable at it; this covers every channel at once:

```bash
export LITEGRIP_CALIB="$HOME/.litegrip/can0_calibration.json"
```

```python
gripper.load_calibration()   # reads LITEGRIP_CALIB first, then the factory file
```

Three rules decide whether the file is read:

- **`path=` accepts any JSON path.** It is not limited to `<channel>_calibration.json`; that
  name is only the default destination of `save_calibration()`. A no-argument load does not
  read that directory or guess file names, so do not rename a calibration and expect it to
  be picked up.
- **The `channel` inside `LITEGRIP_CALIB` must match this instance.** If it does not, a
  no-argument load skips the file and falls back to the factory one, so a `can1` unit never
  silently adopts `can0`'s calibration. `path=` does not skip it; it only warns.
- **`path=` falls back to the factory calibration when it cannot read the file, and may still
  return `True`.** Do not trust the return value alone: `Calibration loaded from <path>` in
  the log names the file actually read, or check the values below.

Verify the loaded calibration before commanding motion:

```python
cfg = gripper.config
print(cfg.calibrated)                                               # must be True
print(abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm)   # travel, mm
```

With `calibrated=False`, every motion method raises `CommandError`, so this check is also the gate before motion.

### Reading the results

The result types are dataclasses that also define `__bool__`, so `if gripper.close():` works.
What it *means*, and what to check, is per action:

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `limit_rad`, `final_cmd_rad`, `steps`, `protected`, `released` | `ok` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles`, `travel_guard`, `released` | `ok` |
| `ReleaseResult` | `ok`, `state`, `opened_mm`, `target_rad`, `aborted` | `ok` |
| `EnableResult` | `ok`, `state`, `tries` | `ok` |

**Read `ok`.** Three combinations look alarming and are correct:

- **`close()` on an empty gripper** returns `ok=True, stalled=True, reached=False`. Success for
  `open`/`close` is *stalling with the command capped at the calibrated limit and the jaw parked
  within `stop_tol` of it*. `reached` is essentially always `False` here, because the target sits
  deliberately past the limit.
- **`grasp()` on an object** returns `stalled=True, reached=False` with `ok=True`. Stopping short
  of the empty-jaw target is the point of gripping.
- **`grasp()` on an empty gripper** can return `reached=True` while the jaw sits about
  `0.010 rad` short of `target_rad` — the close-side stick-slip dead band, which is why
  `reach_tol` defaults to `0.02`. `target_rad` is the closed-side calibrated stop itself.

A fourth combination is a failure, and new in this version:

- **`grasp()` with `ok=False, reached=False, stalled=False`** means the closing leg ran out of
  frames without reaching the stop and without stalling — the jaw is still in free travel. The
  hold is *not* entered: its frames carry no position and no velocity term, so handing a moving
  jaw over to them leaves the feed-forward torque as the only thing acting on it, and the jaw
  accelerates until the runaway watch releases it. The call releases in place instead and reports
  `cycles=0`. Nothing was gripped. If you see this, the closing leg is being starved — a stale
  state read, or friction the mechanism's own budget cannot overcome.

- **`grasp()` ended by a release** returns `released=True`: `ok=True` when the opening completed,
  `ok=False` when it was an E-stop (`release(abort=True)`), which sends no opening move. `reached`
  and `stalled` still describe the closing leg; `state` is the pose after the release.

Stalling far from the limit gives `ok=False`: something blocked the travel. The diagnostic fields
— `final_cmd_rad`, `steps`, `state` — are what you log when it is false.

The positioning calls read differently, because their target is a point inside the travel rather
than a stop: `goto`, `goto_rad`, `move_to`, `home` and `move_at_speed*` are `ok` only when the
jaws arrived and did not stall (`reached and not stalled`). Blocked halfway, they come back
`ok=False, stalled=True` — and with `released=True`, meaning the drive's force was taken off
(`kp = kd = tau = 0`) after the stall instead of being left pressing the part. `protected=True` is
the other way a move can end early: the in-travel torque guard (~7 N, see
`MotionConfig.stop_torque_nm`) tripped. That guard takes the force off on its own, so `protected`
leaves `released` at `False`, and it always forces `ok=False`.

### Progress reporting

The motion actions never print; they report through a `progress` callback, invoked with a
`MoveProgress` snapshot at the sampling rate.

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

For `grasp`, `p.phase` is `"hold"` while force is being held, and `p.total_steps` is `0`. `zero()`
is the exception to the rule: it prints its own probe progress to stdout.

### Tuning

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
| `release_wait_s` | `10.0` | how long `release()` waits for the grasp thread when `timeout_s` is not given |
| `hold_runaway_rad` | `0.05` | past the closed stop by this much, the force hold drops the torque and stops |
| `margin` | `0.05` | fraction of travel kept in reserve from the stop by the `limit_target` helper; no shipped call reaches it |
| `max_lead_mm` | `4.0` | cap on how far the command may lead the measured position, on moves that carry no force setpoint |
| `stop_lead_mm` | `0.7` | the same cap on the approach to a stop, so the pressing torque stays bounded |
| `press_safety` | `0.9` | fraction of the setpoint a force-carrying approach may spend |
| `reach_tol` | `0.02` | position tolerance for `reached`, radians |
| `stop_tol` | `0.03` | how close to the calibrated stop the jaw must park to count as pressed home |
| `settle_s` | `0.3` | time spent holding the target after the ramp |

Every other field — the stall window, the probe settings, the frame timing — is documented on the
dataclass itself, with the reasoning for its default.

`sleep_fn` and `monotonic_fn` are the supported seams for simulation: the engine calls only those
two for time, so `sleep_fn=lambda _: None` collapses a whole ramp to no wall-clock time.

### Teleoperation

Two grippers can be linked so one follows the other. The **master** (leader) motor goes slack —
you push its jaws by hand — and publishes how far open it is. The **slave** (follower) drives its
own jaws to match. What travels over the wire is a normalized opening in `[0, 1]`, not an angle,
so the two ends do not need the same calibration, mount, or zero point.

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

| Method | What it does |
| --- | --- |
| `teleop_start(mode, *, transport=None, link="zenoh", host=None, port=17448, grip_id="gripA", kp=None, kd=None, align=True, watchdog_s=0.2, dq_max=10.0, torque_limit_nm=0.0, require_ready=True, ...)` → `dict` | Starts the background loop for `mode="master"` or `"slave"`. Both ends must share `grip_id` and be connected and enabled first. |
| `teleop_stop(timeout=2.0)` → `dict` | Stops the loop and leaves the jaws holding. |
| `teleop_status()` → `dict` | `active`, `mode`, `frames`, `last_frame_age_ms`, `stale`, `openness`, `position_mm`, `force_n`, `loop_hz`, `rejected`, `send_failed`, `fault`, `torque_nm`, `over_torque`, `torque_trips`, `ready`, and more. |

The link is a point-to-point **zenoh** session, an optional dependency:

```bash
pip install 'litegrip-python[zenoh]'
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

- **The leader holds its jaws under gain until the follower reports ready**, so the operator
  cannot push the target out from under a follower that is still travelling. `ready_timeout_s`
  (default `10.0`) bounds the wait; `require_ready=False` restores the immediate hand-back.
- **An optional torque guard releases a follower that is pressing too hard.** Set
  `torque_limit_nm` and it goes to zero stiffness and damping *in place* after 60 ms over the
  limit, then re-arms once the leader has reopened. The default is `0` — **the guard is off
  unless you ask for it**. Pick the value per machine: it depends on how fragile the part between
  the jaws is.
- **A follower that loses the leader holds its position, it does not go slack.** After
  `watchdog_s` (default `0.2`) without a fresh frame, `stale` goes true and the jaws stay put.
- **The follower feeds the leader's velocity forward** by differencing successive frames, so it
  does not trail a moving leader. `dq_max=0` disables that.
- **Non-finite frames are dropped, never clamped**, on both ends.
- Teleop owns the CAN I/O while it runs: do not drive the gripper from the caller until
  `teleop_stop()`.

### Recording and replay

A motion you teach by hand can be captured once and repeated later. Recording puts the motor into
zero-gravity so you can push the jaws through the motion; replay streams the captured openings
back as command frames. What is stored is the normalized opening in `[0, 1]`, exactly as teleop
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
| `record(duration_s, rate_hz=100.0, zero_gravity=True)` → `Trajectory` | Blocking hand-teach. |
| `record_start(rate_hz=100.0, zero_gravity=True, max_samples=None)` → `dict` | Background recording. |
| `record_stop(allow_empty=False)` → `Trajectory` | Stops and returns the capture. |
| `play(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` → `dict` | Blocking replay. `loop=True` raises `ValueError` — a blocking replay would never return. |
| `play_start(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` → `dict` | Background replay. |
| `play_stop(timeout=2.0)` → `dict` | Stops a replay and leaves the gripper holding. |
| `trajectory_status()` → `dict` | One snapshot for both directions: `active`, `kind`, `samples`, `error`, plus `rate_hz` / `loop_hz` while recording and `frames` / `speed` / `openness` / `completed` while playing. |

```bash
python3 examples/trajectory.py --record 5 --save pick   # hand-teach, then save
python3 examples/trajectory.py --list                   # no hardware needed
python3 examples/trajectory.py --play pick --repeat 3
```

- **Replay commands position, not force.** The recorded torque is stored for diagnostics and never
  fed forward, so a squeeze recorded against an object repeats as a position path. Follow the
  replay with `grasp(force_n=...)` if the grip force matters.
- **`record()` is exclusive and the jaws are slack for its whole duration.** Keep a hand on the
  gripper: nothing is holding the jaws, and whatever is between them will drop.
- **A capture that did not fill raises** rather than returning a short recording as if it were
  whole.
- **Record without zero-gravity when something else drives.** `record_start(zero_gravity=False)`
  only *reads* state, so the caller may run a `grasp()` (ended with `release()`) or a move sequence
  from another thread and capture it.
- `LITEGRIP_TRAJ_DIR` moves the trajectory directory. Recording, replay and teleop are mutually
  exclusive and raise `TrajectoryBusyError` / `TeleopBusyError`; `disconnect()` stops whichever is
  running.

### Errors

| Exception | Raised when |
| --- | --- |
| `LiteGripError` | Base class. `str()` appends the motor's code as `[0xXXXX]` when there is one. |
| `ConnectError` | The CAN interface is unavailable, or the motor never replies. |
| `CommError` | A CAN read or write failed mid-session. |
| `CANTimeoutError` | No reply within the timeout. |
| `CommandError` | The command was rejected: bad argument, uncalibrated config, unknown template, or a NaN / ±inf value. |
| `HardwareError` | The motor reported a fault code — for instance `enable()` never saw `err == 1`. |
| `NotInitializedError` | Not connected, or not enabled. |
| `TeleopError` / `TrajectoryError` | Teleop and recording / replay; each has busy, not-active and format subclasses. |

Errors are typed and never swallowed. `describe_error(code)` turns the motor's own error code into
text. The SDK logs through the standard `logging` module — the calibration it loaded, the
direction it derived, every guard that fired; attach a handler to the `litegrip` logger to see
them, and note that warnings reach stderr even with no handler configured.

A frame that fails to go out follows the table rather than one fixed message. The send reports
`False` and keeps the reason in `gripper.last_send_error`: a transport failure raises `CommError`
carrying that reason, and a gripper that is not connected or not enabled raises
`NotInitializedError` naming the missing step. When nothing recorded a reason — a coarse
`send_mit_frame` stub, say — the message says so instead of guessing.

### Advanced

| Interface | What it is for |
| --- | --- |
| `gripper.actions` | The `GripperActions` engine behind the high-level methods — the layer to build another driver on. |
| `send_mit_frame(q, kp, kd, dq=0.0, tau=0.0)` | One raw MIT frame, when you want to compose the control law yourself. |
| `read_param(rid, timeout_s=0.5)` | Reads a motor register. |
| `enter_zero_gravity(duration=0.0)` / `exit_zero_gravity()` | Slack the motor by hand, then take it back. |
| `clamp_to_travel(config, q)` | Clamps an angle into the calibrated travel — the rule `open` and `close` emit under. |
| `limit_target`, `press_target`, `work_limit_target` | The target math behind `open`, `close` and the work-stroke limit. |
| `force_approach_terms(kp, kd, speed_rad_s, frame_interval, budget_nm, lead_ceiling_rad)` | The speed, damping and lead a force-carrying approach may spend. |
| `closing_travel_excursion_rad(config, q)` | How far past the closed-side limit a position is. |
| `CALIB_TEMPLATES`, `UnitConversion`, `GripperParams`, `GripperGeometry`, `ErrorCode` | Template paths, unit conversions, motor limits, geometry and alarm codes. |
| `litegrip.can` | Raw SocketCAN transport and the DM protocol codec, for multi-motor setups. |

`clamp_to_travel`, `limit_target`, `press_target`, `work_limit_target`, `force_approach_terms` and
`closing_travel_excursion_rad` are module-level functions — import them from `litegrip`. They take
the config as their first argument rather than being called on the gripper.

## Troubleshooting

| Symptom | What it means |
| --- | --- |
| `CommandError` naming a calibration | No calibration loaded for this channel. Call `load_calibration()`. |
| `ConnectError`, or no status frame ever | `can0` is down, or the bitrate is not 1 Mbit/s. See step 2. |
| `HardwareError` from `enable()` | The motor never reported `err == 1`. Check power, then `clear_fault()`. |
| `close()` returns `ok=False` | The jaw parked further than `stop_tol` from the limit. The gap comes from `zero()` probing harder than `close()` presses, so raising `stop_lead_mm` barely moves it — check that the calibration describes the real travel, and raise `stop_tol` if the unit is genuinely parked on the stop. |
| `travel_guard` is set | The runaway watch tripped: the jaw walked past the closed-side stop while a force was held — an empty `grasp()` does this when the stop is not where the calibration says. Check the calibration describes the real travel, and that nothing has removed the stop. The jaws are now outside the calibrated travel: `disable()`, push them back clear of the stop by hand, then `enable()`, before commanding anything else. |
| A `can1` unit's `load_calibration()` returns `False` | A no-argument load reads the factory file, which declares `channel: can0`, so this unit has no implicit source. Name its own file with `path=` or `LITEGRIP_CALIB`; see [Using your own calibration file](#using-your-own-calibration-file). |
| The held force sags below the setpoint | You changed the hold gains, or something is back-driving the jaws faster than the feed-forward. `hold_kp` / `hold_kd` are deprecated and ignored. |
| `import litegrip` loads older code than the checkout you are reading | An installed copy does not update itself across `git pull`. Check `litegrip.__file__`, or `pip install -e .` in the checkout. |

## Safety and limits

- **Load a calibration before any motion.** Until then every motion method raises `CommandError`.
  This is the whole gate: on the placeholder defaults a `goto(40.0)` maps to `+0.61 rad`, past the
  real closed stop, and would press there.
- **Keep hands and objects out of the travel range** during `open`, `close`, `grasp` and `zero()`.
  `open` and `close` stop on the calibrated limit; `zero()` probes the real mechanical stops under
  power.
- **A calibration that is wrong limits the travel; it cannot let the jaw run past it.** `open` and
  `close` cap the emitted command at the calibrated extreme, so once the jaw is there the residual
  error is zero and the steady pressing torque is zero.
- **A held force keeps pushing.** Nothing about a hold is position-bounded: the frames carry no
  gains, so the jaws are compliant and the motor pushes with `force_n` wherever they end up. The
  runaway watch is the only thing that stops it, and it is a backstop, not an operating limit.
- **`write_zero()` is irreversible.** It writes an encoder offset into the motor's flash.
- **Sustained pressing heats the coil.** `open` and `close` hold against the limit for the settle
  phase every time, so check the coil temperature if they run back to back.
- **A non-finite value is refused, never clamped.** `goto`, `goto_rad`, `move_at_speed*` and
  `grasp` raise `CommandError` for a NaN or ±inf argument, and the CAN boundary refuses any such
  field of a frame before it can become bytes.
- **`disconnect()` disables the motor by default**, so a process that dies does not leave the motor
  holding torque. Set `disable_on_disconnect=False` only when something else keeps the gripper
  under control.

## Examples

Every script below is runnable from the repository root and prints what it measured.

| Script | What it answers |
| --- | --- |
| `examples/stroke_test.py` | Does this unit work? Both ends of the travel, exit 0 on success. |
| `examples/grasp_test.py` | Does `grasp()` still do what it says — on clear jaws and on an object? |
| `examples/hold_force_test.py` | Does the held grip hold its force, or drift to a smaller one? |
| `examples/zero_closed.py` | Low-level CAN bench tool, one subcommand per operation: `status`, `release`, `goto`, `zero`, `open`, `close`, `enable`, `disable`. |
| `examples/teleop.py` | One end of a leader/follower link. |
| `examples/trajectory.py` | Hand-teach a motion, save it, play it back. |

The test suite is hardware-free: it drives the engine through a kinematic fake CAN in
`tests/fake_can.py`, so it runs anywhere without a gripper or a CAN interface.

```bash
python3 -m unittest discover -s tests -t tests -v
```

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the [Apache License 2.0](https://github.com/nexform-tech/litegrip-python/blob/main/LICENSE).
