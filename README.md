# litegrip-python

[English](https://github.com/nexform-tech/litegrip-python/blob/main/README.md) | [简体中文](https://github.com/nexform-tech/litegrip-python/blob/main/README.zh-CN.md)

Python SDK for the **LiteGrip lightweight robotic gripper series**, and the primary public
interface to it. One class, `Gripper`, exposes the whole device: enable/disable, open/close,
absolute positioning, adaptive force grasp, calibration, and calibrated feedback.

You do not need to know CAN or the DM4310 protocol. Every move is one blocking method call, and
every call verifies its own outcome before reporting success. A motion that ran but did not reach
its goal returns `False` and logs the reason — it never claims success silently.

## Requirements

| | |
| --- | --- |
| Motor | Damiao DM4310, CAN ID `0x08`, master ID `0x18`, MIT mode, classic CAN at 1 Mbit/s |
| Host | Linux with SocketCAN — the only supported platform |
| Python | 3.10 or newer |
| Dependencies | none: standard library only |

## Install

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

The examples do this for you: each one inserts the checkout's `src/` itself, so
`python3 examples/gripper_example.py --help` works on a bare interpreter.

## Bring up CAN

The gripper is a CAN device. Bring the interface up at 1 Mbit/s before any SDK call, and again
after every reboot:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Both commands need root. If `can0` does not exist, the CAN adapter is not loaded yet. In a product,
run these at boot from your init system so the application never starts the interface itself.

The transport requests classic CAN and upgrades itself when the interface reports an FD MTU. CAN FD
is not part of the public configuration and the DM4310 frame layout is unchanged.

## Quick start

```python
from litegrip import Gripper

with Gripper("can0") as gripper:          # connect() on entry, disconnect() on exit
    gripper.enable()                     # retries until a status frame says err == 1
    gripper.open()                       # toward the open side, default 50 mm/s
    print(gripper.get_gripper_distance()) # mm from the closed zero
    gripper.set_gripper_distance(40.0)   # absolute opening, mm
    gripper.grasp(force_n=20.0, hold_s=3.0)
    print(gripper.get_motor_data())      # {"Position", "Speed", "Torque"}
```

`with` calls `disconnect()` on exit, which disables the motor (zero torque) before closing the bus.

## API

### Constructor

```python
Gripper(channel="can0", can_id=0x08, mst_id=0x18, motion_config=None)
```

| Parameter | Meaning |
| --- | --- |
| `channel` | SocketCAN interface name. |
| `can_id` | Motor CAN ID. |
| `mst_id` | Master/status-frame ID. There is no auto-detection; 0x10 above `can_id` is the default. |
| `motion_config` | Optional `MotionConfig` with the move tuning (speed, frame interval, budgets, …). |

The constructor **loads a calibration automatically**: `$LITEGRIP_CALIB` if set, else
`~/.litegrip/<channel>_calibration.json` if it exists, else the bundled
`factory_calibration.json`. The factory file sets `calibrated=true`, so distance commands work
out of the box. Calling `load_calibration(path)` later overrides the choice.

Diagnostics: `channel`, `can_id`, `mst_id`, `is_connected`, `is_enabled`, `config`,
`motion_config`, and `last_send_error` (why the last low-level frame did not go out).

### Lifecycle

| Method | Returns | What it does |
| --- | --- | --- |
| `connect()` | `bool` | Opens SocketCAN and registers the single motor. Repeated calls return `True`. Raises `ConnectError` / `CommError` on failure. |
| `disconnect()` | `None` | Disables the motor, then closes the bus. Idempotent and non-throwing. |
| `enable(retries=None)` | `bool` | Full init: disable → MIT mode → enable → hold, verified by a fresh status frame (`err == 1`). Retries; clears real faults between attempts. |
| `disable()` | `bool` | Torque off; the jaws become back-drivable. |
| `clear_fault()` | `bool` | Clears latched UV/OC/OT faults and re-enables. Raises `HardwareError` if it cannot. |
| `set_zero()` | `bool` | Makes the **current encoder angle** zero via cmd 0xFE, disables first, re-enables, and requires the read-back to be within 1e-3 rad. **Irreversible** — writes the motor's flash. |
| `calibrate(save=True)` | `CalibrationData` | Probes both mechanical limits with low stiffness and updates `config`; `save=True` writes the per-channel path. Blocking; the jaws move. |
| `load_calibration(path=None)` | `bool` | With `path`, that file is the only source tried (no fallback). Without it: `$LITEGRIP_CALIB`, then the per-channel file, then the factory file. `False` when nothing readable was found. |
| `save_calibration(path=None)` | `str` | Writes the active calibration; default `~/.litegrip/<channel>_calibration.json`. Returns the path. |
| `default_calibration_path(channel="can0")` | `str` | Module-level helper for that default path. |

### Motion

All motion is blocking and requires connected + enabled + calibrated. These methods return `bool`:
`True` means the motion reached its goal; `False` means it ran but did not (the reason is logged).

| Method | Success means |
| --- | --- |
| `open(speed_mm_s=None)` | Pressed and parked near the calibrated open limit, or reached `work_stroke_mm` when that is set below the travel. |
| `close(speed_mm_s=None)` | Pressed and parked near the calibrated closed limit. |
| `set_gripper_distance(distance_mm, speed_mm_s=None)` | Normal positioning to an absolute opening in mm (0 = closed), clamped into the calibrated travel. Reached and not stalled. No limit press. |
| `set_motor_angle(angle_rad, speed_mm_s=None)` | Same, in motor radians. |
| `follow_gripper_distance(distance_mm, kp=None, kd=None, dq=0.0, tau=0.0, force_feedback=None)` | One MIT follow frame was sent (teleoperation; no ramp). See [Teleoperation](#teleoperation). |
| `grasp(force_n=None, hold_s=0.0)` | Adaptive close until stall, then a pure torque hold at `force_n × 0.1 Nm`. Near the closed stop, one short verification squeeze runs before hand-over; if it still moves, the stop was a free-air stiction pocket and the grasp returns false. `hold_s=0` holds until `release()`, a fault, or Ctrl+C. False on no-bite, travel guard, protection, or a hold-phase bus failure; an approach-phase bus failure raises `CommError`. |
| `release(clearance_mm=10.0)` | Thread-safe: ends an in-progress grasp and opens `clearance_mm` beyond the declared grip pose, clamped to the calibrated open limit. Works from another thread while `grasp(hold_s=0)` blocks. Use `disable()` for an emergency torque cut. |

The default speed is `MotionConfig.speed_mm_s` (50 mm/s). `None` in any speed argument selects it.

### Teleoperation

Teleoperation is caller-driven: the SDK exposes one streaming primitive, not a background loop.

```python
import time

while running:
    leader_mm = leader.get_gripper_distance()          # or a Pika Sense
    follower.follow_gripper_distance(leader_mm)        # one MIT frame
    time.sleep(0.005)                                  # 200 Hz
```

The slave example defaults to `--channel can1` for a two-unit host; on a single-unit host pass
`--channel can0`. If the leader is another LiteGrip, call `leader.refresh_status()` before reading — its bus is
idle otherwise. Start with the leader near the follower's current pose: this primitive does no
ramp and no alignment, so the first frame can command the full error.

`follow_gripper_distance()` sends exactly one MIT frame and returns. It does not ramp,
settle, or wait for motion. The value is an absolute opening in mm from the closed zero and
is interpreted with the follower's own calibration, so a leader may send its own
`get_gripper_distance()` even when the two calibration files differ in zero, direction or
scale. A follower with a shorter stroke still clamps at its own travel; for different
strokes, normalise the opening in the caller. The target is clamped into the calibrated
travel. Bounds: kp 0–500, kd 0–5,
`|dq| ≤ 10 rad/s`, `|tau| ≤ 3.5 Nm`; invalid values raise `CommandError` before any bus
write, and `False` means the transport failed (cause in `last_send_error`). The caller owns
the leader read, loop timing, and watchdog: if the loop stops, call `disable()` or keep
sending hold frames — the motor latches a communication-loss fault after ~400 ms of silence
(`TIMEOUT=8000` × 50 µs); SDK waits send 0xCC keepalives to stay under it.

**Force feedback (off by default).** The leader can render the follower's measured squeeze as
resistance:

```python
leader.configure_force_feedback(enabled=True)      # default: disabled
while running:
    follower.follow_gripper_distance(
        leader.get_gripper_distance(),
        force_feedback=leader.render_force_feedback)
    time.sleep(0.005)
```

`follow_gripper_distance(..., force_feedback=cb)` calls `cb(follower.get_gripper_force())`
after a frame is sent; pass the leader's `render_force_feedback`. The leader sends one frame
with `kp=0`, `kd=damping`, `tau=-close_sign × min((F-threshold) × gain, max_tau)`. Defaults:
2 N threshold, 0.1 Nm/N gain, 1 Nm cap, 0.1 damping, 0.3 s timeout; older feedback drops
resistance and damping. The force is approximate (motor torque × 10, not a force sensor).
`render_force_feedback` is **disabled by default** and sends nothing until configured. For a
remote link, forward `get_gripper_force()` yourself and pass the sample age to
`render_force_feedback(force, age_s=...)`.

### Feedback

Getters raise `NotInitializedError` when the gripper is not connected; they never return a
plausible zero. Except for `get_state(wait=True)`, they read the cached snapshot without
blocking — call `refresh_status()` first when the bus has been idle.

| Method | Returns |
| --- | --- |
| `get_state(wait=True)` | `GripperState` snapshot with `position_rad`, `velocity_rad_s`, `torque_nm`, temperatures, `error_code`, `fresh`, `data_age_s`, `timestamp`, `is_stale()`. `wait=True` waits up to 0.5 s for a status frame. |
| `refresh_status()` | `bool` — sends 0xCC and waits briefly for a status frame. Does not move the jaws. |
| `get_motor_data()` | `{"Position": rad, "Speed": rad/s, "Torque": Nm}` |
| `get_motor_status()` | `{"Status": "0x01", "DriverTemp": °C, "MotorTemp": °C}` — MOS and coil. |
| `get_motor_position()` | `float`, rad |
| `get_motor_speed()` | `float`, rad/s |
| `get_motor_torque()` | `float`, Nm |
| `get_gripper_distance()` | `float`, mm opening from the closed zero |
| `get_driver_temp()` | `int`, °C (MOS) |
| `get_motor_temp()` | `int`, °C (coil) |
| `get_status_raw()` | `str`, status/error code as hex (e.g. `"0x01"`) |
| `get_gripper_force()` | `float`, measured squeeze in N (approximate, cached) — follower-side value for force feedback |
| `get_version()` | `str`, the **SDK/package** version — not device firmware (the motor exposes none). |

## Units and invariants

| Quantity | Unit |
| --- | --- |
| Opening / distance | mm, measured from the closed zero. 0 = closed, full stroke = open. |
| Motor angle / position | rad |
| Speed | mm/s for the high-level API, rad/s in feedback |
| Torque | Nm |
| Force | N, **approximate**: 10 N per Nm, not calibrated |
| Temperature | °C |

Nominal geometry: 85 mm jaw travel, 1 mm probe inset, 86 mm span between recorded extremes.
Stale threshold 0.5 s. Maximum command lead in travel 4 mm; default `kp=100`, `kd=2`.

`GripperState.position_mm` and `get_gripper_distance()` use the **opening** convention (single-side
displacement from closed), not total jaw separation.

## Calibration

A calibration is two limits plus a scale: `pos_closed_rad`, `pos_open_rad` and `rad_to_mm`. Which
limit is numerically larger declares the mounting direction, so a reverse-mounted unit needs no
extra switch.

The bundled `factory_calibration.json` is a **reference unit**, not a per-serial file. Zero,
travel and scale vary unit to unit; calibrate each unit (or load the vendor's per-unit file)
before relying on absolute mm accuracy or the `open()`/`close()` stop verdicts.

1. Clear the travel.
2. `gripper.calibrate()` — probes each stop with low stiffness, updates `config`, and writes
   `~/.litegrip/<channel>_calibration.json`.
3. The constructor picks that file up on the next start (`LITEGRIP_CALIB` overrides it).

The probe has two independent guards: the command lead is capped at `calib_step_rad`, so pressing
torque is at most `kp × step`; and the probe stops the moment `|torque|` reaches
`MotionConfig.calib_tau_limit`. The position-stall test alone cannot stop it — at a hard stop the
encoder keeps creeping, so a structure that yields would otherwise be pushed through.

`set_zero()` is different: it declares the **current** encoder angle zero. It does not measure
travel and does not touch the calibration file. It is irreversible.

## Safety

- **The calibrated travel is the outer bound on every emitted command.** `open` and `close` aim
  slightly past the limit so the move ends on a stall, but `clamp_to_travel` caps the command at
  the calibrated extreme. A stale calibration limits the travel; it cannot let the jaws run past
  it.
- **The force-carrying approach has a torque budget.** `grasp()` gives its setpoint
  (`force_n × 0.1 × 0.9 Nm`) to the frame's speed, damping and lead, so meeting an object presses
  no harder than requested.
- **The travel-leg stall guard is independent of the position window.** A hard stop with slow
  structural yield can keep the position window from ever filling; sustained torque plus lagging
  speed trips the guard, which releases torque and returns `False`.
- **A grasp is handed over to the torque-only hold only once the jaws have stopped**, and a stop
  from the starting pose with a budget below breakaway is refused (`no_bite`) instead of becoming a
  powered stroke across free travel.
- **The hold has a runaway watch.** Pure-torque frames carry no position error, so crossing the
  closed-side calibrated limit by more than `MotionConfig.hold_runaway_rad` releases torque and
  returns `False`. Normal grasps and empty grasps do not trip it.
- **Release is coordinated, not a second sender.** While a grasp thread owns the frames, `release()`
  only records a request; the grasp thread opens the jaws and reports the outcome.
- **Non-finite values are refused before any frame is sent.** NaN/±inf targets, speeds, forces and
  hold times raise `CommandError`.
- **Motion is refused until calibrated.** Placeholder limits describe no real travel.
- **Keep hands and objects out of the travel** during `open`, `close`, `grasp` and `calibrate()`.
- **Sustained pressing heats the coil.** Check `get_motor_temp()` if moves run back to back.
- **`disconnect()` disables the motor**, so a killed process does not leave torque on.

## Errors

| Exception | Raised when |
| --- | --- |
| `LiteGripError` | Base class. `str()` appends the motor's code as `[0xXXXX]` when present. |
| `ConnectError` | The CAN interface is unavailable, or the motor cannot be registered. |
| `CommError` | A CAN read/write failed mid-session, including a frame that could not be sent. |
| `CANTimeoutError` | No reply within the timeout. |
| `CommandError` | Invalid argument or invalid state: uncalibrated config, non-finite value, bad speed, negative clearance, malformed calibration file. |
| `HardwareError` | The motor reported a fault: enable never reached `err == 1`, a fault would not clear, 0xFE was not accepted, etc. |
| `NotInitializedError` | Not connected, or not enabled, when the call requires it. |

The SDK logs through the standard `logging` module on the `litegrip` logger. Warnings — a failed
motion, a calibration for another channel, a comm failure — reach stderr even with no handler
configured. `describe_error(code)` turns a motor code into text.

A frame that fails to send keeps its cause in `gripper.last_send_error`. The engine raises
`NotInitializedError` when the missing step is connect/enable, and `CommError` carrying the
transport cause otherwise; when nothing recorded a reason, the message says so instead of guessing.

## Examples

Runnable from the repository root, no install and no `PYTHONPATH`:

| Script | What it does |
| --- | --- |
| `examples/gripper_example.py` | Connect, read status, open/close, absolute distance, grasp/release. |
| `examples/open_close.py` | Repeated open/close cycles with a status line per move. |
| `examples/move_to.py` | Move to an absolute opening (`--distance MM`) or motor angle (`--angle RAD`). |
| `examples/stroke_test.py` | Open/close cycles through the full travel; exit 0 only if every move succeeded. |
| `examples/calibrate.py` | Probe both limits and save the per-channel calibration. |
| `examples/teleop_master.py` | Leader: publish its opening over UDP; optional force feedback (off by default). |
| `examples/teleop_slave.py` | Follower: follow UDP opening targets; publish squeeze back; watchdog holds the last target. |

All examples support `--help` without hardware. A typical bench check:

```bash
python3 examples/stroke_test.py --channel can0 --cycles 3 --speed 10 --yes
```

## Tests

The suite is hardware-free: the engine runs against a kinematic fake CAN in `tests/fake_can.py`,
so it runs anywhere without a gripper or a CAN interface.

```bash
python3 -m unittest discover -s tests -t tests -v
```

Coverage includes the DM4310 codec golden vectors, transport send failures, the motion engine
(lead caps, stall windows, force budgets, hand-over, release coordination), calibration, state
freshness, the public facade, and the examples' `--help` bootstrap.

## Troubleshooting

| Symptom | What it means |
| --- | --- |
| `CommandError` naming a calibration | The active config is uncalibrated. Run `calibrate()`, or point `LITEGRIP_CALIB`/`load_calibration(path=...)` at a valid file. |
| `ConnectError`, or no status frame ever | `can0` is down or the bitrate is not 1 Mbit/s. |
| `HardwareError` from `enable()` | The motor never reported `err == 1`. Check the 24 V supply, then `clear_fault()`. |
| `close()` returns `False` | The jaw parked further than `stop_tol` from the calibrated limit. Check the calibration describes the real travel; the gaps come from the probe pressing harder than `close` does. |
| `grasp()` returns `False` with "no bite" in the log | The approach budget could not break static friction from a standstill. Raise `force_n`; the minimum is around 2.2–3 N on the reference unit. |
| `grasp()` returns `False` after the travel guard fired | The closed side had nothing stopping it while torque was held. Check the calibration and the mechanical stop; the jaws are outside the calibrated travel, so `disable()`, push them back by hand, then `enable()`. |
| `load_calibration()` returns `False` | Neither `path`, `LITEGRIP_CALIB`, the per-channel file nor the factory file was readable. |
| Held force sags | The hold is torque-only by design; if the jaws are back-driven faster than the feed-forward, the force reads lower. There are no hold gains to tune. |
| `import litegrip` loads older code | An installed copy does not update across `git pull`. Check `litegrip.__file__`, or `pip install -e .`. |

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](https://github.com/nexform-tech/litegrip-python/blob/main/LICENSE).
