#!/usr/bin/env python3
"""Bench tool for the LiteGrip gripper over SocketCAN.

Most subcommands talk to :class:`litegrip.LiteGripCAN` directly — no calibration
file, no motion FSM, no high-level ``LiteGrip`` — and are one explicit call
whose parameters are all on the command line, so nothing moves and nothing is
written unless you asked for it.  ``open``/``close`` are the exception: they go
through the high-level ``LiteGrip`` so the action engine's travel-leg stall
guard runs (see below).

Subcommands::

    # What does the motor report right now?
    python3 examples/zero_closed.py status --channel can0

    # Make the jaws hand-movable (zero stiffness) so you can close them by hand.
    python3 examples/zero_closed.py release --channel can0

    # Or drive them under power to an absolute encoder angle (rad).  Raw MIT —
    # no lead cap, no stall guard, so do not use this to run into a stop.
    python3 examples/zero_closed.py goto --rad -1.40 --kp 100 --kd 2 --duration 2

    # With the jaws fully closed, call that angle zero.  ⚠ irreversible.
    python3 examples/zero_closed.py zero --channel can0 --yes

    # Open / close through the SDK motion engine (calibration + stall guard).
    python3 examples/zero_closed.py open  --channel can0
    python3 examples/zero_closed.py close --channel can0 --speed 40

    # Enable / disable the motor.  ``enable`` alone leaves it limp (kp=0) by
    # design; ``--hold-s`` streams a position hold so the jaw is stiff.
    python3 examples/zero_closed.py enable  --channel can0 --hold-s 5
    python3 examples/zero_closed.py disable --channel can0

``zero`` disables the motor, sends CMD ``0xFE`` (a DM motor **ignores 0xFE while
it is enabled**, so it must be disabled first), then re-enables and reads the
angle back to confirm it now reads ~0.  Whatever angle the motor was at becomes
the new zero — only the offset changes, not the span, so whatever span you
already measured stays valid.  The command requires ``--yes``.

``open``/``close`` load this channel's calibration, enable, and run the action
engine.  ``open`` stops at the configured work stroke if the calibration
carries one, otherwise presses onto the open stop; ``close`` presses onto the
closed stop.  Both carry the ≈7 N travel-leg stall guard, which releases the jaw
(kp=0) if it is blocked mid-travel — ``goto`` has no such guard.

Every ``LiteGripCAN`` subcommand but ``disable`` runs the SDK's ``initialize()``
first, which disables → switches to MIT → enables and leaves the motor *limp*
(kp=0, tau=0), so the jaws are hand-movable at rest.  ``disconnect`` disables the
motor on the way out.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# Like tests/_sdkpath.py: import the SDK straight out of the checkout, so the
# script works without `pip install -e .`.  Inserted first, so the checkout wins
# over an installed copy — running it exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import LiteGrip, LiteGripError  # noqa: E402
from litegrip.can import MotorType  # noqa: E402
from litegrip.protocols import LiteGripCAN  # noqa: E402

# A zeroed reading should come back essentially 0; anything above this means the
# motor did not accept the 0xFE offset.
_ZERO_TOL_RAD = 1e-3


def _hex(value: int) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    # Shared transport/motor options, accepted after any subcommand (so both
    # `zero --channel can0` and `status --can-id 0x08` read naturally).
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    common.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    common.add_argument("--mst-id", type=_hex, default=None,
                        help="master/status ID; default auto-detect")
    common.add_argument("--motor", default="DM4310",
                        choices=[m.name for m in MotorType],
                        help="motor model (default: DM4310)")
    common.add_argument("--canfd", action="store_true",
                        help="open the transport in CAN-FD mode")
    common.add_argument("--dry-run", action="store_true",
                        help="print the resolved plan and exit")

    parser = argparse.ArgumentParser(
        description="Low-level LiteGrip CAN bench tool (no calibration, no "
                    "motion FSM).")
    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("status", parents=[common],
                   help="print position/velocity/torque/error once")

    p = sub.add_parser("release", parents=[common],
                       help="stream kp=0/kd=0/tau=0 to go hand-movable")
    p.add_argument("--duration", type=float, default=0.5,
                   help="how long to stream the limp frames, s (default: 0.5)")

    p = sub.add_parser("goto", parents=[common],
                       help="drive to an absolute angle (rad)")
    p.add_argument("--rad", type=float, required=True, help="target angle, rad")
    p.add_argument("--kp", type=float, default=100.0, help="stiffness (default: 100)")
    p.add_argument("--kd", type=float, default=2.0, help="damping (default: 2)")
    p.add_argument("--duration", type=float, default=2.0,
                   help="how long to stream, s (default: 2)")
    p.add_argument("--interval", type=float, default=0.005,
                   help="frame interval, s (default: 0.005)")

    p = sub.add_parser("zero", parents=[common],
                       help="disable, send 0xFE, re-enable: make the current "
                            "angle zero")
    p.add_argument("--yes", action="store_true",
                   help="confirm the irreversible flash write")

    # open/close go through the high-level LiteGrip, so the action engine's
    # travel-leg stall guard (≈7 N) is in play — unlike `goto`, which streams
    # raw MIT frames with no lead cap and no guard.
    for name, blurb in (("open", "open fully (work stroke, if configured)"),
                        ("close", "close onto the closed-side hard stop")):
        p = sub.add_parser(name, parents=[common], help=blurb)
        p.add_argument("--speed", type=float, default=None,
                       help="speed in mm/s (default: MotionConfig)")

    p = sub.add_parser("enable", parents=[common],
                       help="enable the motor (full initialize: disable → MIT "
                            "→ enable)")
    p.add_argument("--hold-s", type=float, default=0.0,
                   help="stream a position hold (kp/kd) for this many seconds "
                        "so the jaw is stiff, not limp (default 0: enable only)")
    p.add_argument("--hold", action="store_true",
                   help="keep streaming the hold until Ctrl+C (keeps a motor "
                        "with a CAN-timeout from disabling itself)")
    p.add_argument("--kp", type=float, default=100.0,
                   help="hold stiffness (default: 100)")
    p.add_argument("--kd", type=float, default=2.0,
                   help="hold damping (default: 2)")
    sub.add_parser("disable", parents=[common], help="send the disable command")
    return parser


def _report(can: LiteGripCAN) -> str:
    mos, coil = can.get_temperature()
    return (f"pos={can.get_position():+.6f} rad  "
            f"vel={can.get_velocity():+.3f} rad/s  "
            f"tau={can.get_torque():+.3f} Nm  "
            f"err={can.get_error()}  temp={mos}/{coil} C")


def _do_release(can: LiteGripCAN, duration_s: float) -> None:
    """Stream limp frames at the current angle: kp=0 then the jaws move freely."""
    can.control_mit_stream(q_target=can.get_position(), kp=0.0, kd=0.0,
                           duration_s=duration_s, interval_s=0.005)
    print("released (kp=0, kd=0, tau=0) — jaws are hand-movable")


def _do_goto(can: LiteGripCAN, args: argparse.Namespace) -> None:
    can.control_mit_stream(q_target=args.rad, kp=args.kp, kd=args.kd,
                           duration_s=args.duration, interval_s=args.interval)
    can.update_state(timeout_s=0.2)
    print(_report(can))


def _do_zero(can: LiteGripCAN) -> int:
    # 1. Limp, so the jaws stay exactly where the operator put them, and read the
    #    angle we are about to call zero.
    _do_release(can, 0.3)
    if not can.update_state(timeout_s=0.3):
        print("warning: no fresh feedback before zeroing", file=sys.stderr)
    before = can.get_position()
    print(f"0xFE: zeroing at {before:+.6f} rad ...")

    # 2. A DM motor *ignores* 0xFE while it is enabled — disable first.  This is
    #    the order the proven v1.x tool uses (0xFD, then 0xFE).
    can.disable()
    time.sleep(0.02)
    if not can.set_zero():
        print("error: 0xFE could not be sent (not connected/initialized)",
              file=sys.stderr)
        return 1
    time.sleep(0.05)

    # 3. Re-enable and force fresh status frames before reading the new zero.
    if not can.initialize():
        print("error: re-enable after zero failed", file=sys.stderr)
        return 1
    _do_release(can, 0.2)
    fresh = can.update_state(timeout_s=0.3)
    after = can.get_position()
    print(f"read back {after:+.6f} rad (was {before:+.6f})"
          + ("" if fresh else "  [no fresh frame — read-back may be stale]"))
    if abs(after) > _ZERO_TOL_RAD:
        print("warning: read-back is not ~0 — 0xFE did not take; check that the "
              "motor's control mode is MIT and that 24V is up", file=sys.stderr)
    return 0


def _do_move(args: argparse.Namespace) -> int:
    """open/close through the high-level LiteGrip — so the action engine's
    travel-leg stall guard (≈7 N) runs, which `goto` deliberately does not."""
    g = LiteGrip(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id,
                 canfd_mode=args.canfd or None, motor_type=MotorType[args.motor])
    if not g.load_calibration():
        print("warning: no calibration loaded — open/close will refuse",
              file=sys.stderr)
    cfg = g.config
    print(f"calibration: closed={cfg.pos_closed_rad:+.6f} "
          f"open={cfg.pos_open_rad:+.6f} rad_to_mm={cfg.rad_to_mm} "
          f"work_stroke_mm={cfg.work_stroke_mm}")
    print(f"stall guard: {g.actions.config.stop_torque_nm:.2f} Nm "
          f"(≈{g.actions.config.stop_torque_nm * 10:.0f} N) in the travel leg; "
          "releases the jaw on trip")
    try:
        g.connect()
        result = g.enable()
        if not result.ok:
            code = None if result.state is None else result.state.error_code
            print(f"error: enable failed after {result.tries} tries (err={code}) "
                  "— check the 24V supply and the bitrate", file=sys.stderr)
            return 1
        err = None if result.state is None else result.state.error_code
        print(f"enabled (tries={result.tries}, err={err})")
        move = (g.open if args.action == "open" else g.close)(args.speed)
        st = move.state
        print(f"{args.action}: ok={move.ok} reached={move.reached} "
              f"stalled={move.stalled} protected={move.protected} "
              f"pos_mm={st.position_mm:+.1f} target_rad={move.target_rad:+.6f} "
              f"limit_rad={move.limit_rad:+.6f} tau={st.torque_nm:+.3f} Nm "
              f"steps={move.steps}")
        if move.protected:
            print("note: stall guard tripped — the jaw was released (kp=0)",
                  file=sys.stderr)
        return 0 if move.ok else 1
    finally:
        g.disconnect()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    mst = "auto" if args.mst_id is None else f"0x{args.mst_id:02X}"
    if args.dry_run:
        print(f"dry run: {args.action} on {args.channel} can_id=0x{args.can_id:02X} "
              f"mst_id={mst} motor={args.motor}"
              + (f" rad={args.rad}" if args.action == "goto" else ""))
        return 0

    if args.action == "zero" and not args.yes:
        print("error: `zero` writes the encoder zero into the motor's flash and "
              "cannot be undone from here; re-run with --yes if that is what you "
              "want", file=sys.stderr)
        return 2

    if args.action in ("open", "close"):
        return _do_move(args)

    can = LiteGripCAN(channel=args.channel, canfd_mode=args.canfd)
    try:
        can.connect()
        actual = can.register_gripper(can_id=args.can_id, mst_id=args.mst_id,
                                      motor_type=MotorType[args.motor])
        print(f"connected: {args.channel} can_id=0x{args.can_id:02X} "
              f"mst_id=0x{actual:02X} motor={args.motor}")

        if args.action == "disable":
            ok = can.disable()
            print("disabled" if ok else "warning: disable returned False",
                  file=sys.stderr if not ok else sys.stdout)
            return 0 if ok else 1

        # Everything else wants the motor enabled and reporting.
        if not can.initialize():
            print("error: initialize failed — check the 24V supply and that "
                  f"{args.channel} is up at the right bitrate", file=sys.stderr)
            return 1
        can.update_state(timeout_s=0.2)

        if args.action == "enable":
            print("enabled: " + _report(can))
            if args.hold or args.hold_s > 0:
                pos = can.get_position()
                print(f"holding {pos:+.6f} rad at kp={args.kp} kd={args.kd}"
                      + (" until Ctrl+C (watch the LED)" if args.hold
                         else f" for {args.hold_s}s"))
                try:
                    # A DM motor that is not continuously commanded can drop
                    # back to disabled — keep feeding it to stay enabled.
                    while True:
                        can.control_mit_stream(
                            q_target=pos, kp=args.kp, kd=args.kd,
                            duration_s=1.0 if args.hold else args.hold_s,
                            interval_s=0.005)
                        if not args.hold:
                            break
                except KeyboardInterrupt:
                    print("\nhold stopped")
                can.update_state(timeout_s=0.2)
                print("hold done: " + _report(can))
            return 0
        if args.action == "status":
            print(_report(can))
            return 0
        if args.action == "release":
            _do_release(can, args.duration)
            return 0
        if args.action == "goto":
            _do_goto(can, args)
            return 0
        return _do_zero(can)
    finally:
        can.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
