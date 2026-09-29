#!/usr/bin/env python3
"""Teach a gripper a motion by hand and play it back.

This is a runnable companion to the trajectory section of the README.

    # Hand-teach 5 seconds and save it as "pick":
    python3 examples/trajectory.py --record 5 --save pick

    # List what has been saved, with no hardware attached:
    python3 examples/trajectory.py --list

    # Play it back three times:
    python3 examples/trajectory.py --play pick --repeat 3

During ``--record`` the motor goes into zero-gravity and the jaws are yours to
push: take the part, move it through the approach, the squeeze and the release,
and the samples are taken as you go.  Keep a hand on the gripper — nothing is
holding the jaws while it is slack, and whatever is between them will drop.

A saved trajectory stores the opening normalised by *this* unit's travel, so it
replays on a gripper with a different mount or calibration.  What it does not
store is force: replay commands position, with the gains you give it.  A squeeze
recorded against an object repeats as a position path, not as the same grip
force — use ``--kp`` and follow it with ``gripper.grasp(force_n=...)`` if the
force matters.
"""

from __future__ import annotations

import argparse
import sys
import time

from litegrip import LiteGrip, LiteGripError, Trajectory, trajectory_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a gripper motion by hand, or replay a saved one.")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--record", type=float, metavar="SECONDS",
        help="hand-teach for this many seconds (the jaws go slack)")
    action.add_argument(
        "--play", metavar="NAME",
        help="replay a saved trajectory (a bare name, or a path to a .lgt file)")
    action.add_argument(
        "--list", action="store_true",
        help="list the saved trajectories and exit; needs no hardware")

    parser.add_argument(
        "--channel", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--can-id", type=lambda s: int(s, 0), default=0x08,
        help="motor CAN ID (default: 0x08)")
    parser.add_argument(
        "--mount", choices=("normal", "reverse"), default=None,
        help="load a mount template instead of this channel's calibration")
    parser.add_argument(
        "--save", metavar="NAME",
        help="save the recording under this name (default: show it, save nothing)")
    parser.add_argument(
        "--rate", type=float, default=100.0,
        help="samples per second while recording (default: 100)")
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="playback speed multiplier; 0.5 is half speed (default: 1.0)")
    parser.add_argument(
        "--kp", type=float, default=None,
        help="replay stiffness (default: the gripper's configured kp)")
    parser.add_argument(
        "--kd", type=float, default=None,
        help="replay damping (default: the gripper's configured kd)")
    parser.add_argument(
        "--no-align", action="store_true",
        help="replay: do not move to the first sample before following")
    parser.add_argument(
        "--repeat", type=int, default=1,
        help="replay this many times (default: 1)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the resolved plan and exit without touching hardware")
    return parser


def _describe(traj: Trajectory) -> str:
    return (f"{len(traj)} samples, {traj.duration:.2f}s at {traj.sample_hz:.0f}Hz, "
            f"mount={traj.mount}, travel="
            f"{abs(traj.pos_open_rad - traj.pos_closed_rad) * traj.rad_to_mm:.1f}mm")


def _list_saved() -> int:
    """Print the saved trajectories.  Reads the directory, not the bus."""
    import glob
    import os

    root = trajectory_dir()
    paths = sorted(glob.glob(os.path.join(root, "*.lgt")))
    if not paths:
        print(f"no trajectories in {root}")
        return 0
    for path in paths:
        try:
            traj = Trajectory.load(path)
        except LiteGripError as error:
            # One unreadable file must not hide the rest of the list.
            print(f"{os.path.basename(path)}: unreadable ({error})")
            continue
        print(f"{os.path.basename(path):<24} {_describe(traj)}")
    return 0


def _record(gripper: LiteGrip, args: argparse.Namespace) -> int:
    print(f"recording {args.record}s at {args.rate}Hz — the jaws are slack now, "
          f"push them through the motion")
    traj = gripper.record(args.record, rate_hz=args.rate)
    print(f"recorded: {_describe(traj)}")
    if args.save:
        written = traj.save(args.save)
        print(f"saved to {written}")
    else:
        # Say so rather than letting the caller assume a file exists.
        print("not saved (pass --save NAME to keep it)")
    return 0


def _play(gripper: LiteGrip, traj: Trajectory, args: argparse.Namespace) -> int:
    for run in range(1, max(1, args.repeat) + 1):
        status = gripper.play(
            traj, speed=args.speed, kp=args.kp, kd=args.kd,
            align=not args.no_align)
        print(f"replay {run}/{args.repeat}: {status['frames']} frames, "
              f"ended at openness {status['openness']:.3f}")
        if run < args.repeat:
            time.sleep(0.2)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        return _list_saved()

    # Read the file before anything touches the bus: a mistyped name is worth
    # finding out about without connecting, and the dry run should say which
    # trajectory it means, not just which name was typed.
    traj = None
    if args.play is not None:
        traj = Trajectory.load(args.play)
        print(f"loaded {args.play}: {_describe(traj)}")

    gripper = LiteGrip(channel=args.channel, can_id=args.can_id)
    if args.mount is not None:
        gripper.load_calibration(template=args.mount)
    else:
        gripper.load_calibration()
    print(f"mount={gripper.mount} closed={gripper.config.pos_closed_rad:+.4f} "
          f"open={gripper.config.pos_open_rad:+.4f} "
          f"rad_to_mm={gripper.config.rad_to_mm}")

    if args.dry_run:
        what = (f"record {args.record}s at {args.rate}Hz"
                if args.record is not None
                else f"replay {args.play} x{args.repeat} at speed {args.speed}")
        print(f"dry run: would {what} on {args.channel} "
              f"(can_id=0x{args.can_id:02X}); nothing sent")
        return 0

    gripper.connect()
    gripper.enable()
    try:
        if args.record is not None:
            return _record(gripper, args)
        return _play(gripper, traj, args)
    finally:
        # The blocking calls already hold the last position, but a Ctrl+C
        # mid-call would otherwise leave the session claimed and the jaws slack.
        gripper.play_stop()
        gripper.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError as error:
        # A mistyped --play name or --save directory; say which file, not a
        # traceback the user has to read to find out.
        print(f"error: no such file: {error.filename}", file=sys.stderr)
        sys.exit(1)
