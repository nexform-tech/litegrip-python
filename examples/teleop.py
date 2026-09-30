#!/usr/bin/env python3
"""Run one end of a leader/follower gripper teleoperation link.

This is a runnable companion to the teleoperation section of the README. It is
meant to be started once per machine — one process per gripper:

    # Machine A (the leader you push by hand):
    python3 examples/teleop.py --mode master --channel can0

    # Machine B (the follower that copies it):
    python3 examples/teleop.py --mode slave --channel can0 --host 192.168.1.20

Both ends must share ``--grip-id``. The default link is the point-to-point zenoh
transport used by the field teleoperation (``pip install litegrip[zenoh]``): the
leader listens on ``--port``, the follower connects to ``--host``. ``--link udp``
selects plain UDP instead, which carries no authentication or encryption — keep
either on a trusted network. Press Ctrl+C on either end to stop; the gripper
holds its position and does not disable.

This script talks to real hardware. It does not detect an object in the jaws,
and the follower holds its position on a leader dropout rather than going
slack, so it can clamp whatever is between the fingers. Keep a hand on the
power switch.

``--fake-leader`` replaces the leader with a synthetic one, so a **single
gripper** can be benched alone — no second gripper, no second machine, no
network.  The frames go over an in-process bus, and the synthetic leader sweeps
its opening open → closed → open so the follower can be driven into a hard stop
and its torque guard (``--torque-limit``) watched tripping and re-arming::

    python3 examples/teleop.py --mode slave --channel can0 \
        --fake-leader --torque-limit 1.0

⚠ Put a **rigid object** between the jaws first.  With nothing to press against,
the follower closes freely, its torque never rises, and there is nothing for the
guard to demonstrate.  ``--torque-limit 0`` (the default) turns the guard off.

It runs from a source checkout as well as from an installed package: ``src/`` is
put on the import path below if ``litegrip`` is not installed yet.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

# Like tests/_sdkpath.py: import the SDK straight out of the checkout, so the
# example works without `pip install -e .`. Inserted first, so the checkout wins
# over an installed copy — running the example exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import (DEFAULT_ALIGN_SPEED_MM_S, DEFAULT_DQ_MAX,  # noqa: E402
                      DEFAULT_GRIP_ID, DEFAULT_GRIP_PORT, DEFAULT_LEAD_CAP_MM,
                      DEFAULT_TORQUE_LIMIT_NM, InProcTeleopTransport, LiteGrip,
                      LiteGripError, encode_frame, teleop_topic)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one end of a LiteGrip leader/follower teleop link.")
    parser.add_argument(
        "--mode", required=True, choices=("master", "slave"),
        help="master = the leader you push by hand; slave = the follower")
    parser.add_argument(
        "--channel", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--can-id", type=lambda s: int(s, 0), default=0x08,
        help="motor CAN ID (default: 0x08)")
    parser.add_argument(
        "--link", choices=("zenoh", "udp"), default="zenoh",
        help="transport (default: zenoh; udp is the plain-LAN fallback)")
    parser.add_argument(
        "--host", default=None,
        help="slave: the leader's address (the master only listens)")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_GRIP_PORT,
        help=f"TCP (zenoh) or UDP port (default: {DEFAULT_GRIP_PORT})")
    parser.add_argument(
        "--grip-id", default=DEFAULT_GRIP_ID,
        help=f"topic id both ends must agree on (default: {DEFAULT_GRIP_ID})")
    parser.add_argument(
        "--mount", choices=("normal", "reverse"), default=None,
        help="load a mount template instead of this channel's calibration")
    parser.add_argument(
        "--kp", type=float, default=None,
        help="follower stiffness (default: the calibration's kp)")
    parser.add_argument(
        "--kd", type=float, default=None,
        help="follower damping (default: the calibration's kd)")
    parser.add_argument(
        "--no-align", action="store_true",
        help="follower: skip the one-shot align to the first frame")
    parser.add_argument(
        "--align-speed", type=float, default=DEFAULT_ALIGN_SPEED_MM_S,
        help="follower: speed of the align move in mm/s of jaw travel "
             f"(default: {DEFAULT_ALIGN_SPEED_MM_S:g})")
    parser.add_argument(
        "--watchdog", type=float, default=0.2,
        help="follower: hold position after this many seconds without a "
             "fresh frame (default: 0.2)")
    parser.add_argument(
        "--dq-max", type=float, default=DEFAULT_DQ_MAX,
        help="follower: ceiling in rad/s on the leader velocity fed forward "
             f"(default: {DEFAULT_DQ_MAX:.0f}; 0 disables the feedforward)")
    parser.add_argument(
        "--lead-cap", type=float, default=DEFAULT_LEAD_CAP_MM,
        help="follower: ceiling in mm on how far the align's commanded "
             "position may lead the measured one, which bounds the align "
             f"torque (default: {DEFAULT_LEAD_CAP_MM:g}; 0 disables the cap)")
    parser.add_argument(
        "--rate", type=float, default=50.0, help="loop rate in Hz (default: 50)")
    parser.add_argument(
        "--torque-limit", type=float, default=DEFAULT_TORQUE_LIMIT_NM,
        help="follower: ceiling in Nm on the follower's own torque; held over "
             "it the follower releases in place and re-arms once the leader "
             f"reopens (default: {DEFAULT_TORQUE_LIMIT_NM:g} = guard off)")
    parser.add_argument(
        "--fake-leader", action="store_true",
        help="bench one gripper alone: drive the follower from a synthetic "
             "leader on an in-process bus instead of a second gripper "
             "(implies --mode slave; ignores --link/--host/--port)")
    parser.add_argument(
        "--openness-rate", type=float, default=0.3,
        help="fake leader: how fast its opening sweeps, in openness per second "
             "(default: 0.3, a full stroke in ~3.3 s)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the resolved plan and exit without touching hardware")
    return parser


def _print_status(status: dict) -> None:
    age = status.get("last_frame_age_ms")
    age_txt = "-" if age is None else f"{age:6.1f}"
    openness = status.get("openness")
    open_txt = "-" if openness is None else f"{openness:5.3f}"
    extra = ""
    if status.get("matching") is not None:
        extra += f" matching={str(status['matching']):>5}"
    if status.get("rejected"):
        extra += f" rejected={status['rejected']}"
    if status.get("send_failed"):
        extra += f" send_failed={status['send_failed']}"
    if status.get("fault"):
        extra += f" fault={status['fault']}"
    if status.get("mode") == "slave":
        # The follower's own torque and the guard's state — the whole point of
        # --torque-limit, so print them rather than hide them in the counters.
        extra += f" torque={status.get('torque_nm', 0.0):+5.2f}"
        if status.get("over_torque"):
            extra += f" OVER_TORQUE(trips={status.get('torque_trips', 0)})"
    print(f"frames={status.get('frames', 0):>7} "
          f"age_ms={age_txt} stale={str(status.get('stale', False)):>5} "
          f"openness={open_txt} dq={status.get('dq_cmd', 0.0):+5.2f} "
          f"loop_hz={status.get('loop_hz', 0.0):4.1f}"
          f"{extra}", flush=True)


class _FakeLeader:
    """A synthetic leader, so one gripper can be benched without a second one.

    Teleoperation needs a leader *and* a follower — two grippers, or two
    machines.  This stands in for the leader and publishes on an
    :class:`~litegrip.InProcTeleopTransport`, so a single gripper can be run as
    the follower.  Drive it into a hard stop and its torque guard can be
    watched tripping and re-arming, with no second gripper and no network.

    The opening sweeps as a triangle, ``start`` → ``stop`` → ``start``: down to
    the closed stop (the follower presses, the guard trips and releases), back
    up (the follower re-arms once the leader has reopened), and down again.
    """

    def __init__(self, bus, topic: str, travel_mm: float, rate_hz: float,
                 openness_rate: float, start: float = 1.0,
                 stop: float = 0.0) -> None:
        self._bus = bus
        self._topic = topic
        self._travel_mm = travel_mm
        self._dt = 1.0 / rate_hz
        self._step = openness_rate * self._dt
        self._start = start
        self._stop = stop
        self._openness = start
        self._direction = -1.0
        self._abort = threading.Event()
        self._thread = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="fake-leader",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._abort.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._abort.is_set():
            t0 = time.monotonic()
            self._publish()
            # Schedule against the clock: a bare sleep(dt) would add the publish
            # cost every cycle and slow the sweep below --openness-rate.
            time.sleep(max(0.0, self._dt - (time.monotonic() - t0)))

    def _publish(self) -> None:
        # Publish *before* stepping: the first frame is the starting opening, so
        # the follower's align sits on its own position instead of jumping a
        # step away and tripping the guard on the transient.
        # position_mm and force_n are diagnostic only — the follower follows
        # ``openness``.  A synthetic leader has no sensor, so force is honestly
        # 0 and position is the opening scaled by this gripper's own travel.
        self._bus.pub(self._topic, encode_frame(
            self._openness, self._openness * self._travel_mm, 0.0,
            time.time()))
        nxt = self._openness + self._direction * self._step
        if nxt <= self._stop:
            nxt, self._direction = self._stop, 1.0
        elif nxt >= self._start:
            nxt, self._direction = self._start, -1.0
        self._openness = nxt


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.fake_leader and args.mode != "slave":
        print("error: --fake-leader drives a follower, so it needs --mode slave",
              file=sys.stderr)
        return 2
    if args.openness_rate <= 0.0:
        print(f"error: --openness-rate must be > 0, got {args.openness_rate}",
              file=sys.stderr)
        return 2
    if args.torque_limit < 0.0:
        print(f"error: --torque-limit must be >= 0 (0 turns the guard off), "
              f"got {args.torque_limit}", file=sys.stderr)
        return 2
    if args.align_speed <= 0.0:
        print(f"error: --align-speed must be > 0, got {args.align_speed}",
              file=sys.stderr)
        return 2
    if args.lead_cap < 0.0:
        print(f"error: --lead-cap must be >= 0 (0 turns the cap off), "
              f"got {args.lead_cap}", file=sys.stderr)
        return 2
    if args.fake_leader and args.torque_limit == 0.0:
        print("warning: --fake-leader with the torque guard off (--torque-limit 0) "
              "has nothing to demonstrate; pass a limit to watch it trip",
              file=sys.stderr)

    gripper = LiteGrip(channel=args.channel, can_id=args.can_id)
    if args.mount is not None:
        gripper.load_calibration(template=args.mount)
    else:
        gripper.load_calibration()
    print(f"mount={gripper.mount} closed={gripper.config.pos_closed_rad:+.4f} "
          f"open={gripper.config.pos_open_rad:+.4f} rad_to_mm={gripper.config.rad_to_mm}")

    # A fake leader replaces the whole remote end: the frames go over an
    # in-process bus, so none of --link/--host/--port apply.
    bus = InProcTeleopTransport() if args.fake_leader else None
    target = "in-process bus" if bus is not None else (
        f"{args.host}:{args.port}" if args.host else f"*:{args.port}")
    if args.dry_run:
        print(f"dry run: would start {args.mode} on {args.channel} over "
              f"{'inproc' if bus is not None else args.link} at {target} "
              f"(topic litearm/v4/{args.grip_id}/gripper_teleop)")
        return 0

    gripper.connect()
    # enable() does not raise on failure — it returns a falsy EnableResult.
    result = gripper.enable()
    if not result.ok:
        code = None if result.state is None else result.state.error_code
        print(f"error: enable failed after {result.tries} tries "
              f"(last error_code={code}); check the 24V supply and that "
              f"{args.channel} is up at the right bitrate", file=sys.stderr)
        gripper.disconnect()
        return 1

    status = gripper.teleop_start(
        args.mode, transport=bus, link=args.link, host=args.host, port=args.port,
        grip_id=args.grip_id, kp=args.kp, kd=args.kd, align=not args.no_align,
        align_speed_mm_s=args.align_speed, watchdog_s=args.watchdog,
        dq_max=args.dq_max, rate_hz=args.rate,
        torque_limit_nm=args.torque_limit, lead_cap_mm=args.lead_cap)
    if args.mode == "slave":
        guard = "off" if args.torque_limit == 0.0 else f"{args.torque_limit:.2f} Nm"
        print(f"torque guard: {guard} "
              "(follower releases in place when its own torque reaches it)")
        cap = ("off" if args.lead_cap == 0.0
               else f"{args.lead_cap:g} mm")
        print(f"align: {'off' if args.no_align else f'{args.align_speed:g} mm/s'}, "
              f"lead cap: {cap} "
              "(bounds the align's commanded torque; the follow is uncapped)")
    print(f"teleop {args.mode} running; Ctrl+C to stop")
    _print_status(status)

    # Started *after* teleop_start so the follower's subscription exists before
    # the first frame — otherwise the opening sweep is missed and `align` waits
    # out its timeout for a frame that was already sent.
    leader = None
    if bus is not None:
        cfg = gripper.config
        travel = (abs(float(cfg.pos_open_rad) - float(cfg.pos_closed_rad))
                  * float(cfg.rad_to_mm))
        leader = _FakeLeader(bus, teleop_topic(args.grip_id), travel, args.rate,
                             args.openness_rate)
        leader.start()

    try:
        while True:
            time.sleep(1.0)
            _print_status(gripper.teleop_status())
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        if leader is not None:
            leader.stop()
        gripper.teleop_stop()
        gripper.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
