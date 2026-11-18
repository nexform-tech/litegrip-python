#!/usr/bin/env python3
"""Interactive smoke test for the common LiteGrip SDK calls.

This is the bench script to reach for when you want to exercise the high-level
API one call at a time — read state, enable, open, close, goto, grasp, release,
stop, disable — instead of running one of the focused tests.  It prints what
each call returned and the state it ended in.

**Read-only by default.**  ``--sequence read`` (the default) only connects,
loads the calibration and prints a status frame.  Any mode that enables or
moves the jaws requires ``--yes``::

    # What does the motor report right now?
    python3 examples/api_test.py --sequence read

    # Open -> goto mid-travel -> grasp 5 N / 2 s -> release 10 mm.
    python3 examples/api_test.py --sequence basic --yes

    # The same calls, one menu entry at a time.
    python3 examples/api_test.py --sequence menu --yes

Safety:

* ``close()`` is a position move to the calibrated closed stop, not a force
  limit.  Use ``grasp()`` to close on an object.
* ``stop()`` sends zero torque but leaves the motor enabled; ``disable()`` is
  what makes the jaws hand-movable.
* Keep a hand on the power switch.  The script disables the motor on exit, so
  anything held will fall.

The script calls the SDK exactly as the menu says and interposes no guard of
its own.  ``grasp(hold_s>0)`` returns with ``released=False`` and leaves the
last hold torque on the bus until ``release()`` or ``disable()``; a position
move issued before that starts against a drive that is still pressing.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any

# Like the other examples: import the SDK straight out of the checkout, so the
# script works without `pip install -e .`.  Inserted first, so the checkout wins
# over an installed copy — running it exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import LiteGrip  # noqa: E402
from litegrip.can import MotorType  # noqa: E402


def _hex(value: str) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactive smoke test for the high-level LiteGrip API. "
                    "Read-only unless --yes is given.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--yes", action="store_true",
                        help="allow enable/motion; without it this script is "
                             "read-only")
    parser.add_argument("--sequence", choices=("read", "basic", "menu"),
                        default="read",
                        help="read=status only; basic=open/goto/grasp/release; "
                             "menu=one call at a time")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=None,
                        help="master/status ID; default auto-detect")
    parser.add_argument("--motor", default="DM4310",
                        choices=[m.name for m in MotorType],
                        help="motor model (default: DM4310)")
    parser.add_argument("--canfd", action="store_true",
                        help="open the transport in CAN-FD mode")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON.  Without it the "
                             "factory calibration is used: a file this machine "
                             "measured is never read unless you name it here "
                             "or in LITEGRIP_CALIB")
    parser.add_argument("--template", default=None,
                        help="calibration template name; see litegrip.list_templates()")
    parser.add_argument("--speed", type=float, default=20.0,
                        help="open/close speed in mm/s")
    parser.add_argument("--force", type=float, default=5.0,
                        help="grasp force in N")
    parser.add_argument("--hold-s", type=float, default=2.0,
                        help="grasp hold time in s; 0 means hold forever")
    parser.add_argument("--goto-mm", type=float, default=None,
                        help="basic mode's goto target; default is mid-travel")
    parser.add_argument("--release-mm", type=float, default=10.0,
                        help="release clearance in mm")
    return parser


def _fresh_state(g: Any) -> Any:
    """Ask for a status frame; retry a few times."""
    last = None
    for _ in range(3):
        g.refresh_status()
        last = g.get_state(wait=True)
        if last.fresh:
            return last
        time.sleep(0.05)
    return last


def _print_config(g: Any) -> None:
    cfg = g.config
    travel = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
    mst = "auto" if g.mst_id is None else f"0x{g.mst_id:02X}"
    print(f"链路: {g.channel}  CAN ID=0x{g.can_id:02X}  MST ID={mst}")
    print(f"校准: calibrated={cfg.calibrated}  "
          f"pos_closed_rad={cfg.pos_closed_rad:+.5f}  "
          f"pos_open_rad={cfg.pos_open_rad:+.5f}")
    print(f"      行程~{travel:.1f} mm  "
          f"rad_to_mm={cfg.rad_to_mm:.3f}  "
          f"work_stroke_mm={cfg.work_stroke_mm:.1f} mm")


def _print_state(label: str, state: Any) -> None:
    age = "None" if state.data_age_s is None else f"{state.data_age_s:.3f}s"
    print(f"{label}: position_rad={state.position_rad:+.5f} "
          f"({state.position_mm:+.2f} mm)  "
          f"velocity_rad_s={state.velocity_rad_s:+.3f}  "
          f"torque_nm={state.torque_nm:+.3f}  "
          f"force_n={state.force_n:+.2f}  "
          f"error_code=0x{state.error_code:X}  "
          f"fresh={state.fresh}  data_age_s={age}")


def _print_move(label: str, result: Any) -> None:
    print(f"{label}: ok={result.ok} reached={result.reached} "
          f"stalled={result.stalled} protected={result.protected} "
          f"steps={result.steps}")
    _print_state("  结束", result.state)


def _print_grasp(label: str, result: Any) -> None:
    print(f"{label}: ok={result.ok} reached={result.reached} "
          f"stalled={result.stalled} no_bite={result.no_bite} "
          f"travel_guard={result.travel_guard} cycles={result.cycles} "
          f"force={result.force_n:.1f} N released={result.released}")
    _print_state("  结束", result.state)


def _print_release(label: str, result: Any) -> None:
    print(f"{label}: ok={result.ok} opened_mm={result.opened_mm:.1f} "
          f"aborted={result.aborted}")
    _print_state("  结束", result.state)


def _open_gripper(args: argparse.Namespace, *, disable_on_exit: bool) -> Any:
    g = LiteGrip(
        channel=args.channel,
        can_id=args.can_id,
        mst_id=args.mst_id,
        canfd_mode=args.canfd or None,
        motor_type=MotorType[args.motor],
        disable_on_disconnect=disable_on_exit,
    )
    # Calibration is a file read, so it happens before connect: the file may
    # carry can_id/mst_id, and connect() is what puts those on the wire.
    if not g.load_calibration(path=args.calib, template=args.template):
        raise SystemExit("错误：没有读到任何校准文件 —— 请传入 "
                         "--calib 或 --template")
    if not g.config.calibrated:
        raise SystemExit("错误：校准文件里 calibrated=false；运动层不会按"
                         "占位限位移动。")
    g.connect()
    _print_config(g)
    return g


def do_read(args: argparse.Namespace) -> int:
    g = _open_gripper(args, disable_on_exit=False)
    try:
        state = _fresh_state(g)
        _print_state("状态", state)
        if not state.fresh:
            print("没有新的状态帧：请检查 can0、CAN ID/MST ID 和电机供电。")
            return 1
        return 0
    finally:
        g.disconnect()


def _enable(g: Any) -> bool:
    result = g.enable()
    print(f"enable: ok={result.ok} tries={result.tries}")
    _print_state("  结束", result.state)
    return bool(result.ok)


def do_basic(args: argparse.Namespace) -> int:
    print("安全：手放在电源开关旁；保持夹爪周围无人。")
    g = _open_gripper(args, disable_on_exit=True)
    try:
        state = _fresh_state(g)
        _print_state("初始", state)
        if not state.fresh:
            print("没有新的状态帧；停止。")
            return 1
        if not _enable(g):
            return 1

        ok = True
        result = g.open(speed_mm_s=args.speed)
        _print_move("open", result)
        ok = ok and bool(result.ok)

        cfg = g.config
        travel = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
        target = args.goto_mm if args.goto_mm is not None else travel / 2.0
        current = g.get_state(wait=False).position_mm
        duration = max(0.5, abs(target - current) / max(args.speed, 1.0))
        result = g.goto(target, duration=duration)
        _print_move(f"goto {target:.1f} mm", result)
        ok = ok and bool(result.ok)

        result = g.grasp(force_n=args.force, hold_s=args.hold_s)
        _print_grasp(f"grasp {args.force:.1f} N / {args.hold_s:.1f} s", result)
        ok = ok and bool(result.ok)

        result = g.release(clearance_mm=args.release_mm)
        _print_release(f"release {args.release_mm:.1f} mm", result)
        ok = ok and bool(result.ok)
        return 0 if ok else 1
    finally:
        try:
            g.stop()
        except Exception:
            pass
        try:
            g.disable()
        except Exception:
            pass
        g.disconnect()


def menu(args: argparse.Namespace) -> int:
    print("安全：手放在电源开关旁；保持夹爪周围无人。")
    g = _open_gripper(args, disable_on_exit=True)
    try:
        while True:
            print("\n" + "=" * 56)
            print("1) get_state()    2) enable()        3) open()")
            print("4) close()        5) goto()          6) grasp()")
            print("7) release()      8) stop()          9) disable()")
            print("0) 退出")
            choice = input("选择: ").strip()
            if choice in ("0", "q", "quit"):
                break
            if choice == "1":
                _print_state("状态", _fresh_state(g))
                continue
            if choice == "2":
                _enable(g)
                continue
            if choice == "9":
                print(f"disable: ok={g.disable()}")
                continue
            if choice not in ("3", "4", "5", "6", "7", "8"):
                print("无效的选项。")
                continue
            if not g.is_enabled:
                print("电机未使能；请先选择 2 (enable())。")
                continue

            # 动作前主动要一帧状态。空闲总线上 SDK 的 get_state() 只回缓存；
            # 夹爪若被手扳过，open/close/goto 会从旧位置规划斜坡。这是刷新，
            # 不是护栏。
            _fresh_state(g)

            if choice == "3":
                _print_move("open", g.open(speed_mm_s=args.speed))
            elif choice == "4":
                print("注意：close() 只做位置移动；要夹住物体请用 grasp()。")
                _print_move("close", g.close(speed_mm_s=args.speed))
            elif choice == "5":
                try:
                    target = float(input("目标位置 (mm): ").strip())
                except ValueError:
                    print("不是数字。")
                    continue
                current = g.get_state(wait=False).position_mm
                duration = max(0.5, abs(target - current) / max(args.speed, 1.0))
                _print_move(f"goto {target:.1f} mm",
                            g.goto(target, duration=duration))
            elif choice == "6":
                try:
                    force = float(input(f"力 (N) [{args.force:g}]: ").strip()
                                  or args.force)
                    hold = float(input(f"保持时间 (s) [{args.hold_s:g}]: ").strip()
                                 or args.hold_s)
                except ValueError:
                    print("不是数字。")
                    continue
                result = g.grasp(force_n=force, hold_s=hold)
                _print_grasp(f"grasp {force:.1f} N / {hold:.1f} s", result)
            elif choice == "7":
                try:
                    clearance = float(
                        input(f"释放间隙 (mm) [{args.release_mm:g}]: ").strip()
                        or args.release_mm)
                except ValueError:
                    print("不是数字。")
                    continue
                result = g.release(clearance_mm=clearance)
                _print_release(f"release {clearance:.1f} mm", result)
            elif choice == "8":
                g.stop()
                print("stop() 已执行：零力矩，电机仍保持使能。")
        return 0
    finally:
        try:
            g.stop()
        except Exception:
            pass
        try:
            g.disable()
        except Exception:
            pass
        g.disconnect()


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.sequence != "read" and not args.yes:
        print("未加 --yes，拒绝使能或运动；只读连接请用 --sequence read。",
              file=sys.stderr)
        return 2
    try:
        if args.sequence == "read":
            return do_read(args)
        if args.sequence == "basic":
            return do_basic(args)
        return menu(args)
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    except Exception as exc:  # noqa: BLE001 - report the real reason
        print(f"\n失败：{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
