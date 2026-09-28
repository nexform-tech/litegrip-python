"""LiteGrip SDK — high-level gripper API.

The LiteGrip class is the primary entry point.  It manages a single
gripper motor (DM4310 by default) over CAN, exposing an intuitive
open/close/grasp interface with automatic unit conversion.

Example::

    from litegrip import LiteGrip

    with LiteGrip(channel="can0", can_id=0x08) as gripper:
        gripper.load_calibration()
        gripper.enable()          # 反复重试直到状态帧确认 err == 1
        gripper.open()
        gripper.grasp(force_n=20.0, hold_s=3.0)
        state = gripper.get_state()
        print(f"Position: {state.position_mm:.1f} mm, Force: {state.force_n:.1f} N")

``enable()`` / ``disable()`` / ``open()`` / ``close()`` / ``grasp()`` /
``zero()`` 都转发到 :class:`~litegrip.actions.GripperActions`（见
:attr:`LiteGrip.actions`），运动参数在 :attr:`LiteGrip.motion_config` 上。
"""

from __future__ import annotations

import logging
import os as _os
import select as _select_mod
import sys
import time
from datetime import datetime
from typing import Callable, Optional

from .can.motor import MotorType
from .protocols.can_bus import LiteGripCAN
from .actions import (
    EnableResult,
    GraspResult,
    GripperActions,
    MotionConfig,
    MoveProgress,
    MoveResult,
)

# Path to built-in factory calibration (ships with the package, read-only fallback).
_FACTORY_CALIB = _os.path.join(_os.path.dirname(__file__), "factory_calibration.json")

# Default user calibration path — a stable absolute location so that a
# calibration saved without an explicit path is picked up on the next load
# without an explicit path, regardless of the process working directory.
# Override with the LITEGRIP_CALIB env var if desired.
DEFAULT_CALIB = _os.environ.get(
    "LITEGRIP_CALIB",
    _os.path.join(_os.path.expanduser("~"), ".litegrip", "litegrip_calibration.json"),
)
from .models import (
    GripperState,
    GripperConfig,
    GripperInfo,
    GripperStatus,
    CalibrationData,
)
from .constants import (
    GripperParams,
    UnitConversion,
    DefaultParams,
    describe_error,
)
from .exceptions import (
    ConnectError,
    CommError,
    HardwareError,
    NotInitializedError,
)

log = logging.getLogger("litegrip")


class LiteGrip:
    """LiteGrip adaptive two-finger gripper.

    Parameters
    ----------
    channel:
        CAN interface name (``"can0"``, ``"vcan0"``, etc.).
    can_id:
        Motor CAN ID (default 0x08).
    mst_id:
        Motor master ID for status frames.  ``None`` = auto-detect.
    canfd_mode:
        ``True`` to prefer CAN FD.  ``None`` = auto-detect from interface MTU.
    motor_type:
        Damiao motor model (default DM4310).
    config:
        Full GripperConfig for advanced tuning.
    motion_config:
        Motion parameters for the high-level moves (see
        :class:`~litegrip.actions.MotionConfig`).  Also settable at runtime
        via the :attr:`motion_config` property.
    disable_on_disconnect:
        ``True`` (default) → :meth:`disconnect` sends a disable command first,
        so the gripper goes limp when the object is released.  Set ``False``
        to leave the motor enabled after disconnecting.

    The gripper works as a context manager::

        with LiteGrip("can0") as gripper:
            gripper.enable()
            gripper.open()
    """

    def __init__(
        self,
        channel: str = DefaultParams.CAN_CHANNEL,
        can_id: int = GripperParams.CAN_ID,
        mst_id: Optional[int] = None,
        canfd_mode: Optional[bool] = None,
        motor_type: MotorType = MotorType.DM4310,
        config: Optional[GripperConfig] = None,
        motion_config: Optional[MotionConfig] = None,
        disable_on_disconnect: bool = True,
    ):
        cfg = config or GripperConfig()
        self._channel = channel if channel != DefaultParams.CAN_CHANNEL or config is None else cfg.can_channel
        self._can_id = can_id if can_id != GripperParams.CAN_ID or config is None else cfg.can_id
        self._mst_id = mst_id if mst_id is not None else cfg.mst_id
        self._canfd_mode = canfd_mode if canfd_mode is not None else cfg.canfd_mode
        self._motor_type = motor_type
        self._config = cfg
        self._disable_on_disconnect = disable_on_disconnect

        self._can: Optional[LiteGripCAN] = None
        self._enabled = False
        self._connected = False
        self._status_flags = GripperStatus.NONE
        self._actions = GripperActions(self, motion_config)

    # ═══════════════════════════════════════════════════════════════════
    # Properties
    # ═══════════════════════════════════════════════════════════════════

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def can_id(self) -> int:
        return self._can_id

    @property
    def mst_id(self) -> Optional[int]:
        return self._mst_id

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def config(self) -> GripperConfig:
        return self._config

    @property
    def actions(self) -> GripperActions:
        """High-level motion API: ``open`` / ``close`` / ``grasp`` / ``zero`` /
        ``enable`` / ``disable``."""
        return self._actions

    @property
    def motion_config(self) -> MotionConfig:
        """Motion parameters used by the high-level moves.  Settable."""
        return self._actions.config

    @motion_config.setter
    def motion_config(self, value: MotionConfig) -> None:
        self._actions.config = value

    @property
    def disable_on_disconnect(self) -> bool:
        return self._disable_on_disconnect

    @disable_on_disconnect.setter
    def disable_on_disconnect(self, value: bool) -> None:
        self._disable_on_disconnect = value

    # ═══════════════════════════════════════════════════════════════════
    # Connection
    # ═══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """Open CAN bus and register the gripper motor.

        If *mst_id* was ``None`` at construction, it is auto-detected here.
        """
        if self._connected:
            return True

        try:
            self._can = LiteGripCAN(
                channel=self._channel,
                canfd_mode=self._canfd_mode,  # None → auto-detect in transport
            )
            self._can.connect()

            # Register motor; mst_id=None triggers auto-detect
            actual_mst = self._can.register_gripper(
                can_id=self._can_id,
                mst_id=self._mst_id,
                motor_type=self._motor_type,
            )
            if self._mst_id is None:
                self._mst_id = actual_mst

            self._connected = True
            self._status_flags = GripperStatus.NONE
            log.info("LiteGrip connected: %s", self)
            return True
        except ConnectError:
            raise
        except Exception as e:
            raise ConnectError(f"连接失败: {e}")

    def disconnect(self) -> None:
        """Close the CAN transport.

        Sends a disable command first unless :attr:`disable_on_disconnect`
        is ``False`` (in which case the motor stays enabled).
        """
        if not self._connected:
            return

        if self._can:
            self._can.disconnect(disable=self._disable_on_disconnect)
            self._can = None

        self._connected = False
        self._enabled = False
        self._status_flags = GripperStatus.NONE

    # ═══════════════════════════════════════════════════════════════════
    # Enable / disable / fault
    # ═══════════════════════════════════════════════════════════════════

    def enable(self, retries: Optional[int] = None) -> EnableResult:
        """Enable the gripper motor and verify it actually took.

        Retries ``enable`` until a status frame reports ``err == 1`` (真使能)
        — the frame must be read back, because ``enable`` is a one-way CAN
        command and a dropped frame would otherwise go unnoticed.  Real
        faults (err ∉ {0, 1}) are cleared before retrying.

        Args:
            retries: Attempts.  ``None`` = ``MotionConfig.enable_retries``.

        Returns:
            :class:`~litegrip.actions.EnableResult` — truthy when enabled.
        """
        result = self._actions.enable(retries)
        # 以「回读到的状态帧」为准，而不是以某一次 initialize() 的返回值为准
        self._enabled = result.ok
        if result.ok:
            self._status_flags |= GripperStatus.ENABLED
        else:
            self._status_flags &= ~GripperStatus.ENABLED
        return result

    def _enable_once(self) -> bool:
        """One ``enable`` attempt: full init (disable → MIT → enable →
        feedback).  No retry, no state-frame verification — that is
        :meth:`actions.enable` / :meth:`enable`'s job.

        Raises:
            HardwareError: the motor did not end up enabled.
        """
        self._check_connected()

        err = self.get_error()
        if err not in (0, 1):
            self.clear_fault()

        try:
            self._enabled = self._can.initialize()
            if self._enabled:
                self._status_flags |= GripperStatus.ENABLED
            return self._enabled
        except HardwareError:
            raise
        except Exception as e:
            raise HardwareError(f"使能失败: {e}")

    def disable(self) -> bool:
        """Disable the gripper motor."""
        return self._actions.disable()

    def _disable_once(self) -> bool:
        """Send a single disable command and clear the enabled flags."""
        self._check_connected()
        try:
            result = self._can.disable()
            self._enabled = False
            self._status_flags &= ~GripperStatus.ENABLED
            return result
        except Exception:
            self._enabled = False
            return False

    def clear_fault(self) -> bool:
        """Clear latched faults (UV / OC / OT).

        Sequence: disable → clear(0xFB) → enable → verify.
        Retries up to *FAULT_CLEAR_RETRIES* times.
        """
        self._check_connected()
        if self._can is None:
            return False

        cleared = self._can.clear_fault()
        if not cleared:
            err = self._can.get_error()
            raise HardwareError(
                f"故障清除失败: {describe_error(err)} (错误码 0x{err:X})",
                error_code=err,
            )
        self._enabled = True
        self._status_flags |= GripperStatus.ENABLED
        return True

    def stop(self) -> None:
        """Emergency stop — send zero-torque MIT frame.

        Does NOT disable the motor; the motor stays enabled but exerts
        zero torque so it can be back-driven.
        """
        if self._can is not None and self._enabled:
            self._can.control_mit(q_target=0, kp=0, kd=0)
            self._can.update_state(timeout_s=0.02)

    def send_mit_frame(
        self,
        q: float,
        kp: float,
        kd: float,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> bool:
        """Send a single MIT control frame — expert/low-level use.

        For sustained motion use :meth:`goto_rad` or :meth:`move_at_speed`
        instead.  This method is exposed for custom control loops that
        manage their own timing (e.g. force-feedback monitors).

        Args:
            q: Target position in rad.
            kp: Position stiffness.
            kd: Velocity damping.
            dq: Target velocity in rad/s.
            tau: Feed-forward torque in Nm.

        Returns:
            True if the frame was sent.
        """
        if self._can is None or not self._enabled:
            return False
        return self._can.control_mit(
            q_target=q, kp=kp, kd=kd, dq_target=dq, tau_feedforward=tau)

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Poll for one CAN frame and update cached motor state.

        Args:
            timeout_s: Max wait time in seconds (0 = non-blocking).

        Returns:
            True if a status frame for this gripper was received.
        """
        if self._can is None:
            return False
        return self._can.poll(timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Zero-gravity mode (manual back-driving)
    # ═══════════════════════════════════════════════════════════════════

    def enter_zero_gravity(self, duration: float = 0.0) -> None:
        """Enter zero-gravity mode — motor stays enabled but exerts no torque.

        The gripper can be freely moved by hand.  Call :meth:`exit_zero_gravity`
        or any motion command to resume normal control.

        Args:
            duration: Seconds to sustain zero-gravity.  If > 0, blocks for
                      that duration while continuously streaming kp=0, kd=0
                      frames.  If 0 (default), the caller must call
                      :meth:`update_state` or poll manually to sustain the
                      mode (single frame sent as bootstrap).
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return

        if duration > 0:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
                self._can.poll(timeout_s=0.0)
                time.sleep(0.005)
            log.info("Zero-gravity mode ended (duration=%.1fs)", duration)
        else:
            self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
            log.info("Zero-gravity mode: gripper is freely back-drivable "
                     "(caller must poll/sustain)")

    def exit_zero_gravity(self) -> None:
        """Exit zero-gravity mode and hold current position."""
        if self._can is not None and self._enabled:
            current_pos = self._can.get_position()
            self._can.control_mit(q_target=current_pos, kp=self._config.kp,
                                  kd=self._config.kd, tau_feedforward=0)
            log.info("Zero-gravity mode exited; holding position")

    # ═══════════════════════════════════════════════════════════════════
    # Manual calibration (zero-gravity assisted)
    # ═══════════════════════════════════════════════════════════════════

    def calibrate_manual(
        self,
        duration: float = 30.0,
        settle_time: float = 2.0,
        sample_interval: float = 0.01,
    ) -> CalibrationData:
        """Calibrate by manually moving the gripper in zero-gravity mode.

        The motor enters zero-torque mode so you can freely push/pull the
        gripper jaws through their full range.  The SDK records the minimum
        (closed limit) and maximum (open limit) positions reached.

        Usage::

            with LiteGrip("can0") as gripper:
                gripper.enable()
                result = gripper.calibrate_manual(duration=30.0)
                print(f"行程: {result.travel_mm:.1f} mm")

        **Procedure:**

        1. Call this method — the gripper goes limp (zero torque).
        2. Manually push the jaws fully closed, then fully open.
        3. Repeat a few times to ensure limits are captured.
        4. The method returns after *duration* seconds, or press Ctrl+C
           to stop early and keep the best readings so far.

        Args:
            duration: Max recording time in seconds.
            settle_time: Extra seconds at the end to hold position before
                         exiting zero-gravity.
            sample_interval: Polling interval in seconds (~100 Hz default).

        Returns:
            CalibrationData with zero_position, max_position, travel_range,
            and updated rad_to_mm.  The internal :attr:`config` is also updated.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        print("=" * 60)
        print("  零重力手动标定")
        print("=" * 60)
        print()
        print("  夹爪已进入零重力模式，可以自由用手掰动。")
        print()
        print("  操作步骤：")
        print("    1. 将夹爪推到完全闭合位置")
        print("    2. 将夹爪拉到完全张开位置")
        print("    3. 反复推拉几次确保极限被抓到")
        print()
        print(f"  标定将持续 {duration:.0f} 秒（可按 Ctrl+C 提前结束）")
        print("=" * 60)
        print()

        # min_pos = most negative rad value seen  → OPEN  position (largest mm)
        # max_pos = most positive rad value seen  → CLOSED position (0 mm)
        open_rad = float("+inf")   # most negative → open
        close_rad = float("-inf")  # most positive → close
        sample_count = 0

        deadline = time.monotonic() + duration

        try:
            while time.monotonic() < deadline:
                # Stream zero-torque MIT frame
                self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)

                # Poll for latest position
                self._can.poll(timeout_s=0.0)

                pos = self._can.get_position()
                # Only count positions that look like real feedback
                if abs(pos) < 50.0:
                    sample_count += 1
                    if pos < open_rad:
                        open_rad = pos
                        print(f"  ★ 新张开极限: {open_rad:.6f} rad")
                    if pos > close_rad:
                        close_rad = pos
                        print(f"  ★ 新闭合极限: {close_rad:.6f} rad")

                # Progress indicator (every ~1 second)
                remaining = deadline - time.monotonic()
                if sample_count % 100 == 0 and sample_count > 0:
                    print(f"  ... 剩余 {remaining:.0f}s  |  当前 pos={pos:.4f} rad  "
                          f"|  open={open_rad:.4f}  close={close_rad:.4f}")

                time.sleep(sample_interval)

        except KeyboardInterrupt:
            print("\n  ⏎ 用户提前结束标定")

        # ── Settle ──────────────────────────────────────────────────────
        print(f"\n  保持零重力 {settle_time:.0f} 秒（稳定位置）...")
        settle_deadline = time.monotonic() + settle_time
        while time.monotonic() < settle_deadline:
            self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
            self._can.poll(timeout_s=0.0)
            pos = self._can.get_position()
            if pos < open_rad:
                open_rad = pos
            if pos > close_rad:
                close_rad = pos
            time.sleep(sample_interval)

        # ── Exit zero-gravity ───────────────────────────────────────────
        self.exit_zero_gravity()
        time.sleep(0.1)

        # ── Validate ────────────────────────────────────────────────────
        if open_rad >= close_rad or open_rad == float("+inf"):
            raise RuntimeError(
                "标定失败：未能捕获有效的位置范围。"
                "请确保夹爪使能正常且有反馈。"
            )

        travel = close_rad - open_rad  # positive: close(0mm) - open(full travel)
        if travel <= 0:
            raise RuntimeError(
                f"标定失败：行程异常 ({travel:.6f} rad)。"
                "请重新标定。"
            )

        rad_to_mm = self._config.max_stroke_mm / travel if travel > 0 else 105.26

        result = CalibrationData(
            zero_position=round(close_rad, 6),      # closed → 0 mm
            max_position=round(open_rad, 6),        # open → max mm
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config: pos_closed_rad = close (more positive), pos_open_rad = open (more negative)
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm

        print(f"\n{'=' * 60}")
        print(f"  标定完成")
        print(f"{'=' * 60}")
        print(f"  样本数:     {sample_count}")
        print(f"  闭合极限:   {result.zero_position:.6f} rad  (0.0 mm)")
        print(f"  张开极限:   {result.max_position:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  行程:       {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  转换系数:   {result.rad_to_mm:.1f} mm/rad")
        print(f"{'=' * 60}")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # Guided calibration (step-by-step, user confirms each limit)
    # ═══════════════════════════════════════════════════════════════════

    def calibrate_guided(
        self,
        kp: float = 60.0,
        kd: float = 2.0,
        step_rad: float = 0.08,
        stall_delta: float = 0.0004,
        stall_cycles: int = 6,
        max_iter: int = 40,
    ) -> CalibrationData:
        """Guided two-step calibration with user confirmation at each limit.

        The gripper moves itself (low stiffness) toward each limit.  You
        press Enter when the limit is reached.

        **Procedure:**

        1. Gripper steps toward the OPEN direction.  Watch the position.
           Press Enter when fully open (or when it stalls at the hard stop).
        2. Gripper steps toward the CLOSE direction.  Press Enter when
           fully closed.
        3. Calibration is saved to :attr:`config` automatically.

        Args:
            kp: Probing stiffness (low = gentle).
            kd: Probing damping.
            step_rad: Step size per iteration (rad).
            stall_delta: Position delta for auto-stall detection (rad).
            stall_cycles: Consecutive stalls to auto-confirm limit.
            max_iter: Max steps per direction.

        Returns:
            CalibrationData; :attr:`config` is updated in-place.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        print("=" * 60)
        print("  引导式标定")
        print("=" * 60)
        print()
        print("  夹爪将自动缓慢移动。到达极限时按 Enter 确认。")
        print()

        def _step_to_limit(direction: str, sign: float, label: str) -> float:
            """Step in *direction* until user presses Enter or stall."""
            print(f"  [{direction}] 正在{label}...")
            print(f"    按 Enter 确认到达{label}，或等待自动检测堵转")
            print()

            self._can.update_state(timeout_s=0.05)
            current = self._can.get_position()
            stall = 0

            for i in range(max_iter):
                target = current + sign * step_rad
                self._can.control_mit_stream(
                    target, kp, kd, duration_s=0.3, interval_s=0.005)
                self._can.update_state(timeout_s=0.1)

                new_pos = self._can.get_position()
                delta = abs(new_pos - current)

                # Non-blocking keyboard check
                hit_enter = False
                if sys.stdin.isatty():
                    r, _, _ = _select_mod.select([sys.stdin], [], [], 0)
                    if r:
                        sys.stdin.readline()
                        hit_enter = True

                print(f"    [{i}] pos={new_pos:.4f} rad  d={delta:.5f}  "
                      f"stall={stall}", end="")
                if hit_enter:
                    print("  ← 用户确认")
                    return new_pos

                if delta < stall_delta:
                    stall += 1
                    print(f"  (堵转检测中)")
                    if stall >= stall_cycles:
                        print(f"    → 自动检测到{label}: {new_pos:.6f} rad")
                        return new_pos
                else:
                    stall = 0
                    print()

                current = new_pos

            print(f"    → 安全停止（达到最大步数 {max_iter}）: {current:.4f} rad")
            return current

        # ── Step 1: Open ─────────────────────────────────────────────────
        print("━" * 60)
        print("  第 1 步：张开")
        print("━" * 60)
        open_rad = _step_to_limit("open", sign=-1.0, label="张开极限")

        # Small back-off
        print(f"\n  回退一小段...")
        self._can.control_mit_stream(open_rad - 0.15, kp=80, kd=kd,
                                     duration_s=0.5, interval_s=0.005)
        time.sleep(0.1)

        # ── Step 2: Close ────────────────────────────────────────────────
        print(f"\n{'━' * 60}")
        print("  第 2 步：闭合")
        print("━" * 60)
        close_rad = _step_to_limit("close", sign=+1.0, label="闭合极限")

        # ── Compute ──────────────────────────────────────────────────────
        travel = close_rad - open_rad  # close > open (rad value)
        if travel <= 0:
            raise RuntimeError(f"行程异常: close={close_rad:.4f} <= open={open_rad:.4f}")

        rad_to_mm = 120.0 / travel

        result = CalibrationData(
            zero_position=round(close_rad, 6),
            max_position=round(open_rad, 6),
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm

        print(f"\n{'=' * 60}")
        print(f"  标定完成")
        print(f"{'=' * 60}")
        print(f"  闭合(0mm):  {result.zero_position:.6f} rad")
        print(f"  张开(120mm): {result.max_position:.6f} rad")
        print(f"  行程:        {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  转换系数:    {result.rad_to_mm:.1f} mm/rad")
        print(f"{'=' * 60}")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # Calibration persistence
    # ═══════════════════════════════════════════════════════════════════

    def save_calibration(self, path: Optional[str] = None) -> str:
        """Save current calibration and settings to a JSON file.

        Args:
            path: Destination file. When omitted, saves to the default user
                  calibration path (:data:`DEFAULT_CALIB`), which is the same
                  location :meth:`load_calibration` reads by default — so a
                  calibration saved here is picked up automatically next run.
                  The factory calibration shipped with the SDK is never
                  overwritten; back it up separately if needed.

        Returns:
            The absolute path the calibration was written to.
        """
        import json
        if path is None:
            path = DEFAULT_CALIB
        data = {
            "channel": self._channel,
            "can_id": self._can_id,
            "mst_id": self._mst_id or 0,
            "canfd_mode": self._canfd_mode or False,
            "zero_position_rad": self._config.pos_closed_rad,
            "max_position_rad": self._config.pos_open_rad,
            "travel_range_rad": abs(self._config.pos_open_rad - self._config.pos_closed_rad),
            "rad_to_mm": self._config.rad_to_mm,
            "motor_type": self._motor_type.name,
            "kp": self._config.kp,
            "kd": self._config.kd,
            "grasp_torque_threshold": self._config.grasp_torque_threshold,
        }
        parent = _os.path.dirname(path)
        if parent:
            _os.makedirs(parent, exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        log.info("Calibration saved to %s", path)
        return path

    def load_calibration(self, path: Optional[str] = None) -> bool:
        """Load calibration from a JSON file into :attr:`config`.

        Tries *path* first (typically a user calibration from a previous
        run).  If that file does not exist, falls back to the built-in
        factory calibration shipped with the SDK.

        Call this after :meth:`connect` but before :meth:`enable`.

        Args:
            path: JSON file path. When omitted, reads the default user
                  calibration path (:data:`DEFAULT_CALIB`) — the same location
                  :meth:`save_calibration` writes to by default. Falls back to
                  the read-only factory calibration if neither exists.

        Returns:
            True if loaded successfully (from either source).
        """
        import json

        if path is None:
            path = DEFAULT_CALIB

        # Try user file first, then factory fallback
        sources = [path, _FACTORY_CALIB]
        loaded_from: str = ""
        for src in sources:
            try:
                with open(src, "r") as f:
                    data = json.load(f)
                log.info("Calibration loaded from %s", src)
                loaded_from = src
                break
            except (FileNotFoundError, json.JSONDecodeError):
                continue
        if not loaded_from:
            log.warning("No calibration found (tried: %s). "
                        "Run calibrate_manual.py first.", ", ".join(sources))
            return False

        self._config.pos_closed_rad = float(data["zero_position_rad"])
        self._config.pos_open_rad = float(data["max_position_rad"])
        self._config.rad_to_mm = float(data["rad_to_mm"])

        # Optional fields (present in newer calibration files)
        for key, attr in [
            ("can_id", "can_id"),
            ("mst_id", "mst_id"),
            ("channel", "can_channel"),
            ("canfd_mode", "canfd_mode"),
            ("kp", "kp"),
            ("kd", "kd"),
            ("grasp_torque_threshold", "grasp_torque_threshold"),
        ]:
            if key in data:
                setattr(self._config, attr, data[key])

        # Also update instance-level IDs if present
        if "can_id" in data:
            self._can_id = int(data["can_id"])
        if "mst_id" in data:
            self._mst_id = int(data["mst_id"])

        log.info("Calibration loaded from %s: range=%.1f mm",
                 loaded_from,
                 abs(self._config.pos_open_rad - self._config.pos_closed_rad) * self._config.rad_to_mm)
        return True

    # ═══════════════════════════════════════════════════════════════════
    # Motion — high-level
    # ═══════════════════════════════════════════════════════════════════

    def home(self) -> bool:
        """Move to the closed (zero) position."""
        self._check_connected()
        self._check_enabled()
        return self.move_to(GripperParams.POS_CLOSED_RAD, duration=1.0)

    def open(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """Open the gripper fully.

        A continuous ramp (velocity feed-forward, one frame per
        :attr:`MotionConfig.frame_interval`) that drives *past* the calibrated
        open limit and lets the mechanical stop end the move. The command lead
        is narrowed to :attr:`MotionConfig.stop_lead_mm` inside
        :attr:`MotionConfig.press_zone_mm` of the limit, so the pressing
        torque stays around ``kp × stop_lead_mm``.

        Args:
            speed_mm_s: Opening speed; ``None`` = ``MotionConfig.speed_mm_s``.
            progress: Optional callback, called with a
                :class:`~litegrip.actions.MoveProgress` per sample.

        Returns:
            :class:`~litegrip.actions.MoveResult` — truthy when it pressed
            onto the stop (``stalled`` and parked within
            :attr:`MotionConfig.stop_tol` of the limit).  Stalling far from
            the limit means something blocked the travel, and is falsy.
        """
        self._check_connected()
        return self._actions.open(speed_mm_s, progress=progress)

    def close(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """Close the gripper.

        Same ramp as :meth:`open`, pressing onto the closed-side mechanical
        stop.  Use :meth:`grasp` for a power grasp (closing onto an object and
        squeezing) — that one stops on the object, not on the empty stop.

        Args:
            speed_mm_s: Closing speed; ``None`` = ``MotionConfig.speed_mm_s``.
            progress: Optional progress callback.

        Returns:
            :class:`~litegrip.actions.MoveResult`.
        """
        self._check_connected()
        return self._actions.close(speed_mm_s, progress=progress)

    def grasp(
        self,
        force_n: Optional[float] = None,
        hold_s: float = 0.0,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> GraspResult:
        """Adaptive grasp — close until stall, then hold with a set force.

        Blocking.  Closes on a ramp (stall detection catches the object),
        then keeps streaming MIT frames holding the position with a
        feed-forward torque of ``force_n × 0.1`` Nm, re-reading the
        position every ``MotionConfig.hold_interval``.

        Args:
            force_n: Gripping force in N; ``None`` = ``MotionConfig.force_n``.
            hold_s: Hold time in seconds; ``0`` = hold until a fault or
                Ctrl+C.
            progress: Optional progress callback.

        Returns:
            :class:`~litegrip.actions.GraspResult` — truthy when the hold
            ended normally.  ``stalled=True`` with ``reached=False`` means
            it closed onto an object, which is the expected outcome.
        """
        self._check_connected()
        return self._actions.grasp(force_n, hold_s, progress=progress)

    def zero(self) -> CalibrationData:
        """Full calibration: probe both mechanical limits and save.

        The gripper is driven against each end stop with low stiffness.
        Make sure the travel is clear.  Writes the result to the default
        user calibration path (see :meth:`save_calibration`).

        Returns:
            :class:`CalibrationData`.
        """
        self._check_connected()
        self._check_enabled()
        return self._actions.zero()

    # ═══════════════════════════════════════════════════════════════════
    # Motion — mid-level
    # ═══════════════════════════════════════════════════════════════════

    def goto(
        self,
        position_mm: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        duration: float = 0.5,
    ) -> bool:
        """Move to an absolute position in millimetres."""
        self._check_connected()
        self._check_enabled()
        # Motor rad decreases toward open → target_rad = close_rad - mm / scale
        position_rad = self._config.pos_closed_rad - position_mm / self._config.rad_to_mm
        return self.goto_rad(position_rad, kp=kp, kd=kd, duration=duration)

    def goto_rad(
        self,
        position_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        dq_target: float = 0.0,
        tau_feedforward: float = 0.0,
        duration: float = 0.5,
    ) -> bool:
        """Move to an absolute position in radians (streams MIT frames)."""
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return False

        kp = kp if kp is not None else self._config.kp
        kd = kd if kd is not None else self._config.kd

        # Clamp: open_rad (more negative) ≤ pos ≤ closed_rad (more positive)
        position_rad = max(self._config.pos_open_rad,
                          min(self._config.pos_closed_rad, position_rad))

        try:
            return self._can.control_mit_stream(
                q_target=position_rad,
                kp=kp, kd=kd,
                duration_s=duration,
                dq_target=dq_target,
                tau_feedforward=tau_feedforward,
            )
        except Exception as e:
            raise CommError(f"位置控制失败: {e}")

    def move_to(
        self,
        target_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        tau_feedforward: float = 0.0,
        duration: float = 1.0,
    ) -> bool:
        """Sustained move to a target position (longer default duration)."""
        return self.goto_rad(target_rad, kp=kp, kd=kd,
                             tau_feedforward=tau_feedforward,
                             duration=duration)

    def set_force(self, force_n: float, duration: float = 0.3) -> bool:
        """Apply a gripping force at the current position.

        Sends MIT frames with feed-forward torque while holding position.

        Args:
            force_n: Target force in newtons.
            duration: Hold time in seconds.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return False

        tau_nm = force_n * UnitConversion.N_TO_NM
        current_pos = self._can.get_position()

        try:
            return self._can.control_mit_stream(
                q_target=current_pos,
                kp=150.0, kd=2.0,
                duration_s=duration,
                tau_feedforward=tau_nm,
            )
        except Exception as e:
            raise CommError(f"力控失败: {e}")

    # ═══════════════════════════════════════════════════════════════════
    # Velocity-controlled moves
    # ═══════════════════════════════════════════════════════════════════

    def move_at_speed(
        self,
        target_mm: float,
        speed_mm_s: float = 30.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """Move to a target position at a constant linear speed (mm/s).

        Uses MIT mode with velocity feedforward.  The trajectory is a
        linear ramp from current position to *target_mm*.

        Args:
            target_mm: Target position in mm (0=closed, 120=open).
            speed_mm_s: Travel speed in mm/s (default 30).
            kp: Position stiffness (default from config).
            kd: Velocity damping.

        Returns:
            True on success.
        """
        self._check_connected()
        self._check_enabled()

        current_mm = self.get_state().position_mm
        distance_mm = abs(target_mm - current_mm)
        if distance_mm < 0.01 or speed_mm_s <= 0:
            return True

        duration_s = distance_mm / speed_mm_s
        # Convert to rad
        current_rad = self._config.pos_closed_rad - current_mm / self._config.rad_to_mm
        target_rad = self._config.pos_closed_rad - target_mm / self._config.rad_to_mm
        speed_rad_s = speed_mm_s / self._config.rad_to_mm

        return self._move_at_speed_rad(
            current_rad, target_rad, speed_rad_s, duration_s, kp, kd)

    def move_at_speed_rad(
        self,
        target_rad: float,
        speed_rad_s: float = 0.5,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """Move to a target position at a constant motor speed (rad/s).

        Args:
            target_rad: Target position in rad.
            speed_rad_s: Motor speed in rad/s (default 0.5).
            kp: Position stiffness (default from config).
            kd: Velocity damping.

        Returns:
            True on success.
        """
        self._check_connected()
        self._check_enabled()

        current_rad = self.get_position_rad()
        distance_rad = abs(target_rad - current_rad)
        if distance_rad < 0.0001 or speed_rad_s <= 0:
            return True

        duration_s = distance_rad / speed_rad_s
        return self._move_at_speed_rad(
            current_rad, target_rad, speed_rad_s, duration_s, kp, kd)

    def _move_at_speed_rad(
        self,
        start_rad: float,
        target_rad: float,
        speed_rad_s: float,
        duration_s: float,
        kp: Optional[float],
        kd: Optional[float],
    ) -> bool:
        """Core: linear ramp from start to target at constant speed."""
        if self._can is None:
            return False

        kp = kp if kp is not None else self._config.kp
        kd = kd if kd is not None else self._config.kd

        # Clamp target
        lo = min(self._config.pos_closed_rad, self._config.pos_open_rad)
        hi = max(self._config.pos_closed_rad, self._config.pos_open_rad)
        target_rad = max(lo, min(hi, target_rad))

        direction = 1.0 if target_rad > start_rad else -1.0

        interval = 0.005  # 200 Hz
        steps = max(1, int(duration_s / interval))
        actual_interval = duration_s / steps

        log.info("move_at_speed: %.4f → %.4f rad @ %.2f rad/s (%.2f s, %d steps)",
                 start_rad, target_rad, speed_rad_s, duration_s, steps)

        for i in range(steps + 1):
            frac = i / steps
            q = start_rad + (target_rad - start_rad) * frac
            dq = direction * speed_rad_s if i < steps else 0.0

            self._can.control_mit(
                q_target=q, kp=kp, kd=kd,
                dq_target=dq, tau_feedforward=0.0)

            if i < steps:
                self._can.poll(timeout_s=0.0)
                time.sleep(actual_interval)

        # Hold at target briefly
        for _ in range(20):
            self._can.control_mit(
                q_target=target_rad, kp=kp, kd=kd,
                dq_target=0.0, tau_feedforward=0.0)
            time.sleep(0.005)

        return True

    # ═══════════════════════════════════════════════════════════════════
    # Calibration
    # ═══════════════════════════════════════════════════════════════════

    def calibrate(
        self,
        kp: float = 60.0,
        kd: float = 2.0,
        step_rad: float = 0.1,
        stall_delta: float = 0.0003,
        stall_cycles: int = 8,
        max_iter: int = 30,
    ) -> CalibrationData:
        """Calibrate the gripper: find close and open mechanical limits.

        Process:
        1. Back off a small amount from current position
        2. Step toward close direction until stall → zero_position
        3. Back off
        4. Step toward open direction until stall → max_position
        5. Compute travel_range and update internal conversion factors

        Args:
            kp: Probing stiffness (low = gentle).
            kd: Probing damping.
            step_rad: Step size per iteration (rad).
            stall_delta: Position change threshold for stall detection (rad).
            stall_cycles: Consecutive stalls to confirm limit.
            max_iter: Max steps per direction (safety cap).

        Returns:
            CalibrationData with zero/max position, travel range, and
            rad-to-mm conversion.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        init_pos = self._can.get_position()
        self._can.update_state(timeout_s=0.1)
        init_pos = self._can.get_position()
        print(f"标定开始  初始位置: {init_pos:.4f} rad")

        def _find_limit(direction: str) -> float:
            sign = +1.0 if direction == "close" else -1.0
            label = "闭合限位" if direction == "close" else "张开限位"
            print(f"  寻找{label}...")

            self._can.update_state(timeout_s=0.05)
            current = self._can.get_position()
            target = current
            stall = 0

            for i in range(max_iter):
                target += sign * step_rad
                self._can.control_mit_stream(target, kp, kd, duration_s=0.3, interval_s=0.005)
                self._can.update_state(timeout_s=0.1)

                new_pos = self._can.get_position()
                delta = abs(new_pos - current)
                tau = self._can.get_torque()

                print(f"    [{i}] tgt={target:+.3f} pos={new_pos:.4f} "
                      f"d={delta:.5f} tau={tau:+.3f} st={stall}")

                if delta < stall_delta:
                    stall += 1
                    if stall >= stall_cycles:
                        print(f"    → 到达{label}: {new_pos:.6f} rad")
                        return new_pos
                else:
                    stall = 0
                current = new_pos

            print(f"    → 安全停止（达到最大步数 {max_iter}）: {current:.4f} rad")
            return current

        # 1. Safe back-off
        print("  安全回退...")
        self.goto_rad(init_pos + 0.2, kp=80, kd=kd, duration=0.5)
        self._can.update_state(timeout_s=0.1)

        # 2. Find zero (close direction)
        zero_pos = _find_limit("close")

        # 3. Back off
        print("  回退...")
        self.goto_rad(zero_pos + 0.3, kp=80, kd=kd, duration=0.5)
        self._can.update_state(timeout_s=0.1)

        # 4. Find max (open direction)
        max_pos = _find_limit("open")

        # 5. Compute results
        # zero_pos = close (more positive rad), max_pos = open (more negative rad)
        travel = zero_pos - max_pos  # positive rad value
        rad_to_mm = self._config.max_stroke_mm / travel if travel > 0 else 105.26

        result = CalibrationData(
            zero_position=round(zero_pos, 6),      # closed → 0 mm
            max_position=round(max_pos, 6),        # open → max mm
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm

        print(f"\n  标定结果:")
        print(f"    闭合(0mm): {result.zero_position:.6f} rad")
        print(f"    张开(120mm): {result.max_position:.6f} rad")
        print(f"    行程:     {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"    转换系数: {result.rad_to_mm:.1f} mm/rad")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # State
    # ═══════════════════════════════════════════════════════════════════

    def get_state(self, wait: bool = True) -> GripperState:
        """Return the current gripper state.

        Args:
            wait: If True (default), waits up to 50 ms for a fresh status
                  frame.  If False, returns immediately with the last cached
                  state (suitable for high-frequency control loops).

        Returns:
            GripperState snapshot.
        """
        self._check_connected()
        if self._can is None:
            return GripperState()

        if wait:
            self._can.update_state(timeout_s=0.05)
        else:
            self._can.poll(timeout_s=0.0)

        position_rad = self._can.get_position()
        velocity_rad_s = self._can.get_velocity()
        torque_nm = self._can.get_torque()
        error_code = self._can.get_error()
        t_mos, t_coil = self._can.get_temperature()

        # pos_closed_rad=closed(0mm), pos_open_rad=open(max_mm).
        # Motor rad decreases when opening → mm = (close_rad - current_rad) * scale
        position_mm = (self._config.pos_closed_rad - position_rad) * self._config.rad_to_mm
        force_n = torque_nm * UnitConversion.NM_TO_N

        return GripperState(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            torque_nm=torque_nm,
            temperature_mos=t_mos,
            temperature_coil=t_coil,
            error_code=error_code,
            timestamp=time.time(),
            position_mm=position_mm,
            force_n=force_n,
        )

    def get_position(self) -> float:
        """Current position in mm."""
        return self.get_state().position_mm

    def get_position_rad(self) -> float:
        """Current position in rad."""
        self._check_connected()
        if self._can is None:
            return 0.0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_position()

    def get_force(self) -> float:
        """Estimated gripping force in N."""
        return self.get_state().force_n

    def get_torque(self) -> float:
        """Current motor torque in Nm."""
        self._check_connected()
        if self._can is None:
            return 0.0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_torque()

    def get_error(self) -> int:
        """Motor error code (0=disabled, 1=enabled, 0x9=UV, ...)."""
        self._check_connected()
        if self._can is None:
            return -1
        self._can.update_state(timeout_s=0.05)
        return self._can.get_error()

    def get_temperature(self) -> tuple[int, int]:
        """Return (MOS_temp, Coil_temp) in °C."""
        self._check_connected()
        if self._can is None:
            return 0, 0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_temperature()

    def get_info(self) -> GripperInfo:
        """Return device metadata."""
        return GripperInfo(
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
        )

    # ═══════════════════════════════════════════════════════════════════
    # State predicates
    # ═══════════════════════════════════════════════════════════════════

    def is_moving(self) -> bool:
        """True if the gripper is currently in motion."""
        state = self.get_state()
        return state.is_moving

    def is_grasped(self) -> bool:
        """True if torque exceeds the grasp-detection threshold."""
        state = self.get_state()
        return abs(state.torque_nm) > self._config.grasp_torque_threshold

    def wait_for_ready(self, timeout: float = 5.0) -> bool:
        """Block until the motor is enabled and not moving."""
        if self._can is None:
            return False
        start = time.time()
        while time.time() - start < timeout:
            self._can.update_state(timeout_s=0.05)
            err = self._can.get_error()
            if err == 1 and not self.is_moving():
                return True
            time.sleep(0.05)
        return False

    # ═══════════════════════════════════════════════════════════════════
    # Parameter access (expert)
    # ═══════════════════════════════════════════════════════════════════

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        """Read a motor register by its RID (expert use).

        See :class:`litegrip.can.protocol.DM_REG` for available registers.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")
        return self._can.read_param(rid, timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Internal
    # ═══════════════════════════════════════════════════════════════════

    def _check_connected(self) -> None:
        if not self._connected:
            raise NotInitializedError("未连接 — 请先调用 connect() 或使用 with 上下文")

    def _check_enabled(self) -> None:
        if not self._enabled:
            raise NotInitializedError("未使能 — 请先调用 enable()")

    # ═══════════════════════════════════════════════════════════════════
    # Context manager
    # ═══════════════════════════════════════════════════════════════════

    def __enter__(self) -> "LiteGrip":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        status = "已连接" if self._connected else "未连接"
        enabled = "已使能" if self._enabled else "未使能"
        mst = f"0x{self._mst_id:02X}" if self._mst_id else "auto"
        return (f"LiteGrip(ch={self._channel}, CAN_ID=0x{self._can_id:02X}, "
                f"MST_ID={mst}, {status}, {enabled})")

    def __str__(self) -> str:
        return self.__repr__()
