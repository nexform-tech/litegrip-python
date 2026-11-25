"""LiteGrip SDK — high-level gripper API.

:class:`Gripper` is the primary entry point.  It manages a single gripper
motor (DM4310) over CAN, exposing open/close, absolute positioning, adaptive
grasping and calibrated feedback.

Example::

    from litegrip import Gripper

    with Gripper(channel="can0") as gripper:
        gripper.enable()          # retries until the status frame says err == 1
        gripper.open()
        gripper.grasp(force_n=20.0, hold_s=3.0)
        print(gripper.get_motor_data())

All motion is blocking.  Commands return ``True`` only when the motion
actually reached its goal; a motion that ran but failed returns ``False`` and
logs the reason.  Invalid arguments or state raise :class:`CommandError` /
:class:`NotInitializedError`; transport and hardware failures raise
:class:`CommError` / :class:`HardwareError`.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from typing import Optional

from .actions import (
    GripperActions,
    MotionConfig,
    MoveResult,
    WriteZeroResult,
    _check_calibrated,
    clamp_to_travel,
)
from .can.bus import CanBus
from .constants import DefaultParams, GripperParams, UnitConversion, describe_error
from .exceptions import (
    CommandError,
    ConnectError,
    HardwareError,
    NotInitializedError,
)
from .models import CalibrationData, GripperConfig, GripperState

log = logging.getLogger("litegrip")

# A cached status frame younger than this is treated as a measurement by
# get_state(wait=True). It makes refresh_status() followed by get_state()
# report fresh instead of spending another 0.5 s waiting for a frame that the
# refresh already consumed.
_FRESH_WINDOW_S = 0.05

# Hard bounds for the single-frame teleoperation primitive. These are stricter
# than the DM4310 frame ranges so a caller cannot stream an unbounded push.
_FOLLOW_KP_MAX = 500.0        # MIT stiffness range
_FOLLOW_KD_MAX = 5.0          # MIT damping range
_FOLLOW_DQ_MAX_RAD_S = 10.0   # teleop velocity feed-forward ceiling
_FOLLOW_TAU_MAX_NM = 3.5      # safety torque ceiling (same as the C++ guard)

#: The calibration the SDK ships for the reference unit.  The constructor falls
#: back to this file when neither ``LITEGRIP_CALIB`` nor the per-channel file
#: exists, so distance commands work out of the box.
_FACTORY_CALIB = os.path.join(os.path.dirname(__file__),
                              "factory_calibration.json")

#: Fields a calibration file must carry.  Everything else is optional.
_REQUIRED_CALIB_KEYS = ("zero_position_rad", "max_position_rad", "rad_to_mm")


def default_calibration_path(channel: str = DefaultParams.CAN_CHANNEL) -> str:
    """Where the calibration for *channel* is saved and loaded from.

    One gripper per channel, one file per channel —
    ``~/.litegrip/<channel>_calibration.json``.  A single shared file would let
    two grippers on one machine (``can0`` / ``can1``, both at CAN ID 0x08)
    overwrite each other's direction and travel.

    This is :meth:`Gripper.save_calibration`'s destination and the second entry
    in :meth:`Gripper.load_calibration`'s search order.  ``HOME`` is read per
    call, so a caller (or a test) can redirect it.
    """
    return os.path.join(os.path.expanduser("~"), ".litegrip",
                        f"{channel}_calibration.json")


def _read_calibration(path: str) -> Optional[dict]:
    """Read one calibration file, mapping every unreadable case to ``None``."""
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        log.debug("Calibration not readable at %s: %s", path, e)
        return None
    return data if isinstance(data, dict) else None


class Gripper:
    """LiteGrip adaptive two-finger gripper.

    Parameters
    ----------
    channel:
        CAN interface name (``"can0"``, ``"vcan0"``, etc.).
    can_id:
        Motor CAN ID (default 0x08).
    mst_id:
        Motor master ID for status frames (default 0x18).
    motion_config:
        Motion parameters for the high-level moves (see
        :class:`~litegrip.actions.MotionConfig`).

    A calibration is loaded in the constructor, in this order: the file named
    by ``LITEGRIP_CALIB``, then ``~/.litegrip/<channel>_calibration.json`` if it
    exists, then the bundled ``factory_calibration.json``.  An explicit
    :meth:`load_calibration` call overrides it.

    The gripper works as a context manager::

        with Gripper("can0") as gripper:
            gripper.enable()
            gripper.open()
    """

    def __init__(
        self,
        channel: str = DefaultParams.CAN_CHANNEL,
        can_id: int = GripperParams.CAN_ID,
        mst_id: int = GripperParams.MST_ID,
        motion_config: Optional[MotionConfig] = None,
    ):
        if not isinstance(channel, str) or not channel:
            raise CommandError(f"channel 必须是非空的 CAN 接口名：{channel!r}")
        for name, value in (("can_id", can_id), ("mst_id", mst_id)):
            if not isinstance(value, int) or not 0 <= value <= 0x7FF:
                raise CommandError(
                    f"{name} 必须是 11 位标准帧 ID（0..0x7FF）：{value!r}")

        self._channel = channel
        self._can_id = can_id
        self._mst_id = mst_id
        self._config = GripperConfig(can_channel=channel, can_id=can_id,
                                     mst_id=mst_id)
        self._can: Optional[CanBus] = None
        self._connected = False
        self._enabled = False
        self._last_send_error: Optional[Exception] = None
        # Reentrant lock around the low-level CAN I/O that a grasp thread and
        # the caller (release, state reads) share.
        self._io_lock = threading.RLock()
        self._actions = GripperActions(self, motion_config)

        # Leader-side haptic rendering. Off until configure_force_feedback()
        # enables it; the defaults mirror the removed teleop module.
        self._feedback_enabled = False
        self._feedback_gain_nm_per_n = 0.10
        self._feedback_threshold_n = 2.0
        self._feedback_max_tau_nm = 1.0
        self._feedback_damping = 0.1
        self._feedback_timeout_s = 0.3

        # A calibration file is just data; nothing here touches the bus.  The
        # factory file sets calibrated=true, so distance commands work before
        # the first site calibration.
        self.load_calibration()

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
    def mst_id(self) -> int:
        return self._mst_id

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def config(self) -> GripperConfig:
        """The active calibration and gains (also updated by :meth:`calibrate`)."""
        return self._config

    @property
    def motion_config(self) -> MotionConfig:
        """Motion parameters used by the high-level moves.  Settable."""
        return self._actions.config

    @motion_config.setter
    def motion_config(self, value: MotionConfig) -> None:
        self._actions.config = value

    @property
    def last_send_error(self) -> Optional[Exception]:
        """Why the last low-level MIT frame did not send, or ``None``.

        The exception the send itself reported — a ``CommError``-family cause
        from the transport, or the ``NotInitializedError`` naming the step that
        was skipped.  The engine reads it to raise the right type instead of one
        fixed message that guesses.
        """
        return self._last_send_error

    # ═══════════════════════════════════════════════════════════════════
    # Lifecycle
    # ═══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """Open the CAN bus and register the single gripper motor.

        Repeated calls return ``True`` without reopening the bus.

        Raises:
            ConnectError: the interface is unavailable or the motor cannot be
                registered.
        """
        if self._connected:
            return True

        can = CanBus(channel=self._channel)
        try:
            can.connect()
            can.register_gripper(can_id=self._can_id, mst_id=self._mst_id)
        except ConnectError:
            try:
                can.disconnect(disable=False)
            except Exception as e:                       # noqa: BLE001
                log.debug("close after failed connect: %s", e)
            raise
        except Exception as e:
            try:
                can.disconnect(disable=False)
            except Exception as close_error:             # noqa: BLE001
                log.debug("close after failed connect: %s", close_error)
            raise ConnectError(f"连接失败: {e}") from e

        self._can = can
        self._connected = True
        log.info("Gripper connected: %s", self)
        return True

    def disconnect(self) -> None:
        """Disable the motor (zero torque), then close the bus.

        Idempotent and non-throwing: a disconnect must not leave the jaws
        pressing or leak the socket.
        """
        self._connected = False
        self._enabled = False
        can, self._can = self._can, None
        if can is None:
            return
        try:
            can.disconnect(disable=True)
        except Exception as e:                           # noqa: BLE001
            log.warning("断开连接时出错（资源仍已释放）：%s", e)

    def enable(self, retries: Optional[int] = None) -> bool:
        """Enable the motor and verify it actually took.

        Full init: disable → MIT mode → enable → hold.  The result is read back
        from a fresh status frame (``err == 1``), because enable is a one-way
        CAN command and a dropped frame would otherwise go unnoticed.  Real
        faults are cleared before retrying.

        Args:
            retries: Attempts; ``None`` uses ``MotionConfig.enable_retries``.

        Returns:
            True only when a status frame reported ``err == 1``.
        """
        result = self._actions.enable(retries)
        self._enabled = result.ok
        if not result.ok:
            state = result.state
            log.warning("使能失败（%d 次尝试，err=%s）", result.tries,
                        "无状态帧" if state is None else state.error_code)
        return result.ok

    def disable(self) -> bool:
        """Disable the motor (torque off).  The jaws become back-drivable."""
        return self._actions.disable()

    def clear_fault(self) -> bool:
        """Clear latched faults (UV / OC / OT) and re-enable.

        Raises:
            HardwareError: the fault could not be cleared.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")
        cleared = self._can.clear_fault()
        if not cleared:
            err = self._can.get_error()
            raise HardwareError(
                f"故障清除失败: {describe_error(err)} (错误码 0x{err:X})",
                error_code=err,
            )
        self._enabled = True
        return True

    def set_zero(self) -> bool:
        """Make the current encoder angle the motor's zero (CMD 0xFE).

        The jaws are left limp at their current angle, that angle is declared
        zero, and the angle is read back to confirm (``|pos| <= 1e-3 rad``).
        Unlike :meth:`calibrate`, this does **not** probe the end stops and
        does not touch the calibration file: it only rewrites the encoder
        offset, so the span already measured stays valid.

        The motor is disabled first because a DM4310 ignores 0xFE while
        enabled, then re-enabled.

        .. warning:: Irreversible — the offset is written to the motor's flash.

        Returns:
            True when the read-back came back ~0.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        # 1. Limp at the current angle, so the jaws stay where the operator put
        #    them, then read the pose we are about to zero.
        before = self._can.get_position()
        self._can.control_mit_stream(q_target=before, kp=0.0, kd=0.0,
                                     duration_s=0.3)
        self._can.update_state(timeout_s=0.3)
        before = self._can.get_position()

        # 2. A DM motor ignores 0xFE while enabled — disable first.
        self.disable()
        time.sleep(0.02)
        if not self._can.set_zero():
            raise HardwareError("0xFE 未能发送（未连接或未注册）")
        time.sleep(0.05)

        # 3. Re-enable, force fresh frames, read the new zero back.
        enabled = self.enable()
        self._can.control_mit_stream(q_target=0.0, kp=0.0, kd=0.0,
                                     duration_s=0.2)
        self._can.update_state(timeout_s=0.3)
        after = self._can.get_position()
        result = WriteZeroResult(before_rad=before, after_rad=after)
        if not enabled:
            log.warning("置零后重新使能失败 —— 夹爪留在失能态，set_zero 返回 False")
            return False
        if not result.ok:
            log.warning(
                "置零回读不是 0：写入前 %.6f rad，重新使能后 %.6f rad"
                "（容差 %.0e rad）—— 电机可能忽略了 0xFE",
                before, after, result.tolerance_rad)
        return result.ok

    def calibrate(self, save: bool = True) -> CalibrationData:
        """Probe both mechanical limits with low stiffness and save the result.

        Blocking; the jaws move against each end stop.  **Make sure the travel
        is clear first.**  The direction comes from the existing calibration's
        limit ordering (both stops look the same to a stall detector), and by
        default the result is written to the per-channel file
        (:func:`default_calibration_path`).  Pass ``save=False`` to update the
        in-memory calibration only; ``examples/calibrate.py`` uses that with
        ``--save-to`` so the default file is not rewritten.

        Returns:
            :class:`~litegrip.models.CalibrationData` with the measured limits,
            travel and conversion factor.
        """
        self._check_connected()
        self._check_enabled()
        data = self._actions.calibrate()
        if save:
            path = self.save_calibration()
            log.info("Calibration saved to %s", path)
        return data

    # ═══════════════════════════════════════════════════════════════════
    # Calibration persistence
    # ═══════════════════════════════════════════════════════════════════

    def load_calibration(self, path: Optional[str] = None) -> bool:
        """Load calibration into :attr:`config`.

        Search order when *path* is omitted: ``LITEGRIP_CALIB`` if set, then
        ``~/.litegrip/<channel>_calibration.json``, then the bundled factory
        file.  An explicit *path* is the only source tried.  The first readable
        file wins.

        A file that names another ``channel`` is loaded with a warning — the
        channel is the identity key when two grippers share CAN ID 0x08.

        Returns:
            True if a file was loaded, False when nothing readable was found
            (formatted JSON errors count as unreadable).

        Raises:
            CommandError: the file is readable but not a valid calibration
                (missing required fields, or non-finite values).
        """
        if path is not None:
            sources = [os.fspath(path)]
        else:
            env = os.environ.get("LITEGRIP_CALIB")
            sources = [s for s in (env, default_calibration_path(self._channel),
                                   _FACTORY_CALIB) if s]

        seen = set()
        data = None
        loaded_from = ""
        for src in sources:
            if src in seen:
                continue
            seen.add(src)
            candidate = _read_calibration(src)
            if candidate is not None:
                data = candidate
                loaded_from = src
                break

        if data is None:
            log.warning("No calibration found (tried: %s). "
                        "Run calibrate() and save_calibration(), or point "
                        "LITEGRIP_CALIB at a file.", ", ".join(sources))
            return False

        missing = [k for k in _REQUIRED_CALIB_KEYS if k not in data]
        if missing:
            raise CommandError(
                f"标定文件 {loaded_from} 缺少必需字段：{', '.join(missing)}")
        try:
            closed = float(data["zero_position_rad"])
            opened = float(data["max_position_rad"])
            rad_to_mm = float(data["rad_to_mm"])
        except (TypeError, ValueError) as e:
            raise CommandError(
                f"标定文件 {loaded_from} 的限位/换算字段不是数字：{e}") from None
        if not all(math.isfinite(v) for v in (closed, opened, rad_to_mm)):
            raise CommandError(
                f"标定文件 {loaded_from} 含非有限数值 —— 拒绝加载")
        if rad_to_mm == 0.0:
            raise CommandError(
                f"标定文件 {loaded_from} 的 rad_to_mm=0 —— 无法换算开口量")

        self._config.pos_closed_rad = closed
        self._config.pos_open_rad = opened
        self._config.rad_to_mm = rad_to_mm
        for key in ("kp", "kd", "work_stroke_mm"):
            if key in data:
                try:
                    value = float(data[key])
                except (TypeError, ValueError):
                    raise CommandError(
                        f"标定文件 {loaded_from} 的 {key} 不是数字："
                        f"{data[key]!r}") from None
                if not math.isfinite(value):
                    raise CommandError(
                        f"标定文件 {loaded_from} 的 {key} 不是有限数")
                setattr(self._config, key, value)

        # Older calibration files predate the flag and always came from a real
        # calibration run, so absence means calibrated.
        self._config.calibrated = bool(data.get("calibrated", True))

        file_channel = data.get("channel")
        # The bundled factory file names can0 because that is the reference
        # unit's channel; using it on can1 is the documented fallback, not a
        # mistake worth warning about.  A user file that names another channel
        # is, so that one still speaks up.
        if (file_channel and file_channel != self._channel
                and loaded_from != _FACTORY_CALIB):
            log.warning(
                "标定文件的 channel=%s 与本实例的 %s 不一致 —— 同一台电脑上"
                "多台夹爪共用 CAN ID 时，通道是唯一身份键，确认没有指错文件。",
                file_channel, self._channel)

        s = self._config.close_sign
        log.info("Calibration loaded from %s: range=%.1f mm，方向=%s",
                 loaded_from,
                 abs(self._config.pos_open_rad - self._config.pos_closed_rad)
                 * self._config.rad_to_mm,
                 "闭合时 rad 增大" if s > 0 else "闭合时 rad 减小")
        return True

    def save_calibration(self, path: Optional[str] = None) -> str:
        """Write the active calibration to a JSON file.

        Args:
            path: Destination; defaults to
                ``~/.litegrip/<channel>_calibration.json`` (one file per
                channel, so two grippers on one machine do not overwrite each
                other).

        Returns:
            The path written.
        """
        if path is None:
            path = default_calibration_path(self._channel)
        path = os.fspath(path)
        data = {
            "channel": self._channel,
            "can_id": self._can_id,
            "mst_id": self._mst_id,
            "calibrated": bool(self._config.calibrated),
            # Which of zero/max is numerically larger carries the mounting
            # direction; there is no separate field for it.
            "zero_position_rad": self._config.pos_closed_rad,
            "max_position_rad": self._config.pos_open_rad,
            "travel_range_rad": abs(self._config.pos_open_rad
                                    - self._config.pos_closed_rad),
            "rad_to_mm": self._config.rad_to_mm,
            "work_stroke_mm": self._config.work_stroke_mm,
            "kp": self._config.kp,
            "kd": self._config.kd,
        }
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        log.info("Calibration saved to %s", path)
        return path

    # ═══════════════════════════════════════════════════════════════════
    # Motion
    # ═══════════════════════════════════════════════════════════════════

    def open(self, speed_mm_s: Optional[float] = None) -> bool:
        """Open the gripper toward the calibrated open limit.

        A continuous ramp; the command is capped at the calibrated travel, so
        the jaws can never be driven past it.  If ``work_stroke_mm`` is set and
        smaller than the travel, the move stops there instead of pressing the
        limit.

        Args:
            speed_mm_s: Opening speed; ``None`` uses ``MotionConfig.speed_mm_s``.

        Returns:
            True when the jaws pressed and parked near the calibrated open
            limit (or reached the work stroke) without tripping the stall
            guard.  False otherwise; the reason is logged.
        """
        self._check_connected()
        result = self._actions.open(speed_mm_s)
        self._report_move("open", result)
        return result.ok

    def close(self, speed_mm_s: Optional[float] = None) -> bool:
        """Close the gripper toward the calibrated closed limit.

        A continuous ramp with the command capped at the calibrated travel; the
        jaws park on the closed limit and press lightly.  Use :meth:`grasp` to
        close onto an object with a set force.

        Args:
            speed_mm_s: Closing speed; ``None`` uses ``MotionConfig.speed_mm_s``.

        Returns:
            True when the jaws pressed and parked near the calibrated closed
            limit.  False otherwise; the reason is logged.
        """
        self._check_connected()
        result = self._actions.close(speed_mm_s)
        self._report_move("close", result)
        return result.ok

    def set_gripper_distance(self, distance_mm: float,
                             speed_mm_s: Optional[float] = None) -> bool:
        """Move to an absolute opening in millimetres (0 = fully closed).

        Normal positioning between the calibrated limits: no limit press, and
        the target is clamped into the calibrated travel.

        Args:
            distance_mm: Opening from the closed zero, in mm.
            speed_mm_s: Travel speed; ``None`` uses
                ``MotionConfig.speed_mm_s``.

        Returns:
            True when the jaws reached the target and did not stall.

        Raises:
            CommandError: ``distance_mm`` is not finite, or the configuration
                is uncalibrated.
        """
        return self._set_position(distance_mm, speed_mm_s, distance=True)

    def set_motor_angle(self, angle_rad: float,
                        speed_mm_s: Optional[float] = None) -> bool:
        """Move to an absolute motor angle in radians.

        Normal positioning between the calibrated limits: no limit press, and
        the target is clamped into the calibrated travel.

        Args:
            angle_rad: Target motor angle in rad.
            speed_mm_s: Travel speed; ``None`` uses
                ``MotionConfig.speed_mm_s``.

        Returns:
            True when the jaws reached the target and did not stall.

        Raises:
            CommandError: ``angle_rad`` is not finite, or the configuration is
                uncalibrated.
        """
        return self._set_position(angle_rad, speed_mm_s, distance=False)

    def follow_gripper_distance(
        self,
        distance_mm: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        dq: float = 0.0,
        tau: float = 0.0,
        force_feedback: Optional[Callable[[float], None]] = None,
    ) -> bool:
        """Send one MIT follow frame for teleoperation, in millimetres.

        This is the streaming primitive the teleoperation loop calls at its own
        cadence (typically 100–200 Hz), one frame per call.  It does **not**
        ramp, settle, wait for motion, or read the bus.  The caller owns the
        loop, the leader-side reading and the watchdog.

        ``distance_mm`` is an absolute opening from the closed zero and is
        interpreted with **this** gripper's calibration, so a leader may send
        its own :meth:`get_gripper_distance` even when the two calibration
        files differ in zero, direction or scale.  A follower with a shorter
        physical stroke still clamps at its own travel; for different strokes
        use a normalised opening in the caller.

        The target is clamped into the calibrated travel, exactly like the
        positioning commands.  The frame is refused before any bus write when
        a value is non-finite or out of range.

        Args:
            distance_mm: Target opening from the closed zero in mm (0 = closed).
            kp: Position stiffness; ``None`` uses ``config.kp``.
            kd: Velocity damping; ``None`` uses ``config.kd``.
            dq: Velocity feed-forward in rad/s, ``|dq| <= 10``.
            tau: Torque feed-forward in Nm, ``|tau| <= 3.5``.
            force_feedback: Optional callback called with the follower's
                measured squeeze (N) after the frame is sent.  ``None``
                (default) disables it.  The callback normally forwards to the
                leader's :meth:`render_force_feedback`; it runs on the caller's
                thread and its exceptions propagate.

        Returns:
            True when the frame was sent.  A ``False`` return leaves the
            transport cause in :attr:`last_send_error`; it does not mean the
            motor accepted the frame — the bus is one-way for commands.

        Raises:
            NotInitializedError: not connected or not enabled.
            CommandError: uncalibrated, a non-finite value, or a value outside
                the bounds above.

        Example::

            while running:
                leader_mm = leader.get_gripper_distance()   # or a Pika Sense
                follower.follow_gripper_distance(leader_mm)
                time.sleep(0.005)
        """
        self._check_connected()
        self._check_enabled()
        _check_calibrated(self._config)

        kp_v = self._config.kp if kp is None else float(kp)
        kd_v = self._config.kd if kd is None else float(kd)
        dq_v = float(dq)
        tau_v = float(tau)

        values = (("distance_mm", float(distance_mm)), ("kp", kp_v),
                  ("kd", kd_v), ("dq", dq_v), ("tau", tau_v))
        for name, value in values:
            if not math.isfinite(value):
                raise CommandError(
                    f"follow_gripper_distance 的 {name} 不是有限数：{value!r}")
        if not 0.0 <= kp_v <= _FOLLOW_KP_MAX:
            raise CommandError(
                f"follow_gripper_distance 的 kp 必须在 "
                f"0..{_FOLLOW_KP_MAX}：{kp_v!r}")
        if not 0.0 <= kd_v <= _FOLLOW_KD_MAX:
            raise CommandError(
                f"follow_gripper_distance 的 kd 必须在 "
                f"0..{_FOLLOW_KD_MAX}：{kd_v!r}")
        if abs(dq_v) > _FOLLOW_DQ_MAX_RAD_S:
            raise CommandError(
                f"follow_gripper_distance 的 |dq| 不能超过 "
                f"{_FOLLOW_DQ_MAX_RAD_S} rad/s：{dq_v!r}")
        if abs(tau_v) > _FOLLOW_TAU_MAX_NM:
            raise CommandError(
                f"follow_gripper_distance 的 |tau| 不能超过 "
                f"{_FOLLOW_TAU_MAX_NM} Nm：{tau_v!r}")

        target_rad = (self._config.pos_closed_rad
                      - self._config.close_sign * float(distance_mm)
                      / self._config.rad_to_mm)
        target = clamp_to_travel(self._config, target_rad)
        sent = self._send_mit_frame(target, kp_v, kd_v, dq_v, tau_v)
        if sent and force_feedback is not None:
            force_feedback(self.get_gripper_force())
        return sent

    def get_gripper_force(self) -> float:
        """Measured squeeze in N (approximate; cached, non-blocking).

        This is the follower-side value to forward to the leader's
        :meth:`render_force_feedback`.  It is derived from motor torque
        (about 10 N per Nm), not a fingertip force sensor.

        Raises:
            NotInitializedError: not connected.
        """
        return self.get_state(wait=False).force_n

    def configure_force_feedback(
        self,
        enabled: bool = False,
        gain_nm_per_n: float = 0.10,
        threshold_n: float = 2.0,
        max_tau_nm: float = 1.0,
        damping: float = 0.1,
        timeout_s: float = 0.3,
    ) -> None:
        """Configure leader-side force-feedback rendering.

        **Disabled by default.**  Call this on the leader (the gripper the
        operator moves) before :meth:`render_force_feedback`.

        Args:
            enabled: Turn rendering on.  ``False`` (default) makes
                :meth:`render_force_feedback` a no-op returning ``False``.
            gain_nm_per_n: Leader torque per newton above the threshold.
            threshold_n: Squeeze below which the leader feels nothing.
            max_tau_nm: Ceiling on the rendered resistance, Nm (0..3.5).
            damping: Leader velocity damping while in contact (0..5).
            timeout_s: Feedback age beyond which rendering is dropped.

        Raises:
            CommandError: a value is non-finite or outside its range.
        """
        values = (("gain_nm_per_n", float(gain_nm_per_n)),
                  ("threshold_n", float(threshold_n)),
                  ("max_tau_nm", float(max_tau_nm)),
                  ("damping", float(damping)),
                  ("timeout_s", float(timeout_s)))
        for name, value in values:
            if not math.isfinite(value):
                raise CommandError(
                    f"configure_force_feedback 的 {name} 不是有限数：{value!r}")
        if values[0][1] < 0.0:
            raise CommandError("configure_force_feedback: gain_nm_per_n >= 0")
        if values[1][1] < 0.0:
            raise CommandError("configure_force_feedback: threshold_n >= 0")
        if not 0.0 <= values[2][1] <= _FOLLOW_TAU_MAX_NM:
            raise CommandError(
                f"configure_force_feedback: max_tau_nm 必须在 "
                f"0..{_FOLLOW_TAU_MAX_NM}")
        if not 0.0 <= values[3][1] <= _FOLLOW_KD_MAX:
            raise CommandError(
                f"configure_force_feedback: damping 必须在 0..{_FOLLOW_KD_MAX}")
        if values[4][1] <= 0.0:
            raise CommandError("configure_force_feedback: timeout_s > 0")

        self._feedback_enabled = bool(enabled)
        self._feedback_gain_nm_per_n = values[0][1]
        self._feedback_threshold_n = values[1][1]
        self._feedback_max_tau_nm = values[2][1]
        self._feedback_damping = values[3][1]
        self._feedback_timeout_s = values[4][1]

    def render_force_feedback(self, force_n: float,
                              age_s: float = 0.0) -> bool:
        """Render the follower's squeeze on the leader as one MIT frame.

        Sends ``kp=0``, ``kd=damping``, ``tau=-close_sign * magnitude`` — pure
        resistance plus viscous braking, no position hold.  This is the leader
        side of the force-feedback pair; the follower sends its
        :meth:`get_gripper_force` value (for example through
        ``follow_gripper_distance(..., force_feedback=...)``).

        **Disabled by default**: returns ``False`` and sends nothing until
        :meth:`configure_force_feedback` enables it.

        Args:
            force_n: Follower squeeze in N.  Below the configured threshold
                the frame is a free (zero-gain) frame.
            age_s: Age of the force sample.  Older than ``timeout_s`` drops the
                resistance and damping, so a dead link cannot pin the hand.

        Returns:
            True when a frame was sent, False when feedback is disabled or the
            transport failed (cause in :attr:`last_send_error`).

        Raises:
            NotInitializedError: enabled but not connected/enabled.
            CommandError: uncalibrated, or a non-finite / negative input.
        """
        if not self._feedback_enabled:
            return False
        self._check_connected()
        self._check_enabled()
        _check_calibrated(self._config)

        force_v = float(force_n)
        age_v = float(age_s)
        if not math.isfinite(force_v):
            raise CommandError(
                f"render_force_feedback 的 force_n 不是有限数：{force_v!r}")
        if not math.isfinite(age_v) or age_v < 0.0:
            raise CommandError(
                f"render_force_feedback 的 age_s 必须是有限的非负数：{age_v!r}")

        tau_v = 0.0
        kd_v = 0.0
        if age_v <= self._feedback_timeout_s:
            squeeze = max(0.0, force_v - self._feedback_threshold_n)
            magnitude = min(squeeze * self._feedback_gain_nm_per_n,
                            self._feedback_max_tau_nm)
            if magnitude > 0.0:
                tau_v = -self._config.close_sign * magnitude
                kd_v = self._feedback_damping
        return self._send_mit_frame(0.0, 0.0, kd_v, 0.0, tau_v)

    def grasp(self, force_n: Optional[float] = None,
              hold_s: float = 0.0) -> bool:
        """Adaptive grasp — close until stall, then hold with a set force.

        Blocking.  The approach carries the force budget, so hitting an object
        never presses harder than the setpoint; after contact the motor streams
        a pure torque feed-forward of ``force_n × 0.1 Nm`` (no position or
        velocity gain), ramped up from the torque in flight.  ``hold_s=0``
        holds until :meth:`release`, a fault, or Ctrl+C.

        Args:
            force_n: Gripping force in N; ``None`` uses
                ``MotionConfig.force_n``.  The N scale is approximate
                (10 N per Nm, not calibrated).
            hold_s: Hold duration in seconds; ``0`` = until release/fault.

        Returns:
            True only when the hold was established and ended normally.  False
            on no-bite, travel guard, protection, or comm failure; the reason
            is logged.
        """
        self._check_connected()
        result = self._actions.grasp(force_n, hold_s)
        if not result.ok:
            reason = []
            if result.no_bite:
                reason.append("接近段没能脱离静摩擦（未夹到东西）")
            if result.travel_guard:
                reason.append("保力段越出闭合侧标定限位，已撤力")
            if result.released:
                reason.append("被 release() 结束")
            if not result.reached and not result.stalled and not result.no_bite:
                reason.append("闭合段既未到位也未堵转")
            log.warning("grasp 未成功（%s）：force=%.1f N，stalled=%s，"
                        "reached=%s，cycles=%d",
                        "；".join(reason) or "保力被中断",
                        result.force_n, result.stalled, result.reached,
                        result.cycles)
        return result.ok

    def release(self, clearance_mm: float = 10.0) -> bool:
        """End an in-progress :meth:`grasp` and open the jaws — thread-safe.

        Callable from any thread, including while ``grasp(hold_s=0)`` blocks in
        another one.  The jaws open from the pose where the grip was declared,
        not from where the set force pressed them, by ``clearance_mm`` clamped
        to the calibrated open limit.

        Args:
            clearance_mm: Extra opening beyond the grip pose, in mm; must be a
                non-negative finite number.

        Returns:
            True when the release completed.  False on timeout or a failed
            opening move; the reason is logged.
        """
        self._check_connected()
        result = self._actions.release(clearance_mm)
        if not result.ok:
            log.warning("release 未完成：目标 %.1f mm（%.5f rad），实测 %.1f mm",
                        result.opened_mm, result.target_rad,
                        result.state.position_mm)
        return result.ok

    def _set_position(self, value: float, speed_mm_s: Optional[float],
                      *, distance: bool) -> bool:
        """Shared path for :meth:`set_gripper_distance` / :meth:`set_motor_angle`.

        The finiteness check is deliberately above the clamp: ``min`` / ``max``
        compare false against a NaN, so the clamp would answer it with one of
        the calibrated limits — a real position that was never asked for.
        """
        self._check_connected()
        self._check_enabled()
        _check_calibrated(self._config)
        name = "distance_mm" if distance else "angle_rad"
        if not math.isfinite(value):
            raise CommandError(
                f"{name} 不是有限数：{value!r}。拒绝下发：NaN 落在限位 clamp "
                "里会变成量程的一端，也就是一个调用方没要过的真实位置。")
        if speed_mm_s is None:
            speed = self._actions.config.speed_mm_s
        else:
            speed = speed_mm_s
        if not math.isfinite(speed) or speed <= 0.0:
            raise CommandError(
                f"速度必须是正的有限数：speed_mm_s={speed!r}。速度为 0 或负的"
                "移动没有对应的斜坡。")

        if distance:
            # Opening may increase or decrease the motor rad count depending on
            # the mount; close_sign carries that.
            target_rad = (self._config.pos_closed_rad
                          - self._config.close_sign * value
                          / self._config.rad_to_mm)
            toward = "open" if value > 0.0 else "close"
        else:
            target_rad = value
            toward = ("close"
                      if abs(value - self._config.pos_closed_rad)
                      <= abs(value - self._config.pos_open_rad) else "open")
        result = self._actions.ramp_to(clamp_to_travel(self._config, target_rad),
                                       speed_mm_s=speed)
        label = "set_gripper_distance" if distance else "set_motor_angle"
        if not result.ok:
            log.warning(
                "%s 未到位：目标 %.5f rad（%s），停在 %.5f rad；"
                "reached=%s stalled=%s protected=%s", label,
                result.target_rad, toward, result.state.position_rad,
                result.reached, result.stalled, result.protected)
        return result.ok

    @staticmethod
    def _report_move(label: str, result: MoveResult) -> None:
        if result.ok:
            return
        log.warning(
            "%s 未完成：停在 %.5f rad（目标 %.5f，限位 %.5f）；"
            "reached=%s stalled=%s protected=%s", label,
            result.state.position_rad, result.target_rad, result.limit_rad,
            result.reached, result.stalled, result.protected)

    # ═══════════════════════════════════════════════════════════════════
    # Feedback
    # ═══════════════════════════════════════════════════════════════════

    def get_state(self, wait: bool = True) -> GripperState:
        """Return the current state snapshot, saying how fresh it is.

        Args:
            wait: ``True`` waits up to 0.5 s for a status frame; ``False``
                returns immediately with the cached readings (suitable for
                high-frequency control loops).

        ``wait=True`` sends 0xCC keepalives while it waits, so a 0.5 s wait
        cannot trip the motor's ~400 ms communication-loss latch.  A cached
        frame younger than 50 ms counts as answered, which makes
        ``refresh_status()`` followed by ``get_state()`` report ``fresh``
        instead of waiting for a frame the refresh already consumed.

        Raises:
            NotInitializedError: not connected.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")
        can = self._can

        with self._io_lock:
            if wait:
                last_update = getattr(can, "last_update", None)
                if (last_update is not None
                        and time.monotonic() - last_update <= _FRESH_WINDOW_S):
                    answered = True
                else:
                    answered = can.wait_for_status(timeout_s=0.5)
            else:
                answered = can.poll(timeout_s=0.0)

            position_rad = can.get_position()
            velocity_rad_s = can.get_velocity()
            torque_nm = can.get_torque()
            error_code = can.get_error()
            t_mos, t_coil = can.get_temperature()

            # Both stamps come from the frame, not from this call.
            last_update = getattr(can, "last_update", None)
            frame_time = getattr(can, "last_frame_time", None)
            data_age_s = (None if last_update is None
                          else time.monotonic() - last_update)

            s = self._config.close_sign
            position_mm = ((self._config.pos_closed_rad - position_rad)
                           * s * self._config.rad_to_mm)
            force_n = s * torque_nm * UnitConversion.NM_TO_N

            return GripperState(
                position_rad=position_rad,
                velocity_rad_s=velocity_rad_s,
                torque_nm=torque_nm,
                temperature_mos=t_mos,
                temperature_coil=t_coil,
                error_code=error_code,
                timestamp=0.0 if frame_time is None else frame_time,
                position_mm=position_mm,
                force_n=force_n,
                fresh=bool(answered),
                data_age_s=data_age_s,
            )

    def refresh_status(self) -> bool:
        """Ask the motor for a status frame (0xCC) and wait briefly for it.

        A gripper holding a pose sends nothing on its own, so an idle bus
        yields only cached readings.  This is the request that makes it speak;
        it does not move the jaws.

        Returns:
            True when the request went out **and** a status frame answered.
            False when there is no connection or the write/read failed.

        Raises:
            NotInitializedError: not connected.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")
        with self._io_lock:
            if not self._can.refresh_status():
                log.warning("状态帧请求下发失败（%s）", self._channel)
                return False
            return bool(self._can.update_state(timeout_s=0.1))

    def get_motor_data(self) -> dict:
        """Motor feedback as ``{"Position": rad, "Speed": rad/s, "Torque": Nm}``.

        Reads the cached snapshot without waiting; call :meth:`refresh_status`
        first when the bus has been idle.  Use :meth:`get_state` when the
        freshness flags matter.

        Raises:
            NotInitializedError: not connected.
        """
        state = self.get_state(wait=False)
        return {
            "Position": state.position_rad,
            "Speed": state.velocity_rad_s,
            "Torque": state.torque_nm,
        }

    def get_motor_status(self) -> dict:
        """Status frame summary.

        Keys: ``Status`` (hex string, e.g. ``"0x01"``), ``DriverTemp`` (°C,
        MOS) and ``MotorTemp`` (°C, coil).  Cached and non-blocking; call
        :meth:`refresh_status` for a fresh frame.

        Raises:
            NotInitializedError: not connected.
        """
        state = self.get_state(wait=False)
        return {
            "Status": f"0x{state.error_code:02X}",
            "DriverTemp": int(state.temperature_mos),
            "MotorTemp": int(state.temperature_coil),
        }

    def get_motor_position(self) -> float:
        """Current motor angle in radians (cached, non-blocking)."""
        return self.get_state(wait=False).position_rad

    def get_motor_speed(self) -> float:
        """Current motor speed in rad/s (cached, non-blocking)."""
        return self.get_state(wait=False).velocity_rad_s

    def get_motor_torque(self) -> float:
        """Current motor torque in Nm (cached, non-blocking)."""
        return self.get_state(wait=False).torque_nm

    def get_gripper_distance(self) -> float:
        """Opening in mm from the closed zero (0 = closed; cached)."""
        return self.get_state(wait=False).position_mm

    def get_driver_temp(self) -> int:
        """MOS/driver temperature in °C (cached, non-blocking)."""
        return int(self.get_state(wait=False).temperature_mos)

    def get_motor_temp(self) -> int:
        """Motor coil temperature in °C (cached, non-blocking)."""
        return int(self.get_state(wait=False).temperature_coil)

    def get_status_raw(self) -> str:
        """Raw status/error code of the newest frame, as a hex string."""
        return f"0x{self.get_state(wait=False).error_code:02X}"

    def get_version(self) -> str:
        """SDK/package version — **not** the device firmware version.

        The DM4310 does not expose a firmware readout over this protocol, so
        this reports the installed ``litegrip-python`` distribution version
        (``"0.0.0+source"`` when running from a bare checkout).
        """
        from . import __version__
        return __version__

    # ═══════════════════════════════════════════════════════════════════
    # Internal hooks used by the motion engine
    # ═══════════════════════════════════════════════════════════════════

    def _send_mit_frame(self, q: float, kp: float, kd: float,
                        dq: float = 0.0, tau: float = 0.0) -> bool:
        """Send one MIT frame; the reason for a ``False`` lands in
        :attr:`last_send_error`."""
        if self._can is None:
            self._last_send_error = self._not_connected_error()
            return False
        if not self._enabled:
            self._last_send_error = self._not_enabled_error()
            return False
        with self._io_lock:
            sent = self._can.control_mit(
                q_target=q, kp=kp, kd=kd, dq_target=dq, tau_feedforward=tau)
            self._last_send_error = (
                None if sent else getattr(self._can, "last_send_error", None))
            return sent

    def _enable_once(self) -> bool:
        """One enable attempt: full init (disable → MIT → enable → feedback).

        Raises:
            HardwareError: the motor did not end up enabled.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")

        self._can.update_state(timeout_s=0.05)
        err = self._can.get_error()
        if err not in (0, 1):
            self.clear_fault()

        try:
            self._enabled = self._can.initialize()
            return self._enabled
        except HardwareError:
            raise
        except Exception as e:
            raise HardwareError(f"使能失败: {e}") from e

    def _disable_once(self) -> bool:
        """Send a single disable command and clear the enabled flag."""
        self._check_connected()
        if self._can is None:
            self._enabled = False
            return False
        try:
            result = self._can.disable()
        except Exception as e:                           # noqa: BLE001
            log.warning("失能命令下发失败：%s", e)
            result = False
        self._enabled = False
        return result

    def _not_connected_error(self) -> NotInitializedError:
        return NotInitializedError(
            "未连接 — 请先调用 connect() 或使用 with 上下文")

    def _not_enabled_error(self) -> NotInitializedError:
        return NotInitializedError("未使能 — 请先调用 enable()")

    def _check_connected(self) -> None:
        if not self._connected:
            raise self._not_connected_error()

    def _check_enabled(self) -> None:
        if not self._enabled:
            raise self._not_enabled_error()

    # ═══════════════════════════════════════════════════════════════════
    # Context manager
    # ═══════════════════════════════════════════════════════════════════

    def __enter__(self) -> "Gripper":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        status = "已连接" if self._connected else "未连接"
        enabled = "已使能" if self._enabled else "未使能"
        return (f"Gripper(ch={self._channel}, CAN_ID=0x{self._can_id:02X}, "
                f"MST_ID=0x{self._mst_id:02X}, {status}, {enabled})")

    def __str__(self) -> str:
        return self.__repr__()
