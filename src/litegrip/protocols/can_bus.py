"""LiteGrip CAN bus layer — self-contained, no damiao_socketcan dependency.

Wraps the litegrip.can subpackage (transport + protocol + motor + controller)
into a simplified interface tailored for a single gripper motor.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from ..can.transport import CanTransport, CanMode as TransportMode
from ..can.controller import MotorController
from ..can.motor import MotorState, MotorType
from ..can.protocol import (
    ControlMode,
    ControlModeCode,
    DM_REG,
)
from ..constants import GripperParams, DefaultParams
from ..exceptions import ConnectError, CommError, HardwareError, NotInitializedError

log = logging.getLogger("litegrip.can_bus")


class LiteGripCAN:
    """LiteGrip CAN communication client.

    Manages a single gripper motor (DM4310 by default) over SocketCAN.
    Entirely self-contained — does NOT import damiao_socketcan or any
    arm-specific library.

    Typical usage::

        can = LiteGripCAN(channel="can0")
        can.connect()
        can.register_gripper(can_id=0x08)
        can.initialize()
        can.control_mit_stream(target_position, kp=100, kd=2, duration_s=1.0)
    """

    def __init__(
        self,
        channel: str = DefaultParams.CAN_CHANNEL,
        canfd_mode: bool = DefaultParams.CANFD_MODE,
    ):
        self._channel = channel
        self._canfd_mode = canfd_mode
        self._transport: Optional[CanTransport] = None
        self._controller: Optional[MotorController] = None
        self._motor: Optional[MotorState] = None
        self._connected = False
        self._initialized = False

    # ═══════════════════════════════════════════════════════════════════
    # Connection
    # ═══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """Open the CAN transport. Auto-detects CAN/CAN-FD mode."""
        if self._connected:
            return True

        # None → let CanTransport auto-detect via interface MTU
        mode = TransportMode.CANFD if self._canfd_mode else TransportMode.CAN

        try:
            self._transport = CanTransport(self._channel, mode)
            self._transport.open()
        except OSError as e:
            raise ConnectError(f"CAN 接口 '{self._channel}' 不可用: {e}")

        self._controller = MotorController(self._transport)
        self._connected = True
        log.info("LiteGripCAN connected on %s (mode=%s)",
                 self._channel, self._transport.mode.name)
        return True

    def disconnect(self, disable: bool = True) -> None:
        """Close the CAN transport and release resources.

        Args:
            disable: Send a disable command first (default).  ``False``
                leaves the motor enabled after the transport is closed.
        """
        if not self._connected:
            return

        if self._initialized and disable:
            try:
                self.disable()
            except Exception:
                pass

        if self._controller:
            self._controller.close()
        elif self._transport:
            self._transport.close()

        self._transport = None
        self._controller = None
        self._motor = None
        self._connected = False
        self._initialized = False

    # ═══════════════════════════════════════════════════════════════════
    # Motor registration
    # ═══════════════════════════════════════════════════════════════════

    def register_gripper(
        self,
        can_id: int = GripperParams.CAN_ID,
        mst_id: Optional[int] = None,
        motor_type: MotorType = MotorType.DM4310,
    ) -> int:
        """Register the gripper motor.

        Args:
            can_id: Motor CAN ID (usually 0x08).
            mst_id: Master ID. None = auto-detect via register read / scan.
            motor_type: Motor model (default DM4310).

        Returns:
            The actual (possibly auto-detected) MST_ID.
        """
        if not self._connected or self._controller is None:
            raise CommError("CAN 未连接")

        try:
            self._motor = self._controller.add_motor(
                can_id=can_id,
                mst_id=mst_id,
                motor_type=motor_type,
                control_mode=ControlMode.MIT_MODE,
            )
            return self._motor.mst_id
        except Exception as e:
            raise CommError(f"注册夹爪电机失败: {e}")

    # ═══════════════════════════════════════════════════════════════════
    # Enable window
    # ═══════════════════════════════════════════════════════════════════
    #
    # In MIT mode the motor continuously executes the last target frame it
    # received: τ = kp·(q_target − q) + kd·(dq_target − dq) + tau_ff, with
    # every one of those five values coming from that frame.  The 0xFC enable
    # command carries no target of its own and does not clear the target
    # registers — it only re-engages the control loop.  So the instant enable
    # takes effect, a motor left over from a previous session would resume
    # driving toward *that* session's target (e.g. you Ctrl+C'd mid-close()
    # and the register still holds q=closed, kp=100).
    #
    # These two helpers close that window: stream zero-gain frames over it,
    # then leave the motor holding wherever it actually is.

    def _hold_at_current(self, kp: float, kd: float,
                         duration_s: float = 0.05) -> bool:
        """Stream hold-position frames at the motor's current position.

        Call this only after a fresh status frame has been decoded — the
        MotorState default position is 0.0, and holding at 0.0 with a real
        gain would drive the gripper to a bogus target.

        Leaving "hold where you are" as the last target also makes the *next*
        enable inherently safe, since that is what the motor resumes from.

        Returns:
            True if the hold frames were streamed.
        """
        if self._controller is None or self._motor is None:
            return False
        if self._motor.rx_count == 0:
            log.warning("Refusing to hold position: no status frame decoded yet")
            return False
        return self.control_mit_stream(
            q_target=self._motor.position, kp=kp, kd=kd,
            duration_s=duration_s)

    def _enable_and_hold(self, kp: float, kd: float,
                         timeout_s: float = DefaultParams.INIT_TIMEOUT_S
                         ) -> Optional[int]:
        """Enable the motor and leave it holding its current position.

        Sequence: 0xFC enable → streamed zero-gain frames → wait for a fresh
        status frame → hold at the freshly-read position with *kp*/*kd*.

        Returns:
            The error code from the first fresh status frame, or None if no
            feedback arrived within *timeout_s*.
        """
        if self._controller is None or self._motor is None:
            return None

        ctrl = self._controller
        m = self._motor
        prev_rx = m.rx_count

        ctrl.enable(m)

        # Zero-gain cover.  Streamed rather than a single frame: everything
        # else in this SDK streams for the same reason, and here one lost
        # frame means the motor keeps running the stale target indefinitely
        # rather than for ~10 ms.
        self.control_mit_stream(q_target=0.0, kp=0.0, kd=0.0,
                                duration_s=0.05, interval_s=0.005)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            # Keep feeding the motor while we wait.  It is ENABLED from the
            # 0xFC above, and an enabled motor that hears nothing for ~900 ms
            # latches the 0xD comm-loss fault (measured on hardware) — which
            # is exactly the fault this wait is supposed to detect, so silence
            # here manufactures the failure it is looking for.  The 0.05 s
            # burst before the loop is not long enough to cover a 2 s wait.
            ctrl.control_mit(m, kp=0, kd=0, q=0, dq=0, tau=0)
            ctrl.poll(timeout_s=0.01)
            if m.rx_count > prev_rx:
                break
            time.sleep(0.005)
        else:
            return None

        # rx_count only advances on a decoded status frame, so m.position is a
        # real reading from here on.
        self._hold_at_current(kp, kd)
        return m.error

    # ═══════════════════════════════════════════════════════════════════
    # Initialization / enable / disable
    # ═══════════════════════════════════════════════════════════════════

    def initialize(self, kp: Optional[float] = None,
                   kd: Optional[float] = None) -> bool:
        """Full initialization: disable → switch to MIT → enable → verify feedback.

        Once enabled, the motor is left holding its current position (see
        :meth:`_enable_and_hold`) rather than outputting zero torque, so it
        cannot jump toward whatever target the previous session left behind.

        Args:
            kp: Stiffness used for the post-enable hold (default DEFAULT_KP).
            kd: Damping used for the post-enable hold (default DEFAULT_KD).

        Returns:
            True if the motor reports ``err == 1`` (enabled).  ``err == 0``
            means the enable command did not take effect — the attempt is
            retried rather than reported as success.

        Raises:
            HardwareError: on timeout, persistent fault, no feedback, or if
                every attempt leaves the motor not enabled (``err == 0``).
        """
        if not self._connected or self._controller is None or self._motor is None:
            raise NotInitializedError("未连接或未注册")

        ctrl = self._controller
        m = self._motor
        kp = kp if kp is not None else GripperParams.DEFAULT_KP
        kd = kd if kd is not None else GripperParams.DEFAULT_KD
        last_err = -1

        for attempt in range(GripperParams.FAULT_CLEAR_RETRIES):
            try:
                # 1. Disable
                ctrl.disable(m)
                time.sleep(0.01)

                # 2. Switch to MIT mode
                if not ctrl.switch_control_mode(m, ControlModeCode.MIT):
                    log.warning("switchControlMode verification failed; proceeding anyway")
                time.sleep(0.05)

                # 3+4. Enable, cover the window, verify feedback, then hold
                #      at the freshly-read position.
                err = self._enable_and_hold(kp, kd)

                if err is None:
                    last_err = -2  # timeout, no feedback
                elif err == 1:
                    self._initialized = True
                    return True
                else:
                    # err == 0 → the enable frame was lost; retry.
                    # anything else → a real fault; retry after clearing.
                    last_err = err

            except Exception:
                last_err = -3

            # Retry: clear the fault before the next attempt.  No bare
            # enable() here — the next iteration disables immediately, and an
            # uncovered enable is exactly the window this method avoids.
            if attempt < GripperParams.FAULT_CLEAR_RETRIES - 1:
                ctrl.clear_fault(m)
                time.sleep(0.01)

        # All retries exhausted → raise
        if last_err == 0x9:
            raise HardwareError(
                "电机欠压故障 (UV_FAULT)，请检查夹爪 24V 供电", error_code=0x9)
        elif last_err == 0:
            raise HardwareError(
                "使能未生效（err=0，enable 命令可能丢帧）", error_code=0)
        elif last_err == -2:
            raise HardwareError(
                f"电机无反馈（超时 {DefaultParams.INIT_TIMEOUT_S}s），"
                "请检查夹爪供电和 CAN 连接")
        elif last_err != -3:
            raise HardwareError(
                f"电机故障 0x{last_err:X} 无法清除", error_code=last_err)
        else:
            raise HardwareError("初始化失败：所有重试耗尽")

    def enable(self, kp: Optional[float] = None,
               kd: Optional[float] = None) -> bool:
        """Send enable command only (no full init). Prefer initialize().

        The motor is left holding its current position once enabled — see
        :meth:`_enable_and_hold`.
        """
        if self._controller is None or self._motor is None:
            return False
        kp = kp if kp is not None else GripperParams.DEFAULT_KP
        kd = kd if kd is not None else GripperParams.DEFAULT_KD
        try:
            # None (no feedback) fails the membership test, so a silent motor
            # is no longer reported as a successful enable.
            return self._enable_and_hold(kp, kd) in (0, 1)
        except Exception:
            return False

    def disable(self) -> bool:
        """Send disable command."""
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.disable(self._motor)
            self._initialized = False
            return True
        except Exception:
            return False

    # ═══════════════════════════════════════════════════════════════════
    # Fault handling
    # ═══════════════════════════════════════════════════════════════════

    def _clear_fault(self) -> bool:
        """[Deprecated] Use :meth:`clear_fault` instead."""
        return self.clear_fault()

    def clear_fault(self, kp: Optional[float] = None,
                    kd: Optional[float] = None) -> bool:
        """Clear latched fault and re-enable the motor.

        Tries two strategies:
        1. Direct clear (0xFB) + enable (0xFC) — works for UV/OC/OT faults
           where the motor is already effectively disabled.
        2. Full disable → clear → enable cycle.

        Both re-enable through :meth:`_enable_and_hold`, so the motor ends up
        holding its current position instead of resuming a stale target, and a
        motor that never answers counts as a failure rather than a success.
        """
        if self._controller is None or self._motor is None:
            return False

        ctrl = self._controller
        m = self._motor
        kp = kp if kp is not None else GripperParams.DEFAULT_KP
        kd = kd if kd is not None else GripperParams.DEFAULT_KD

        for _ in range(GripperParams.FAULT_CLEAR_RETRIES):
            # Strategy 1: Direct clear + enable (proven for UV_FAULT)
            ctrl.clear_fault(m)
            time.sleep(0.005)
            if self._enable_and_hold(kp, kd) in (0, 1):
                return True

            # Strategy 2: Full disable → clear → enable
            ctrl.disable(m)
            time.sleep(0.01)
            ctrl.clear_fault(m)
            time.sleep(0.01)
            if self._enable_and_hold(kp, kd) in (0, 1):
                return True

        return False

    # ═══════════════════════════════════════════════════════════════════
    # Motion control
    # ═══════════════════════════════════════════════════════════════════

    def control_mit(self, q_target: float, kp: float, kd: float,
                    dq_target: float = 0.0,
                    tau_feedforward: float = 0.0) -> bool:
        """Send a single MIT control frame."""
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.control_mit(
                self._motor, kp=kp, kd=kd,
                q=q_target, dq=dq_target, tau=tau_feedforward)
            return True
        except Exception:
            return False

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Poll for one CAN frame and update motor state if relevant.

        Args:
            timeout_s: Max wait time (0 = non-blocking).

        Returns:
            True if a frame for our motor was received.
        """
        if self._controller is None:
            return False
        return self._controller.poll(timeout_s=timeout_s) is not None

    def control_mit_stream(
        self,
        q_target: float,
        kp: float,
        kd: float,
        duration_s: float,
        dq_target: float = 0.0,
        tau_feedforward: float = 0.0,
        interval_s: float = 0.005,
    ) -> bool:
        """Stream MIT frames for a duration (blocking).

        DM motors require continuous MIT frames to sustain motion.
        """
        if self._controller is None or self._motor is None:
            return False

        ctrl = self._controller
        m = self._motor
        deadline = time.monotonic() + duration_s

        try:
            while time.monotonic() < deadline:
                ctrl.control_mit(m, kp=kp, kd=kd,
                                 q=q_target, dq=dq_target, tau=tau_feedforward)
                ctrl.poll(timeout_s=0.0)
                time.sleep(interval_s)
            return True
        except Exception:
            return False

    # ═══════════════════════════════════════════════════════════════════
    # Status
    # ═══════════════════════════════════════════════════════════════════

    def update_state(self, timeout_s: float = 0.05) -> bool:
        """Poll until at least one new status frame arrives."""
        if self._controller is None or self._motor is None:
            return False
        return self._controller.poll_until(self._motor, timeout_s=timeout_s)

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        """Request a status frame and wait for it (0xCC refresh command).

        A disabled motor does not stream status frames on its own, so
        :meth:`update_state` finds nothing and callers read stale/default
        values.  The refresh command is answered regardless of enable state,
        which makes the position readable before the first enable.  Sends no
        motion command and changes no motor output.

        Returns:
            True if a fresh status frame arrived within *timeout_s*.
        """
        if self._controller is None or self._motor is None:
            return False
        m = self._motor
        prev_rx = m.rx_count
        self._controller.refresh_status(m)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._controller.poll(timeout_s=0.01)
            if m.rx_count > prev_rx:
                return True
            time.sleep(0.005)
        return False

    def get_position(self) -> float:
        """Current position in rad."""
        if self._motor is None:
            return 0.0
        return self._motor.position

    def get_velocity(self) -> float:
        """Current velocity in rad/s."""
        if self._motor is None:
            return 0.0
        return self._motor.velocity

    def get_torque(self) -> float:
        """Current torque in Nm."""
        if self._motor is None:
            return 0.0
        return self._motor.torque

    def get_error(self) -> int:
        """Error code (0=disabled, 1=enabled, 0x9=UV, etc.)."""
        if self._motor is None:
            return -1
        return self._motor.error

    def get_temperature(self) -> tuple[int, int]:
        """MOS and coil temperatures in °C."""
        if self._motor is None:
            return 0, 0
        return self._motor.t_mos, self._motor.t_coil

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        """Read a motor register by RID."""
        if self._controller is None or self._motor is None:
            raise NotInitializedError("未连接或未注册")
        return self._controller.read_param(self._motor, DM_REG(rid),
                                           timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Properties
    # ═══════════════════════════════════════════════════════════════════

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def motor(self) -> Optional[MotorState]:
        """Direct access to the underlying MotorState (expert use)."""
        return self._motor
