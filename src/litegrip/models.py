"""LiteGrip SDK — data models."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
import time
from typing import Optional


class GripperMode(IntEnum):
    """Gripper control mode."""
    MIT = 0
    POSITION = 1
    VELOCITY = 2
    FORCE = 3


class GripperStatus(IntFlag):
    """Gripper status flags (bitmask)."""
    NONE = 0x00
    ENABLED = 0x01
    MOVING = 0x02
    AT_TARGET = 0x04
    GRASPED = 0x08
    ERROR = 0x10


@dataclass
class GripperState:
    """Live gripper state snapshot.

    All fields are updated on each :meth:`LiteGrip.get_state` call.
    """

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_mos: int = 0
    temperature_coil: int = 0
    error_code: int = 0
    timestamp: float = field(default_factory=time.time)

    # Convenience — computed from raw values with unit conversion
    position_mm: float = 0.0
    force_n: float = 0.0

    @property
    def is_enabled(self) -> bool:
        """True if the motor is enabled (error_code == 1)."""
        return self.error_code == 1

    @property
    def is_error(self) -> bool:
        """True if a fault is active (error_code ∉ {0, 1})."""
        return self.error_code not in (0, 1)

    @property
    def is_moving(self) -> bool:
        """True if velocity exceeds a small threshold."""
        return abs(self.velocity_rad_s) > 0.01

    @property
    def aperture_mm(self) -> float:
        """Opening distance in mm (single-side displacement).

        For total jaw separation multiply by 2.
        """
        return self.position_mm


@dataclass
class GripperConfig:
    """Gripper configuration — tune these for your hardware."""

    # CAN
    can_channel: str = "can0"
    can_id: int = 0x08
    mst_id: Optional[int] = None          # None = auto-detect
    canfd_mode: bool = False

    # Control gains (MIT mode)
    kp: float = 100.0                      # position stiffness [0, 500]
    kd: float = 2.0                        # velocity damping [0, 5]

    # Position limits (rad) — updated by calibrate()
    # closed (0 mm) → numerically larger rad value
    pos_closed_rad: float = 0.0
    # open (full stroke) → numerically smaller rad value
    pos_open_rad: float = 1.14

    # Mechanical stroke (mm) — set to match your gripper's physical travel
    max_stroke_mm: float = 120.0

    # Unit conversion — update after calibration
    rad_to_mm: float = 105.26              # rad → mm
    nm_to_n: float = 10.0                  # Nm → N (approximate)

    # Grasp detection
    grasp_torque_threshold: float = 0.5    # Nm


@dataclass
class GripperInfo:
    """Static device information."""

    model: str = "LiteGrip"
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    firmware_version: str = ""
    serial_number: str = ""


@dataclass
class CalibrationData:
    """Result of a gripper calibration run."""

    zero_position: float = 0.0             # closed limit (rad)
    max_position: float = 1.14             # open limit (rad)
    travel_range: float = 1.14             # max - zero (rad)
    rad_to_mm: float = 105.26              # calibrated conversion
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    calibration_time: str = ""

    @property
    def travel_mm(self) -> float:
        """Full stroke in mm."""
        return self.travel_range * self.rad_to_mm
