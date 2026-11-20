"""LiteGrip SDK — self-contained gripper library for the LiteGrip adaptive
two-finger hand.

Built on the Damiao motor MIT protocol over SocketCAN.  Zero dependencies
outside Python's standard library + Linux SocketCAN.

Quick start::

    from litegrip import Gripper

    with Gripper(channel="can0", can_id=0x08) as gripper:
        gripper.enable()          # retries until the status frame says err == 1
        gripper.open()
        gripper.grasp(force_n=20.0, hold_s=3.0)
        print(gripper.get_motor_data())

Subpackages
-----------
- ``litegrip.can`` — raw SocketCAN transport and DM motor protocol codec.
"""

# Version reported when no ``litegrip-python`` distribution is installed in reach at all —
# no wheel, no editable install, nothing in ``site-packages``/``dist-packages``.
# The ``+`` local segment keeps it valid PEP 440 and makes it obvious in a bug
# report that nobody is looking at an official build.
_VERSION_SOURCE_TREE = "0.0.0+source"


def _detect_version(dist_name: str = "litegrip-python") -> str:
    """Report the installed distribution's version, or a marker for a bare machine.

    The repository never commits a version back: semantic-release derives it
    from the commit prefixes and writes it only into the publish workspace, so
    the git tag is the single source of truth.  Reading the installed metadata
    instead keeps whatever the user actually installed in one place.

    Returns:
        The installed distribution version, or ``"0.0.0+source"`` when no
        ``litegrip-python`` distribution is installed at all.
    """
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version(dist_name)
    except PackageNotFoundError:
        return _VERSION_SOURCE_TREE


__version__ = _detect_version()

# This package deliberately installs no handler and does not call
# ``logging.disable()``: a WARNING such as "this calibration file belongs to
# another channel" is a safety signal, and must reach the user even when the
# embedding program never configured logging.  Applications that want to
# capture the records can attach their own handler to the ``litegrip`` logger.

# ── High-level API ──────────────────────────────────────────────────────
from .gripper import Gripper, default_calibration_path

# ── Motion configuration (engine controls; results are internal) ────────
from .actions import MotionConfig

# ── Data models ─────────────────────────────────────────────────────────
from .models import (
    STALE_AFTER_S,
    CalibrationData,
    GripperConfig,
    GripperState,
)

# ── Constants ───────────────────────────────────────────────────────────
from .constants import (
    DefaultParams,
    ErrorCode,
    GripperGeometry,
    GripperParams,
    UnitConversion,
    describe_error,
)

# ── Exceptions ──────────────────────────────────────────────────────────
from .exceptions import (
    CANTimeoutError,
    CommandError,
    CommError,
    ConnectError,
    HardwareError,
    LiteGripError,
    NotInitializedError,
)

# ── CAN subpackage (expert) ─────────────────────────────────────────────
from . import can

__all__ = [
    "__version__",
    # High-level
    "Gripper",
    "default_calibration_path",
    # Motion configuration
    "MotionConfig",
    # Models
    "STALE_AFTER_S",
    "GripperState",
    "GripperConfig",
    "CalibrationData",
    # Constants
    "GripperGeometry",
    "GripperParams",
    "UnitConversion",
    "ErrorCode",
    "DefaultParams",
    "describe_error",
    # Exceptions
    "LiteGripError",
    "CommError",
    "ConnectError",
    "CommandError",
    "CANTimeoutError",
    "HardwareError",
    "NotInitializedError",
    # Subpackages
    "can",
]
