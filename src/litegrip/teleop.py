"""Leader/follower gripper teleoperation — single-DOF position mirroring.

Ported from ``litearm_device.gripper_teleop`` (the implementation that drove the
same feature inside the litearm server stack), minus the parts that only made
sense there.  The algorithm is unchanged; what is new is that the transport is
pluggable and the module depends on nothing outside the standard library, so a
bare ``LiteGrip`` on a CAN bus is all it takes.

Topology::

    leader  (zero-gravity, hand-back-driven)  --pub-->  follower (MIT follow)

``master`` streams zero-torque frames so the jaws can be pushed by hand, and
publishes its normalised opening at ``rate_hz``.  ``slave`` subscribes, aligns
once, then streams MIT position frames toward the received opening.

Why the wire carries ``openness`` and not radians
-------------------------------------------------
Each gripper has its own zero, direction and calibration (one unit opens at
-1.42 rad, another at +1.14 rad), so a raw angle is meaningless on the far
side.  ``openness`` is the opening normalised by the *local* travel and is
therefore dimensionless and direction-free; each side converts on its own.

Frame layout (big-endian, four doubles, 32 bytes) — byte-compatible with the
litearm implementation so the two can interoperate::

    openness[0..1] | position_mm | force_n | timestamp

Safety notes
------------
* ``openness`` is clamped to ``[0, 1]``, which keeps every commanded target
  inside the calibrated travel.  That clamp is the only limit this layer
  applies; there is no red-line logic here.
* A ``slave`` whose leader goes quiet **holds** its last target at the follow
  gains (it does not relax to zero torque).  The jaws therefore keep pressing
  whatever is between them — the same behaviour as the litearm original.
* Teleoperation is exclusive: stop any motion you started elsewhere before
  calling :meth:`~litegrip.LiteGrip.teleop_start`.
* :class:`UdpTeleopTransport` is plain, unauthenticated UDP.  Use it only on a
  trusted network.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, Optional, Tuple, Union

from .exceptions import LiteGripError

log = logging.getLogger("litegrip.teleop")

# ── Frame codec ───────────────────────────────────────────────────────────

_FRAME = struct.Struct(">4d")

#: Size of one teleop frame in bytes (four doubles).
FRAME_SIZE = _FRAME.size


def encode_frame(openness: float, position_mm: float, force_n: float,
                 timestamp: float) -> bytes:
    """Pack one teleop frame.

    Args:
        openness: Normalised opening in ``[0, 1]`` (0 = closed, 1 = fully
            open).  The quantity that actually drives the follower.
        position_mm: Leader opening in mm — diagnostic only.
        force_n: Leader gripping force in N — diagnostic only.
        timestamp: Sender clock in seconds — diagnostic only; the follower
            judges liveness from its own receive time, not from this field.
    """
    return _FRAME.pack(float(openness), float(position_mm), float(force_n),
                       float(timestamp))


def decode_frame(payload: bytes) -> Tuple[float, float, float, float]:
    """Unpack a teleop frame into ``(openness, position_mm, force_n, timestamp)``.

    Raises:
        ValueError: ``payload`` is not exactly :data:`FRAME_SIZE` bytes.
    """
    if len(payload) != FRAME_SIZE:
        raise ValueError(
            f"teleop frame must be {FRAME_SIZE} bytes, got {len(payload)}")
    return _FRAME.unpack(payload)


def teleop_topic(master_id: str = "master") -> str:
    """Topic the leader publishes and the follower subscribes to.

    Both ends must agree on ``master_id``; it defaults to ``"master"`` so a
    single pair needs no configuration.  Use a distinct id per pair when more
    than one teleoperation runs on the same transport.
    """
    return f"litegrip/teleop/{master_id}"


# ── Transport abstraction ─────────────────────────────────────────────────


class TeleopSubscription:
    """A non-blocking subscription handle."""

    def try_recv(self) -> Optional[bytes]:
        """Return the next payload, or ``None`` if none is waiting."""
        raise NotImplementedError

    def drain_latest(self) -> Optional[bytes]:
        """Discard all but the newest queued payload and return it.

        Teleoperation only ever wants the latest sample, so a slow consumer
        skips history instead of replaying it.
        """
        latest = None
        while True:
            msg = self.try_recv()
            if msg is None:
                return latest
            latest = msg


class TeleopTransport:
    """Publish/subscribe transport between a leader and a follower.

    Implement this to carry teleop frames over anything (a different network
    stack, an in-process bus, ...).  Two implementations ship with the SDK:
    :class:`UdpTeleopTransport` and :class:`InProcTeleopTransport`.
    """

    def pub(self, topic: str, payload: bytes) -> None:
        raise NotImplementedError

    def sub(self, topic: str) -> TeleopSubscription:
        raise NotImplementedError

    def close(self) -> None:
        """Release any resources.  Idempotent."""


def _parse_addr(addr: Union[str, Tuple[str, int]]) -> Tuple[str, int]:
    if isinstance(addr, str):
        host, _, port = addr.rpartition(":")
        if not host or not port:
            raise ValueError(f"address must be 'host:port', got {addr!r}")
        return host, int(port)
    host, port = addr
    return str(host), int(port)


class _UdpSubscription(TeleopSubscription):
    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock

    def try_recv(self) -> Optional[bytes]:
        try:
            data, _ = self._sock.recvfrom(2048)
            return data
        except (BlockingIOError, InterruptedError):
            return None
        except OSError as e:
            log.debug("udp recv failed: %s", e)
            return None


class UdpTeleopTransport(TeleopTransport):
    """Plain UDP transport — the zero-dependency default.

    The leader sends frames to ``pub_addr``; the follower receives on
    ``bind_addr``.  Either or both may be given, so one object can both send
    and receive (not needed for a single leader/follower pair).  Same machine:
    ``"127.0.0.1:7448"``.  Across machines: the peer's real address,
    ``"0.0.0.0:<port>"`` to accept on every interface.

    Unauthenticated and unencrypted — trusted networks only.  A dropped
    datagram is simply the next sample being late, which the follower's
    watchdog already tolerates.
    """

    def __init__(self, pub_addr: Union[str, Tuple[str, int], None] = None,
                 bind_addr: Union[str, Tuple[str, int], None] = None) -> None:
        self._pub_addr = _parse_addr(pub_addr) if pub_addr is not None else None
        self._sock: Optional[socket.socket] = None
        self._sub: Optional[_UdpSubscription] = None

        if bind_addr is not None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(_parse_addr(bind_addr))
            sock.setblocking(False)
            self._sock = sock
        elif self._pub_addr is not None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if self._pub_addr is None and bind_addr is None:
            raise ValueError("UdpTeleopTransport needs pub_addr, bind_addr or both")

    def pub(self, topic: str, payload: bytes) -> None:
        if self._pub_addr is None or self._sock is None:
            return
        try:
            self._sock.sendto(payload, self._pub_addr)
        except OSError as e:
            log.debug("udp send failed: %s", e)

    def sub(self, topic: str) -> TeleopSubscription:
        if self._sock is None:
            raise TeleopError(
                "UdpTeleopTransport was built without bind_addr; it cannot subscribe")
        if self._sub is None:
            self._sub = _UdpSubscription(self._sock)
        return self._sub

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._sub = None


class _InProcSubscription(TeleopSubscription):
    def __init__(self, queue: "deque[bytes]") -> None:
        self._queue = queue

    def try_recv(self) -> Optional[bytes]:
        try:
            return self._queue.popleft()
        except IndexError:
            return None


class InProcTeleopTransport(TeleopTransport):
    """In-process bus — for tests and for two grippers in one program.

    A single instance is shared by both ends; nothing crosses a process
    boundary.  Each topic keeps a bounded FIFO per subscriber, so a subscriber
    that falls behind skips frames rather than growing without bound.
    """

    def __init__(self, fifo_depth: int = 16) -> None:
        self._depth = fifo_depth
        self._queues: Dict[str, list] = {}
        self._lock = threading.Lock()

    def pub(self, topic: str, payload: bytes) -> None:
        with self._lock:
            for queue in self._queues.get(topic, []):
                queue.append(payload)
                while len(queue) > self._depth:
                    queue.popleft()

    def sub(self, topic: str) -> TeleopSubscription:
        queue: "deque[bytes]" = deque()
        with self._lock:
            self._queues.setdefault(topic, []).append(queue)
        return _InProcSubscription(queue)

    def close(self) -> None:
        with self._lock:
            self._queues.clear()


# ── openness <-> radians conversion ───────────────────────────────────────


def travel_mm(cfg: Any) -> float:
    """Full calibrated stroke in mm (``|open - closed| * rad_to_mm``)."""
    return abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm


def rad_to_openness(position_rad: float, cfg: Any) -> float:
    """Motor angle -> normalised opening in ``[0, 1]``.

    Uses the same sign convention as :meth:`LiteGrip.get_state`, so it is
    correct for both mountings (``cfg.close_sign`` carries the direction).
    """
    stroke = travel_mm(cfg)
    if stroke <= 0.0:
        return 0.0
    position_mm = ((cfg.pos_closed_rad - position_rad)
                   * cfg.close_sign * cfg.rad_to_mm)
    return _clamp01(position_mm / stroke)


def openness_to_rad(openness: float, cfg: Any) -> float:
    """Normalised opening in ``[0, 1]`` -> motor angle.

    Mirrors :meth:`LiteGrip.goto_mm` including its ``close_sign`` factor, so a
    reverse-mounted follower moves the correct way.  ``openness`` is clamped
    to ``[0, 1]`` first, which bounds the target to the calibrated travel.
    """
    if cfg.rad_to_mm <= 0.0:
        return cfg.pos_closed_rad
    openness = _clamp01(openness)
    return (cfg.pos_closed_rad
            - cfg.close_sign * openness * travel_mm(cfg) / cfg.rad_to_mm)


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


# ── exceptions ────────────────────────────────────────────────────────────


class TeleopError(LiteGripError):
    """Base class for teleoperation errors."""


class TeleopBusyError(TeleopError):
    """Raised when teleop is started while it is already running."""


class TeleopNotActiveError(TeleopError):
    """Raised when an operation needs an active session but none is running."""


# ── the algorithm ─────────────────────────────────────────────────────────


class GripperTeleop:
    """One side of a gripper teleoperation, driven by a background thread.

    ``mode="master"`` streams zero-torque frames (so the jaws can be moved by
    hand) and publishes the opening.  ``mode="slave"`` subscribes, aligns to
    the first frame with a single :meth:`~litegrip.LiteGrip.goto_rad`, then
    follows every fresh sample with ``send_mit_frame``.

    One loop thread per side, sampling and sending in the same cycle — no
    shared buffers and no contention, which is all a single-DOF gripper needs
    (see the litearm original for the reasoning).

    Args:
        gripper: The ``LiteGrip`` this side drives.
        transport: Transport used to publish (master) and, unless
            ``sub_transport`` is given, to subscribe (slave).
        mode: ``"master"`` or ``"slave"``.
        topic: Topic to publish/subscribe.
        rate_hz: Loop rate.  ~50 Hz is plenty; the CAN frame stream and the
            publish share the same cycle.
        kp, kd: Follow gains (slave).  ``None`` uses ``100.0`` / ``2.0``.
        align: Slave only — align to the first frame before following.
        watchdog_s: Slave only — seconds without a fresh frame before the
            follower is considered stale and starts holding.
        sub_transport: Slave only — a separate transport to subscribe on when
            the leader is remote (the master's transport is local-only).
        sleep_fn, time_fn: Timing seams for tests.  ``time_fn`` must be
            monotonic.
    """

    def __init__(
        self,
        gripper: Any,
        transport: TeleopTransport,
        mode: str,
        topic: str,
        rate_hz: float = 50.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        align: bool = True,
        watchdog_s: float = 0.2,
        sub_transport: Optional[TeleopTransport] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if mode not in ("master", "slave"):
            raise ValueError(f"mode must be 'master' or 'slave', got {mode!r}")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        self._g = gripper
        self._tp = transport
        self._mode = mode
        self._topic = topic
        self._dt = 1.0 / rate_hz
        self._kp = kp
        self._kd = kd
        self._align = align
        self._watchdog_s = watchdog_s
        self._sub_tp = sub_transport
        self._sleep_fn = sleep_fn
        self._time_fn = time_fn

        self._thread: Optional[threading.Thread] = None
        self._running = False

        # Diagnostics.
        self._frames = 0
        self._last_openness = 0.0
        self._last_frame_ts = 0.0
        self._stale = False
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ── lifecycle ─────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start the background loop.  Raises :class:`TeleopBusyError` if it
        is already running."""
        if self._running:
            raise TeleopBusyError("teleop already running")
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name=f"litegrip-teleop-{self._mode}", daemon=True)
        self._thread.start()
        log.info("teleop started: mode=%s topic=%s rate=%.0fHz",
                 self._mode, self._topic, 1.0 / self._dt)

    def _run(self) -> None:
        # Whatever ends the loop — a stop, a CAN error, a bad transport — the
        # session is no longer active once the thread returns.
        try:
            if self._mode == "master":
                self._master_loop()
            else:
                self._slave_loop()
        finally:
            self._running = False

    def stop(self, timeout: float = 2.0) -> None:
        """Stop the loop and leave the gripper holding its position.

        The master also leaves zero-gravity mode, so the jaws hold under the
        configured gains instead of falling slack.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        if self._mode == "master":
            try:
                self._g.exit_zero_gravity()
            except Exception as e:  # noqa: BLE001
                log.debug("exit_zero_gravity on stop failed: %s", e)
        log.info("teleop stopped: mode=%s frames=%d", self._mode, self._frames)

    def status(self) -> dict:
        """A snapshot of the session, for logging and diagnostics."""
        age_ms = None
        if self._mode == "slave" and self._last_frame_ts > 0.0:
            age_ms = (self._time_fn() - self._last_frame_ts) * 1000.0
        return {
            "active": self._running,
            "mode": self._mode,
            "topic": self._topic,
            "frames": self._frames,
            "last_frame_age_ms": age_ms,
            "stale": self._stale,
            "openness": round(self._last_openness, 4),
            "loop_hz": round(self._loop_hz, 1),
        }

    # ── master ────────────────────────────────────────────────────────

    def _master_loop(self) -> None:
        log.info("[master] zero-gravity, publishing to %s", self._topic)
        try:
            while self._running:
                t0 = self._time_fn()
                # Keep the frame stream alive: the DM motor self-locks a
                # "communication loss" fault ~100 ms after frames stop, so a
                # zero-torque frame goes out every cycle even though nothing
                # is being commanded.
                try:
                    self._g.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
                    state = self._g.get_state(wait=False)
                except Exception:  # noqa: BLE001
                    log.exception("[master] CAN error; loop exiting")
                    break
                openness = rad_to_openness(state.position_rad, self._g.config)
                self._last_openness = openness
                try:
                    self._tp.pub(self._topic, encode_frame(
                        openness, state.position_mm, state.force_n,
                        self._time_fn()))
                except Exception as e:  # noqa: BLE001
                    log.debug("[master] publish failed: %s", e)
                self._frames += 1
                self._sleep_rest(t0)
        finally:
            log.info("[master] loop exited (%d frames sent)", self._frames)

    # ── slave ─────────────────────────────────────────────────────────

    def _slave_loop(self) -> None:
        transport = self._sub_tp or self._tp
        sub = transport.sub(self._topic)
        cfg = self._g.config
        log.info("[slave] subscribed %s (align=%s watchdog=%.0fms)",
                 self._topic, self._align, self._watchdog_s * 1000.0)

        # Until a frame arrives, hold wherever the jaws already are.
        openness_cmd = rad_to_openness(self._g.get_state(wait=False).position_rad, cfg)
        q_cmd = openness_to_rad(openness_cmd, cfg)

        if self._align:
            first = self._wait_first_frame(sub, timeout_s=5.0)
            if first is not None:
                openness_cmd = _clamp01(first[0])
                q_cmd = openness_to_rad(openness_cmd, cfg)
                log.info("[slave] aligning to first frame: openness=%.3f -> %.3f rad",
                         openness_cmd, q_cmd)
                try:
                    self._g.goto_rad(q_cmd, kp=self._resolve_kp(),
                                     kd=self._resolve_kd(), duration=1.0)
                except Exception as e:  # noqa: BLE001
                    log.warning("[slave] align goto_rad failed: %s", e)
                self._last_frame_ts = self._time_fn()
            else:
                log.warning("[slave] no frame within align timeout; "
                            "holding current position")

        try:
            while self._running:
                t0 = self._time_fn()
                msg = sub.drain_latest()
                if msg is not None:
                    try:
                        openness, _mm, _force, _ts = decode_frame(msg)
                    except ValueError as e:
                        log.debug("[slave] ignoring bad frame: %s", e)
                    else:
                        openness_cmd = _clamp01(openness)
                        q_cmd = openness_to_rad(openness_cmd, cfg)
                        self._last_openness = openness_cmd
                        self._last_frame_ts = self._time_fn()
                        self._frames += 1
                        self._stale = False
                elif (self._last_frame_ts > 0.0
                      and (self._time_fn() - self._last_frame_ts) > self._watchdog_s):
                    if not self._stale:
                        log.warning("[slave] frames stale (>%.0fms); holding position",
                                    self._watchdog_s * 1000.0)
                    self._stale = True

                # Always send — including while stale.  The frame both holds
                # the position and keeps the motor from self-locking.
                try:
                    self._g.send_mit_frame(q=q_cmd, kp=self._resolve_kp(),
                                           kd=self._resolve_kd(), dq=0.0)
                except Exception:  # noqa: BLE001
                    log.exception("[slave] CAN error; loop exiting")
                    break
                self._sleep_rest(t0)
        finally:
            log.info("[slave] loop exited (%d frames received)", self._frames)

    # ── helpers ───────────────────────────────────────────────────────

    def _wait_first_frame(self, sub: TeleopSubscription,
                          timeout_s: float) -> Optional[Tuple[float, float, float, float]]:
        deadline = self._time_fn() + timeout_s
        while self._running and self._time_fn() < deadline:
            msg = sub.drain_latest()
            if msg is not None:
                try:
                    return decode_frame(msg)
                except ValueError:
                    continue
            self._sleep_fn(0.01)
        return None

    def _resolve_kp(self) -> float:
        return self._kp if self._kp is not None else 100.0

    def _resolve_kd(self) -> float:
        return self._kd if self._kd is not None else 2.0

    def _sleep_rest(self, t0: float) -> None:
        self._loops += 1
        now = self._time_fn()
        if self._hz_t0 == 0.0:
            self._hz_t0 = now
            self._hz_n0 = self._loops
        elif now - self._hz_t0 >= 1.0:
            self._loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0 = now
            self._hz_n0 = self._loops
        rest = self._dt - (self._time_fn() - t0)
        if rest > 0.0:
            self._sleep_fn(rest)
