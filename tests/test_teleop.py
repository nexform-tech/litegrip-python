"""Hardware-free tests for leader/follower gripper teleoperation.

The loops run for real (a thread each) but with the sleep seam stubbed, so a
"cycle" costs microseconds.  Assertions therefore poll with a deadline rather
than assuming a fixed number of cycles elapsed.

The slave-side tests publish through :class:`_PreSubTransport`, whose
subscription exists before the loop starts.  A real ``InProcTeleopTransport``
only delivers to subscribers that already exist, so publishing before
``start()`` would otherwise race the loop's own ``sub()`` call.
"""

from __future__ import annotations

import time
import types
import unittest
import unittest.mock

import _sdkpath  # noqa: F401
from litegrip import (DEFAULT_ALIGN_SPEED_MM_S, DEFAULT_DQ_MAX,
                      DEFAULT_LEAD_CAP_MM, FRAME_SIZE, GripperTeleop,
                      InProcTeleopTransport, TeleopBusyError, TeleopNotReady,
                      UdpTeleopTransport, decode_frame, encode_frame,
                      teleop_topic)
from litegrip.teleop import (MAX_FRAME_GAP_S, TORQUE_REARM_OPENNESS,
                             TORQUE_TRIP_CYCLES, clamp_to_calibrated, check_ready,
                             openness_to_rad, rad_to_openness, travel_mm)

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM, make_gripper

TOPIC = teleop_topic("gripA")

# Far longer than the stub-sleep loop needs; a timeout here means the loop is
# not running at all, not that the machine is slow.
WAIT_S = 2.0


def _wait_until(predicate, timeout_s: float = WAIT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


def _nop_sleep(_seconds: float) -> None:
    """Stub the loop's pacing so tests do not pay the frame interval."""


class _PreSubTransport(InProcTeleopTransport):
    """In-process bus with the slave's subscription created up front."""

    def __init__(self) -> None:
        super().__init__()
        self.handle = super().sub(TOPIC)

    def sub(self, topic: str):
        return self.handle if topic == TOPIC else super().sub(topic)


class FrameCodecTest(unittest.TestCase):
    def test_topic_matches_the_litearm_namespace(self):
        # Shared with the validated litearm teleoperation stack, deliberately.
        self.assertEqual(TOPIC, "litearm/v4/gripA/gripper_teleop")
        self.assertEqual(teleop_topic(), "litearm/v4/gripA/gripper_teleop")

    def test_size_is_four_doubles(self):
        self.assertEqual(FRAME_SIZE, 32)
        self.assertEqual(len(encode_frame(0.0, 0.0, 0.0, 0.0)), FRAME_SIZE)

    def test_roundtrip(self):
        payload = encode_frame(0.25, 61.5, -3.5, 1234.5)
        self.assertEqual(decode_frame(payload), (0.25, 61.5, -3.5, 1234.5))

    def test_decode_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            decode_frame(b"\x00" * (FRAME_SIZE - 1))


class ConversionTest(unittest.TestCase):
    def test_travel_matches_limits(self):
        g, _ = make_gripper()
        self.assertAlmostEqual(
            travel_mm(g.config), abs(POS_CLOSED_RAD - POS_OPEN_RAD) * RAD_TO_MM)

    def test_normal_mount_endpoints_and_inverse(self):
        g, _ = make_gripper(reverse=False)
        cfg = g.config
        self.assertAlmostEqual(openness_to_rad(0.0, cfg), cfg.pos_closed_rad)
        self.assertAlmostEqual(openness_to_rad(1.0, cfg), cfg.pos_open_rad)
        self.assertAlmostEqual(rad_to_openness(cfg.pos_closed_rad, cfg), 0.0)
        self.assertAlmostEqual(rad_to_openness(cfg.pos_open_rad, cfg), 1.0)
        # Round trip, and agreement with the SDK's own goto_mm formula.
        self.assertAlmostEqual(rad_to_openness(openness_to_rad(0.37, cfg), cfg),
                               0.37, places=6)
        self.assertAlmostEqual(
            openness_to_rad(0.4, cfg),
            cfg.pos_closed_rad - 0.4 * travel_mm(cfg) / cfg.rad_to_mm)

    def test_reverse_mount_endpoints_and_inverse(self):
        g, _ = make_gripper(reverse=True)
        cfg = g.config
        self.assertEqual(cfg.close_sign, -1.0)
        self.assertAlmostEqual(openness_to_rad(0.0, cfg), cfg.pos_closed_rad)
        self.assertAlmostEqual(openness_to_rad(1.0, cfg), cfg.pos_open_rad)
        self.assertAlmostEqual(rad_to_openness(cfg.pos_open_rad, cfg), 1.0)
        self.assertAlmostEqual(rad_to_openness(openness_to_rad(0.6, cfg), cfg),
                               0.6, places=6)
        # Reverse is the normal formula with the sign flipped — the fix over
        # the litearm original, which assumed a normal mount.
        self.assertAlmostEqual(
            openness_to_rad(0.4, cfg),
            cfg.pos_closed_rad + 0.4 * travel_mm(cfg) / cfg.rad_to_mm)

    def test_openness_is_clamped(self):
        g, _ = make_gripper()
        cfg = g.config
        self.assertAlmostEqual(openness_to_rad(-5.0, cfg), openness_to_rad(0.0, cfg))
        self.assertAlmostEqual(openness_to_rad(5.0, cfg), openness_to_rad(1.0, cfg))


class InProcTransportTest(unittest.TestCase):
    def test_pub_sub_drain_latest(self):
        transport = InProcTeleopTransport()
        sub = transport.sub("t")
        transport.pub("t", b"old")
        transport.pub("t", b"new")
        self.assertEqual(sub.drain_latest(), b"new")
        self.assertIsNone(sub.drain_latest())

    def test_bounded_queue_drops_oldest(self):
        transport = InProcTeleopTransport(fifo_depth=2)
        sub = transport.sub("t")
        for i in range(5):
            transport.pub("t", bytes([i]))
        self.assertEqual(sub.drain_latest(), bytes([4]))
        self.assertIsNone(sub.try_recv())


class UdpTransportTest(unittest.TestCase):
    def test_loopback_roundtrip(self):
        receiver = UdpTeleopTransport(bind_addr=("127.0.0.1", 0))
        port = receiver._sock.getsockname()[1]
        sender = UdpTeleopTransport(pub_addr=("127.0.0.1", port))
        sub = receiver.sub(TOPIC)
        try:
            sender.pub(TOPIC, b"frame")
            self.assertTrue(_wait_until(lambda: sub.drain_latest() == b"frame"))
        finally:
            sender.close()
            receiver.close()

    def test_requires_an_address(self):
        with self.assertRaises(ValueError):
            UdpTeleopTransport()


class MasterLoopTest(unittest.TestCase):
    def test_publishes_openness_and_zero_torque(self):
        g, fake = make_gripper(start_rad=POS_OPEN_RAD)
        # The fake motor is purely kinematic: it tracks ``q`` regardless of gain,
        # so the master's zero-torque ``q=0`` command would drag it closed.  A
        # real slack jaw does not move under zero gain, so freeze the fake to
        # model that — the opening published is then where the jaws started.
        fake.motor.step = lambda *a, **k: None
        transport = InProcTeleopTransport()
        sub = transport.sub(TOPIC)
        mgr = GripperTeleop(g, transport, "master", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["frames"] > 0))
            openness, position_mm, _force, _ts = decode_frame(sub.drain_latest())
            self.assertAlmostEqual(openness, 1.0, places=3)
            self.assertAlmostEqual(position_mm, travel_mm(g.config), places=3)
            last = fake.frames[-1]
            self.assertEqual((last.kp, last.kd), (0.0, 0.0))
        finally:
            mgr.stop()

    def test_status_reports_position_and_force(self):
        # A caller (an operator UI) shows the jaws' position and force from the
        # status alone — it cannot read the bus itself, the loop owns it.
        g, fake = make_gripper(start_rad=POS_OPEN_RAD)
        fake.motor.step = lambda *a, **k: None
        transport = InProcTeleopTransport()
        sub = transport.sub(TOPIC)
        mgr = GripperTeleop(g, transport, "master", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["frames"] > 0))
            _o, position_mm, force_n, _ts = decode_frame(sub.drain_latest())
            status = mgr.status()
            self.assertAlmostEqual(status["position_mm"], position_mm, places=3)
            self.assertAlmostEqual(status["force_n"], force_n, places=3)
        finally:
            mgr.stop()

    def test_stop_leaves_zero_gravity(self):
        g, fake = make_gripper()
        mgr = GripperTeleop(g, InProcTeleopTransport(), "master", TOPIC,
                            rate_hz=200.0, sleep_fn=_nop_sleep)
        mgr.start()
        self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
        mgr.stop()
        self.assertFalse(mgr.is_running)
        # The final frame is not zero-torque: exit_zero_gravity holds instead.
        self.assertEqual(fake.frames[-1].kp, g.config.kp)
        self.assertFalse(mgr.status()["active"])


class SlaveLoopTest(unittest.TestCase):
    def _slave(self, **kwargs):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, reverse=False)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, **kwargs)
        return g, fake, transport, mgr

    def test_follows_published_openness(self):
        g, fake, transport, mgr = self._slave(align=False, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(1.0, 120.0, 0.0, 0.0))
        target = openness_to_rad(1.0, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(
                lambda: fake.frames and abs(fake.frames[-1].q - target) < 1e-9))
            last = fake.frames[-1]
            self.assertEqual((last.kp, last.kd), (100.0, 2.0))
            self.assertFalse(mgr.status()["stale"])
        finally:
            mgr.stop()

    def test_status_reports_the_followed_position_and_force(self):
        g, fake, transport, mgr = self._slave(align=False, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(0.25, 30.0, -4.5, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["frames"] > 0))
            status = mgr.status()
            self.assertAlmostEqual(status["openness"], 0.25, places=4)
            self.assertAlmostEqual(status["position_mm"], 30.0, places=4)
            self.assertAlmostEqual(status["force_n"], -4.5, places=4)
        finally:
            mgr.stop()

    def test_watchdog_holds_instead_of_relaxing(self):
        g, fake, transport, mgr = self._slave(align=False, watchdog_s=0.05)
        transport.pub(TOPIC, encode_frame(0.8, 96.0, 0.0, 0.0))
        target = openness_to_rad(0.8, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(
                lambda: fake.frames and abs(fake.frames[-1].q - target) < 1e-9))
            # Silence the leader and let the watchdog trip.
            self.assertTrue(_wait_until(lambda: mgr.status()["stale"]))
            before = len(fake.frames)
            self.assertTrue(_wait_until(lambda: len(fake.frames) > before + 5))
            # Still streaming, still on target, still under gain.
            self.assertAlmostEqual(fake.frames[-1].q, target)
            self.assertEqual(fake.frames[-1].kp, 100.0)
        finally:
            mgr.stop()

    def test_align_ramps_to_the_first_frame_instead_of_stepping(self):
        # The align target arrives in a single frame.  Commanding it outright —
        # what ``goto_rad(..., duration=1.0)`` did, because ``duration`` is a
        # deadline and not a ramp — demands ``kp`` times the whole error on the
        # first CAN frame.  At the shipped kp that is a full-torque step
        # wherever the jaws start, which is how a follower broke its own hard
        # stop.  So: the first frame is one schedule step, not the target.
        g, fake, transport, mgr = self._slave(align=True, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(0.5, 60.0, 0.0, 0.0))
        expected = openness_to_rad(0.5, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
            self.assertNotAlmostEqual(fake.frames[0].q, expected, places=3)
            # It still gets there — over a ramp, not a step.
            self.assertTrue(_wait_until(
                lambda: any(abs(f.q - expected) < 1e-9 for f in fake.frames)))
        finally:
            mgr.stop()

    def test_ignores_malformed_payload(self):
        g, fake, transport, mgr = self._slave(align=False, watchdog_s=5.0)
        transport.pub(TOPIC, b"too short")
        transport.pub(TOPIC, encode_frame(0.25, 30.0, 0.0, 0.0))
        target = openness_to_rad(0.25, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(
                lambda: fake.frames and abs(fake.frames[-1].q - target) < 1e-9))
        finally:
            mgr.stop()


class LiteGripTeleopApiTest(unittest.TestCase):
    def test_status_idle(self):
        g, _ = make_gripper()
        self.assertEqual(g.teleop_status(), {"active": False, "mode": None})

    def test_stop_when_idle_is_harmless(self):
        g, _ = make_gripper()
        self.assertEqual(g.teleop_stop(), {"active": False, "mode": None})

    def test_start_then_busy_then_stop(self):
        g, _ = make_gripper()
        transport = InProcTeleopTransport()
        status = g.teleop_start("slave", transport=transport, align=False)
        self.assertTrue(status["active"])
        self.assertEqual(status["mode"], "slave")
        try:
            with self.assertRaises(TeleopBusyError):
                g.teleop_start("slave", transport=transport, align=False)
        finally:
            stopped = g.teleop_stop()
        self.assertFalse(stopped["active"])
        self.assertEqual(g.teleop_status(), {"active": False, "mode": None})

    def test_start_passes_the_torque_limit_through(self):
        g, _ = make_gripper()
        transport = InProcTeleopTransport()
        g.teleop_start("slave", transport=transport, align=False,
                       torque_limit_nm=2.5)
        try:
            self.assertEqual(g._teleop._torque_limit_nm, 2.5)
        finally:
            g.teleop_stop()

    def test_start_defaults_to_the_guard_disabled(self):
        g, _ = make_gripper()
        transport = InProcTeleopTransport()
        g.teleop_start("slave", transport=transport, align=False)
        try:
            self.assertEqual(g._teleop._torque_limit_nm, 0.0)
        finally:
            g.teleop_stop()

    def test_rejects_unknown_mode(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            g.teleop_start("sideways", transport=InProcTeleopTransport())

    def test_start_discovers_udp_transport_and_closes_it(self):
        g, _ = make_gripper()
        g.teleop_start("slave", link="udp", host="127.0.0.1", port=0,
                       align=False, rate_hz=200.0)
        self.assertIsNotNone(g._teleop_transport)
        self.assertIsNotNone(g._teleop_transport._sock)
        g.teleop_stop()
        self.assertIsNone(g._teleop_transport)

    def test_disconnect_stops_teleop(self):
        g, _ = make_gripper()
        g.teleop_start("slave", transport=InProcTeleopTransport(), align=False)
        self.assertIsNotNone(g._teleop)
        g.disconnect()
        self.assertIsNone(g._teleop)

    def test_start_refuses_uncalibrated_gripper(self):
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(TeleopNotReady):
            g.teleop_start("slave", transport=InProcTeleopTransport())

    def test_start_refuses_zenoh_link_without_the_extra(self):
        # Without the optional dependency the failure must name the extra to
        # install, not the bare ``zenoh`` module.
        g, _ = make_gripper()
        from litegrip import gripper as gripper_mod
        with unittest.mock.patch.object(
                gripper_mod, "_zenoh_transport",
                side_effect=ImportError("pip install litegrip[zenoh]")):
            with self.assertRaises(ImportError):
                g.teleop_start("master")

    def test_master_keeps_its_resident_publisher_across_sessions(self):
        """A stop must not close the leader's resident zenoh publisher.

        Registering it as the per-session transport made ``teleop_stop`` close
        it, while ``_teleop_pub`` kept pointing at the dead endpoint — so every
        session after the first published into nothing.  On hardware the first
        pairing delivered frames and every later one delivered none, while the
        leader's own frame counter kept climbing.
        """
        g, _ = make_gripper()
        from litegrip import gripper as gripper_mod
        built = []

        class _FakeLink:
            def __init__(self):
                self.closed = False
                self.puts = 0

            def pub(self, topic, payload):
                self.puts += 1

            def close(self):
                self.closed = True

        def _factory(role, key, port, host):
            built.append(_FakeLink())
            return built[-1]

        with unittest.mock.patch.object(gripper_mod, "_zenoh_transport",
                                        _factory):
            g.teleop_start("master", rate_hz=200.0)
            g.teleop_stop()
            resident = g._teleop_pub
            self.assertIsInstance(resident, _FakeLink)
            self.assertFalse(
                resident.closed,
                "teleop_stop must not close the resident publisher")
            self.assertTrue(_wait_until(lambda: resident.puts > 0))

            g.teleop_start("master", rate_hz=200.0)
            self.assertIs(g._teleop_pub, resident)
            self.assertEqual(len(built), 1,
                             "the listener is built once, not per session")
            g.teleop_stop()
            self.assertFalse(resident.closed)

            g.disconnect()
            self.assertTrue(resident.closed,
                            "disconnect() owns the resident publisher")


class ReadinessTest(unittest.TestCase):
    """The three preconditions that must hold before anything is enabled."""

    def test_accepts_a_calibrated_gripper(self):
        g, _ = make_gripper()
        check_ready(g.config)

    def test_rejects_uncalibrated(self):
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(TeleopNotReady):
            check_ready(g.config)

    def test_rejects_zero_travel(self):
        g, _ = make_gripper()
        g.config.pos_open_rad = g.config.pos_closed_rad
        with self.assertRaises(TeleopNotReady):
            check_ready(g.config)

    def test_rejects_zero_rad_to_mm(self):
        g, _ = make_gripper()
        g.config.rad_to_mm = 0.0
        with self.assertRaises(TeleopNotReady):
            check_ready(g.config)


class ClampToCalibratedTest(unittest.TestCase):
    def test_clamps_for_both_mountings(self):
        for reverse in (False, True):
            g, _ = make_gripper(reverse=reverse)
            cfg = g.config
            lo, hi = sorted((cfg.pos_closed_rad, cfg.pos_open_rad))
            self.assertAlmostEqual(clamp_to_calibrated(cfg, lo - 5.0), lo)
            self.assertAlmostEqual(clamp_to_calibrated(cfg, hi + 5.0), hi)
            mid = (lo + hi) / 2.0
            self.assertAlmostEqual(clamp_to_calibrated(cfg, mid), mid)


class NonFiniteFrameTest(unittest.TestCase):
    """A NaN frame must be dropped, never clamped onto a limit."""

    def test_master_does_not_publish_a_nan_reading(self):
        g, fake = make_gripper(start_rad=POS_OPEN_RAD)
        fake.motor.step = lambda *a, **k: None
        g.get_state = lambda wait=True: types.SimpleNamespace(
            position_rad=float("nan"), position_mm=0.0, force_n=0.0,
            error_code=1)
        transport = InProcTeleopTransport()
        sub = transport.sub(TOPIC)
        mgr = GripperTeleop(g, transport, "master", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["rejected"] > 0))
            self.assertIsNone(sub.drain_latest())
        finally:
            mgr.stop()

    def test_slave_drops_a_nan_frame_and_holds(self):
        g, fake, transport, mgr = SlaveLoopTest()._slave(
            align=False, watchdog_s=5.0)
        hold = clamp_to_calibrated(g.config, POS_CLOSED_RAD)
        transport.pub(TOPIC, encode_frame(float("nan"), 60.0, 0.0, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["rejected"] > 0))
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 3))
            # Every frame is the hold position: the NaN was not folded onto an
            # end stop.
            for frame in fake.frames:
                self.assertAlmostEqual(frame.q, hold)
        finally:
            mgr.stop()


class SlaveGuardTest(unittest.TestCase):
    def test_align_skips_nan_and_keeps_sending_hold_frames(self):
        g, fake, transport, mgr = SlaveLoopTest()._slave(
            align=True, watchdog_s=5.0)
        hold = clamp_to_calibrated(g.config, POS_CLOSED_RAD)
        mgr.start()
        try:
            # Waiting for the first frame: hold frames must keep flowing.
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 3))
            self.assertAlmostEqual(fake.frames[-1].q, hold)
            transport.pub(TOPIC, encode_frame(float("nan"), 60.0, 0.0, 0.0))
            self.assertTrue(_wait_until(lambda: mgr.status()["rejected"] > 0))
            self.assertAlmostEqual(fake.frames[-1].q, hold)
            # A good frame then aligns normally.
            transport.pub(TOPIC, encode_frame(0.5, 60.0, 0.0, 0.0))
            target = openness_to_rad(0.5, g.config)
            self.assertTrue(_wait_until(
                lambda: abs(fake.frames[-1].q - target) < 1e-9))
        finally:
            mgr.stop()

    def test_counts_send_failures(self):
        g, fake, transport, mgr = SlaveLoopTest()._slave(
            align=False, watchdog_s=5.0)
        g.send_mit_frame = lambda **kwargs: False
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["send_failed"] > 0))
        finally:
            mgr.stop()

    def test_reports_the_gripper_fault_code(self):
        g, _ = make_gripper(start_rad=POS_CLOSED_RAD, err=0)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, align=False, watchdog_s=5.0)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: bool(mgr.status()["fault"])))
        finally:
            mgr.stop()

    def test_kp_kd_fall_back_to_the_calibration(self):
        g, fake, transport, mgr = SlaveLoopTest()._slave(
            align=False, watchdog_s=5.0)
        g.config.kp, g.config.kd = 33.0, 4.0
        transport.pub(TOPIC, encode_frame(0.8, 96.0, 0.0, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
            self.assertEqual((fake.frames[-1].kp, fake.frames[-1].kd), (33.0, 4.0))
        finally:
            mgr.stop()

    def test_rejects_non_positive_watchdog(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                          watchdog_s=0.0)


class TorqueGuardTest(unittest.TestCase):
    """The follower releases when its own torque stays over the limit.

    Torque here is the follower's own reading — the status frame's ``tau``,
    derived from coil current — never the leader's ``force_n`` on the wire.
    Position is deliberately not consulted: a jam partway through the travel is
    indistinguishable from slow motion.  The guard is opt-in (``0`` disables it),
    so the loop tests elsewhere run with it off.
    """

    def _mgr(self, **kwargs):
        g, _ = make_gripper()
        return GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                             sleep_fn=_nop_sleep, **kwargs)

    @staticmethod
    def _torque(value: float):
        return types.SimpleNamespace(torque_nm=value)

    def test_trips_only_after_consecutive_over_limit_cycles(self):
        mgr = self._mgr(torque_limit_nm=1.0)
        for _ in range(TORQUE_TRIP_CYCLES - 1):
            mgr._update_torque_guard(self._torque(1.5))
        self.assertFalse(mgr.status()["over_torque"])
        mgr._update_torque_guard(self._torque(1.5))
        self.assertTrue(mgr.status()["over_torque"])
        self.assertEqual(mgr.status()["torque_trips"], 1)

    def test_a_sample_under_the_limit_resets_the_run(self):
        mgr = self._mgr(torque_limit_nm=1.0)
        for _ in range(TORQUE_TRIP_CYCLES - 1):
            mgr._update_torque_guard(self._torque(1.5))
        mgr._update_torque_guard(self._torque(0.1))
        for _ in range(TORQUE_TRIP_CYCLES - 1):
            mgr._update_torque_guard(self._torque(1.5))
        self.assertFalse(mgr.status()["over_torque"])

    def test_latches_until_the_leader_reopens(self):
        mgr = self._mgr(torque_limit_nm=1.0)
        mgr._last_openness = 0.1
        for _ in range(TORQUE_TRIP_CYCLES):
            mgr._update_torque_guard(self._torque(2.0))
        self.assertTrue(mgr.status()["over_torque"])
        # Releasing drops the torque to zero, but that alone must not re-arm —
        # otherwise the guard would chatter.
        mgr._update_torque_guard(self._torque(0.0))
        self.assertTrue(mgr.status()["over_torque"])
        mgr._last_openness = 0.1 + TORQUE_REARM_OPENNESS / 2.0
        mgr._update_torque_guard(self._torque(0.0))
        self.assertTrue(mgr.status()["over_torque"])
        mgr._last_openness = 0.1 + TORQUE_REARM_OPENNESS
        mgr._update_torque_guard(self._torque(0.0))
        self.assertFalse(mgr.status()["over_torque"])

    def test_a_trip_near_full_open_re_arms_at_the_open_stop(self):
        # A trip above ``1 - TORQUE_REARM_OPENNESS`` has no travel left to
        # reopen into, so the uncapped target (trip + margin) is unreachable
        # and the guard latched off for the rest of the session — while its own
        # log line asked for a movement the stroke does not allow.  There the
        # open stop is the target: the most back-off that end has.
        mgr = self._mgr(torque_limit_nm=1.0)
        mgr._last_openness = 0.97
        for _ in range(TORQUE_TRIP_CYCLES):
            mgr._update_torque_guard(self._torque(2.0))
        self.assertTrue(mgr.status()["over_torque"])
        # Releasing drops the torque to zero, but that alone must not re-arm.
        mgr._update_torque_guard(self._torque(0.0))
        self.assertTrue(mgr.status()["over_torque"])
        # 0.97 + 0.05 = 1.02 is past the end of the stroke; anything short of
        # the open stop is still latched.
        mgr._last_openness = 0.99
        mgr._update_torque_guard(self._torque(0.0))
        self.assertTrue(mgr.status()["over_torque"])
        mgr._last_openness = 1.0
        mgr._update_torque_guard(self._torque(0.0))
        self.assertFalse(mgr.status()["over_torque"])

    def test_a_trip_mid_travel_still_needs_the_full_margin(self):
        # The cap only bites near the open stop: a jam partway through the
        # travel still waits for the margin, so backing off barely does not
        # re-engage into the same obstruction.
        mgr = self._mgr(torque_limit_nm=1.0)
        mgr._last_openness = 0.50
        for _ in range(TORQUE_TRIP_CYCLES):
            mgr._update_torque_guard(self._torque(2.0))
        mgr._last_openness = 0.50 + TORQUE_REARM_OPENNESS / 2.0
        mgr._update_torque_guard(self._torque(0.0))
        self.assertTrue(mgr.status()["over_torque"])
        mgr._last_openness = 0.50 + TORQUE_REARM_OPENNESS
        mgr._update_torque_guard(self._torque(0.0))
        self.assertFalse(mgr.status()["over_torque"])

    def test_reports_the_torque_it_saw(self):
        mgr = self._mgr(torque_limit_nm=1.0)
        mgr._update_torque_guard(self._torque(0.42))
        self.assertAlmostEqual(mgr.status()["torque_nm"], 0.42, places=4)

    def test_zero_limit_disables_the_guard(self):
        mgr = self._mgr(torque_limit_nm=0.0)
        for _ in range(TORQUE_TRIP_CYCLES * 3):
            mgr._update_torque_guard(self._torque(9.0))
        self.assertFalse(mgr.status()["over_torque"])
        self.assertEqual(mgr.status()["torque_trips"], 0)

    def test_rejects_a_negative_limit(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                          torque_limit_nm=-0.1)

    def test_slave_releases_in_place_and_keeps_streaming(self):
        # A block partway through the travel, under a command to close: the
        # follower presses it and its torque climbs past the limit.
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=block)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, align=False, watchdog_s=5.0,
                            torque_limit_nm=1.0)
        transport.pub(TOPIC, encode_frame(0.0, 0.0, 0.0, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["over_torque"]))
            tripped = len(fake.frames)
            # Released in place: zero stiffness, zero damping.
            self.assertTrue(_wait_until(
                lambda: fake.frames and fake.frames[-1].kp == 0.0
                and fake.frames[-1].kd == 0.0))
            # Still streaming, so the motor does not latch comm loss.
            self.assertTrue(_wait_until(lambda: len(fake.frames) > tripped + 5))
            self.assertEqual(fake.frames[-1].kp, 0.0)
        finally:
            mgr.stop()

    def test_align_releases_in_place_when_the_guard_trips(self):
        # The align used to run inside a blocking ``goto_rad``, outside the
        # guard's reach: ``torque_limit_nm`` did not cover it, and a jam during
        # the align pressed until the move finished.  It now sends through the
        # loop's own path, so the guard sees it.
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=block)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, align=True, watchdog_s=5.0,
                            torque_limit_nm=1.0)
        transport.pub(TOPIC, encode_frame(0.0, 0.0, 0.0, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(
                lambda: any(f.kp == 0.0 for f in fake.frames)))
        finally:
            mgr.stop()

    def test_slave_releases_in_place_on_the_handoff(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=block)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, align=False, watchdog_s=5.0,
                            torque_limit_nm=1.0)
        transport.pub(TOPIC, encode_frame(0.0, 0.0, 0.0, 0.0))
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["over_torque"]))
        finally:
            mgr.stop()
        # The final frame must not re-apply the gains and press again.
        self.assertEqual(fake.frames[-1].kp, 0.0)
        self.assertEqual(fake.frames[-1].kd, 0.0)


class LeadCapTest(unittest.TestCase):
    """The align command never leads the measurement by more than the cap.

    Torque is ``kp * (q_cmd - q_measured)``, so bounding the lead bounds the
    commanded torque by construction.  This is what keeps the align ramp from
    demanding ``kp`` times a whole-stroke error in a single frame.  The follow
    loop is deliberately *not* capped, so the follower stays responsive — see
    :meth:`test_first_follow_frame_commands_the_target`.
    """

    def _slave(self, **kwargs):
        return SlaveLoopTest()._slave(**kwargs)

    @staticmethod
    def _cap_rad(g) -> float:
        return DEFAULT_LEAD_CAP_MM / g.config.rad_to_mm

    @staticmethod
    def _max_lead(fake, start_rad: float) -> float:
        """Largest distance any command led the position it was sent from."""
        worst, pos = 0.0, start_rad
        for frame in fake.frames:
            worst = max(worst, abs(frame.q - pos))
            pos = frame.pos_after
        return worst

    def test_align_never_leads_more_than_the_cap(self):
        g, fake, transport, mgr = self._slave(align=True, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(1.0, 120.0, 0.0, 0.0))
        target = openness_to_rad(1.0, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(
                lambda: any(abs(f.q - target) < 1e-9 for f in fake.frames)))
            self.assertLessEqual(self._max_lead(fake, POS_CLOSED_RAD),
                                 self._cap_rad(g) + 1e-9)
            # The step this replaces was an order of magnitude past the cap.
            self.assertGreater(abs(target - POS_CLOSED_RAD),
                               10.0 * self._cap_rad(g))
        finally:
            mgr.stop()

    def test_first_follow_frame_commands_the_target(self):
        # The follow loop is deliberately uncapped: it commands the leader's
        # whole opening from the first frame, exactly as it did before the align
        # ramp was added.  Capping it would bound the follow torque but blunt the
        # follower's response, which is not what the align fix was for.
        g, fake, transport, mgr = self._slave(align=False, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(1.0, 120.0, 0.0, 0.0))
        target = openness_to_rad(1.0, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
            self.assertAlmostEqual(fake.frames[0].q, target)
        finally:
            mgr.stop()

    def test_zero_cap_disables_it(self):
        # Same opt-out shape as ``dq_max`` and ``torque_limit_nm``: 0 turns the
        # align's cap off, passing the command through untouched.  The align is
        # still a bounded-speed ramp — that is ``align_speed_mm_s``, a separate
        # knob.
        _g, _fake, _transport, mgr = self._slave(lead_cap_mm=0.0)
        self.assertEqual(mgr._cap_lead(1.0, 0.0), 1.0)
        self.assertEqual(mgr._cap_lead(-1.0, 0.0), -1.0)

    def test_rejects_a_negative_cap(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                          lead_cap_mm=-1.0)

    def test_rejects_a_non_positive_align_speed(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                          align_speed_mm_s=0.0)


class VelocityFeedforwardTest(unittest.TestCase):
    """The leader's velocity is recovered locally and fed forward as ``dq``.

    The gripper wire frame carries no velocity field, so the follower
    differences successive positions.  These bounds keep a bad estimate from
    turning into a large ``kd * (dq - dq_measured)`` torque.
    """

    def _mgr(self, **kwargs):
        g, _ = make_gripper()
        return GripperTeleop(g, InProcTeleopTransport(), "slave", TOPIC,
                             sleep_fn=_nop_sleep, **kwargs)

    def test_first_sample_has_nothing_to_difference(self):
        self.assertEqual(self._mgr()._estimate_dq(0.5, 100.0), 0.0)

    def test_is_a_finite_difference(self):
        mgr = self._mgr()
        mgr._prev_q, mgr._prev_rx_ts = 0.10, 100.00
        self.assertAlmostEqual(mgr._estimate_dq(0.15, 100.01), 5.0)

    def test_is_clamped_to_dq_max(self):
        mgr = self._mgr(dq_max=2.0)
        mgr._prev_q, mgr._prev_rx_ts = 0.0, 100.0
        self.assertEqual(mgr._estimate_dq(5.0, 100.01), 2.0)
        self.assertEqual(mgr._estimate_dq(-5.0, 100.01), -2.0)

    def test_refuses_a_dropout_sized_gap(self):
        mgr = self._mgr()
        mgr._prev_q, mgr._prev_rx_ts = 0.0, 100.0
        self.assertEqual(
            mgr._estimate_dq(0.5, 100.0 + MAX_FRAME_GAP_S + 0.01), 0.0)

    def test_refuses_a_degenerate_interval(self):
        mgr = self._mgr()
        mgr._prev_q, mgr._prev_rx_ts = 0.0, 100.0
        self.assertEqual(mgr._estimate_dq(0.5, 100.0 + 1e-6), 0.0)

    def test_zero_dq_max_disables_the_feedforward(self):
        mgr = self._mgr(dq_max=0.0)
        mgr._prev_q, mgr._prev_rx_ts = 0.0, 100.0
        self.assertEqual(mgr._estimate_dq(0.5, 100.01), 0.0)

    def test_slave_sends_the_leader_velocity_as_dq(self):
        # Paced by the real clock, so consecutive frames land a realistic
        # interval apart — the recovered velocity is then far past ``dq_max``
        # and clamps, which is stable against the exact timing.
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, reverse=False)
        transport = _PreSubTransport()
        mgr = GripperTeleop(g, transport, "slave", TOPIC, rate_hz=100.0,
                            align=False, watchdog_s=5.0)
        transport.pub(TOPIC, encode_frame(0.2, 24.0, 0.0, 0.0))
        mgr.start()
        try:
            # The first frame has nothing to difference against: a position
            # command with the feed-forward at zero.
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
            self.assertEqual(fake.frames[0].dq, 0.0)
            # A frame far away, one loop period later.  Opening up drives the
            # normal-mount jaw toward a smaller angle, so the fed-forward
            # velocity is negative — and bounded by ``dq_max``.
            transport.pub(TOPIC, encode_frame(0.8, 96.0, 0.0, 0.0))
            self.assertTrue(_wait_until(
                lambda: any(f.dq == -DEFAULT_DQ_MAX for f in fake.frames)))
            # The target is still reached; the cap only paces the approach.
            q2 = openness_to_rad(0.8, g.config)
            self.assertTrue(_wait_until(
                lambda: any(abs(f.q - q2) < 1e-9 for f in fake.frames)))
        finally:
            mgr.stop()


if __name__ == "__main__":
    unittest.main()
