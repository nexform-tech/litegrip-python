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
import unittest

import _sdkpath  # noqa: F401
from litegrip import (FRAME_SIZE, GripperTeleop, InProcTeleopTransport,
                      TeleopBusyError, UdpTeleopTransport, decode_frame,
                      encode_frame, teleop_topic)
from litegrip.teleop import openness_to_rad, rad_to_openness, travel_mm

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM, make_gripper

TOPIC = teleop_topic("master")

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

    def test_align_uses_first_frame_before_following(self):
        g, fake, transport, mgr = self._slave(align=True, watchdog_s=5.0)
        # Queued before start; the align step must be the first CAN traffic.
        transport.pub(TOPIC, encode_frame(0.5, 60.0, 0.0, 0.0))
        expected = openness_to_rad(0.5, g.config)
        mgr.start()
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 0))
            self.assertAlmostEqual(fake.frames[0].q, expected)
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

    def test_rejects_unknown_mode(self):
        g, _ = make_gripper()
        with self.assertRaises(ValueError):
            g.teleop_start("sideways", transport=InProcTeleopTransport())

    def test_start_discovers_udp_transport_and_closes_it(self):
        g, _ = make_gripper()
        g.teleop_start("slave", host="127.0.0.1", port=0, align=False,
                       rate_hz=200.0)
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


if __name__ == "__main__":
    unittest.main()
