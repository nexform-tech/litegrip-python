"""Tests for the point-to-point zenoh link.

Skipped entirely when the optional ``zenoh`` dependency is not installed — the
base SDK must stay usable without it, so this module is not part of the
mandatory suite.

The loopback cases open real sockets.  A hang here would hang the suite, so the
suite is expected to be run under a timeout (``timeout 300 python3 -m unittest``
— which is what CI does); that timeout is the guard the design's "must call
close() or the process never exits" rule is checked against.
"""

from __future__ import annotations

import socket
import time
import unittest

import _sdkpath  # noqa: F401

try:
    import zenoh  # noqa: F401
    HAVE_ZENOH = True
except ImportError:                                   # pragma: no cover
    HAVE_ZENOH = False

if HAVE_ZENOH:
    from litegrip.teleop import (DEFAULT_GRIP_PORT, TeleopError, teleop_topic)
    from litegrip.zenoh_link import (Connector, LatestSlot, Listener,
                                     ZenohTeleopTransport, _base_config)

KEY = teleop_topic("gripA")
WAIT_S = 5.0


def _wait_until(predicate, timeout_s: float = WAIT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _free_port() -> int:
    """A port nothing is listening on right now (close it, then hand it over)."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@unittest.skipUnless(HAVE_ZENOH, "zenoh is not installed")
class ConfigTest(unittest.TestCase):
    def test_discovery_is_off_and_mode_is_peer(self):
        # The whole point of the link: no broadcast discovery, explicit
        # endpoints only.
        cfg = _base_config()
        self.assertEqual(cfg.get_json("scouting/multicast/enabled"), "false")
        self.assertEqual(cfg.get_json("scouting/gossip/enabled"), "false")
        # zenoh reports JSON values verbatim, so the mode arrives quoted.
        self.assertEqual(cfg.get_json("mode"), '"peer"')


@unittest.skipUnless(HAVE_ZENOH, "zenoh is not installed")
class LatestSlotTest(unittest.TestCase):
    def test_never_received_is_not_zero(self):
        slot = LatestSlot()
        self.assertFalse(slot.ever_received)
        # ``None``, not ``0.0``: ``0.0`` is a legitimate age ("just arrived"),
        # and using it for "never received" voids the startup diagnostic.
        self.assertIsNone(slot.peek_age(100.0))
        self.assertEqual(slot.take(), (None, None))

    def test_take_does_not_clear_the_timestamp(self):
        slot = LatestSlot()
        slot.put(b"a", 10.0)
        self.assertTrue(slot.ever_received)
        self.assertEqual(slot.take(), (b"a", 10.0))
        # Taking the payload must not erase "when we last received".
        self.assertEqual(slot.take(), (None, 10.0))
        self.assertTrue(slot.ever_received)
        self.assertAlmostEqual(slot.peek_age(10.5), 0.5)

    def test_age_is_clamped_at_zero(self):
        # The loop may read ``now`` before a frame carrying a larger ``now``
        # lands; a negative age is a hair away from a spurious watchdog trip.
        slot = LatestSlot()
        slot.put(b"a", 10.0)
        self.assertEqual(slot.peek_age(9.0), 0.0)

    def test_latest_wins_and_dropped_counts_overwrites(self):
        slot = LatestSlot()
        slot.put(b"old", 1.0)
        slot.put(b"new", 2.0)
        self.assertEqual(slot.dropped, 1)
        self.assertEqual(slot.take(), (b"new", 2.0))


@unittest.skipUnless(HAVE_ZENOH, "zenoh is not installed")
class LoopbackTest(unittest.TestCase):
    def test_listener_to_connector_roundtrip(self):
        port = _free_port()
        slot = LatestSlot()
        listener = Listener(port, KEY)
        connector = Connector("127.0.0.1", port, KEY,
                             on_frame=lambda p: slot.put(p, time.monotonic()))
        try:
            self.assertTrue(_wait_until(lambda: listener.matching),
                            "publisher never matched a subscriber")
            listener.put(b"frame-1")
            self.assertTrue(_wait_until(lambda: slot.take()[0] == b"frame-1"))
            self.assertEqual(connector.received, 1)
        finally:
            connector.close()
            listener.close()

    def test_transport_pub_to_sub(self):
        port = _free_port()
        master = ZenohTeleopTransport("master", KEY, port=port)
        slave = ZenohTeleopTransport("slave", KEY, port=port, host="127.0.0.1")
        try:
            sub = slave.sub(KEY)
            self.assertTrue(_wait_until(lambda: master.matching))
            master.pub(KEY, b"payload")
            self.assertTrue(_wait_until(lambda: sub.try_recv() == b"payload"))
            # Latest wins: a subscriber that fell behind skips history.
            master.pub(KEY, b"older")
            master.pub(KEY, b"newer")
            self.assertTrue(_wait_until(lambda: sub.try_recv() == b"newer"))
        finally:
            slave.close()
            master.close()

    def test_roles_are_one_way(self):
        port = _free_port()
        master = ZenohTeleopTransport("master", KEY, port=port)
        try:
            with self.assertRaises(TeleopError):
                master.sub(KEY)
        finally:
            master.close()

    def test_topic_must_match_the_key(self):
        port = _free_port()
        master = ZenohTeleopTransport("master", KEY, port=port)
        try:
            with self.assertRaises(TeleopError):
                master.pub("some/other/topic", b"x")
        finally:
            master.close()

    def test_slave_requires_a_host(self):
        with self.assertRaises(ValueError):
            ZenohTeleopTransport("slave", KEY, port=_free_port())

    def test_close_is_idempotent(self):
        master = ZenohTeleopTransport("master", KEY, port=_free_port())
        master.close()
        master.close()


@unittest.skipUnless(HAVE_ZENOH, "zenoh is not installed")
class DefaultsTest(unittest.TestCase):
    def test_default_port_is_the_gripper_port(self):
        self.assertEqual(DEFAULT_GRIP_PORT, 17448)


if __name__ == "__main__":
    unittest.main()
