"""``MotorController.poll`` must drain the RX queue; the newest frame wins.

The motor answers every control frame with a status frame.  A caller that
sends a burst (a force-hold slice, a ramp step) and polls once per burst leaves
the answers queued; reading one frame returns an old position, and the age is
stamped when it is read, so the stale reading looks fresh.  On 2026-11-18 the
ramp after a hold started from the hold's first pose (78 mm) while the jaws
were at 26 mm — the first frame demanded ~0.7 rad, and the drive left at
23 rad/s out of a 20 mm/s move.
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip.can.controller import MotorController
from litegrip.can.protocol import float_to_uint
from litegrip.can.transport import CanFrame

MST_ID = 0x18
CAN_ID = 0x08


def status_frame(position_rad: float, err: int = 1) -> CanFrame:
    """Build one DM status frame (q, dq=0, tau=0)."""
    q_uint = float_to_uint(position_rad, -12.5, 12.5, 16)
    dq_uint = float_to_uint(0.0, -30.0, 30.0, 12)
    tau_uint = float_to_uint(0.0, -10.0, 10.0, 12)
    data = bytes([
        ((err & 0x0F) << 4) | (CAN_ID & 0x0F),
        (q_uint >> 8) & 0xFF,
        q_uint & 0xFF,
        (dq_uint >> 4) & 0xFF,
        ((dq_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
        tau_uint & 0xFF,
        30, 30,
    ])
    return CanFrame(can_id=MST_ID, data=data)


class QueueTransport:
    """A transport that hands out pre-queued frames, then goes quiet."""

    def __init__(self, frames):
        self._frames = list(frames)
        self.sent = []

    def recv(self, timeout_s: float = 0.0):
        return self._frames.pop(0) if self._frames else None

    def send(self, can_id, data, timeout=0.1):
        self.sent.append((can_id, data))

    def send_multi(self, can_id, data, count=1, interval_s=0.002):
        for _ in range(count):
            self.send(can_id, data)

    def set_id_filter(self, ids):
        pass


class TestPollDrainsTheQueue(unittest.TestCase):

    def make_controller(self, frames):
        transport = QueueTransport(frames)
        controller = MotorController(transport)
        motor = controller.add_motor(can_id=CAN_ID, mst_id=MST_ID)
        return controller, motor

    def test_the_newest_queued_frame_wins(self):
        controller, motor = self.make_controller([
            status_frame(-1.0),
            status_frame(-0.5),
            status_frame(-0.2),
        ])

        updated = controller.poll()

        self.assertIs(updated, motor)
        self.assertEqual(motor.rx_count, 3, "队列没有被排空")
        self.assertAlmostEqual(motor.position, -0.2, places=3,
                               msg="读到的还是队列里最旧的那一帧")

    def test_foreign_frames_between_status_frames_do_not_stop_the_drain(self):
        foreign = CanFrame(can_id=0x123, data=bytes(8))
        controller, motor = self.make_controller([
            status_frame(-1.0),
            foreign,
            status_frame(-0.3),
        ])

        controller.poll()

        self.assertEqual(motor.rx_count, 2)
        self.assertAlmostEqual(motor.position, -0.3, places=3)

    def test_a_single_frame_is_still_reported(self):
        controller, motor = self.make_controller([status_frame(0.25)])

        updated = controller.poll()

        self.assertIs(updated, motor)
        self.assertEqual(motor.rx_count, 1)
        self.assertAlmostEqual(motor.position, 0.25, places=3)

    def test_an_empty_queue_returns_none(self):
        controller, _ = self.make_controller([])

        self.assertIsNone(controller.poll())


if __name__ == "__main__":
    unittest.main()
