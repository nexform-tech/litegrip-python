"""CAN 编码器的数值边界：非有限值不能变成字节。

MIT 帧里的每一格都是有量程的整数（q 16 bit、dq/kp/kd/tau 各 12 bit），而
``float_to_uint`` 会把超量程的**有限**值钳到量程端点 —— 那是这个编解码器既定
的饱和语义。非有限值不一样：NaN 在 ``min`` / ``max`` 的比较里全为假，钳位会
把它抬到量程的**一端**（q → +12.5 rad、kp → 500、tau → +10 Nm），也就是一个
调用方没要过的真实指令压到电机上。所以这里守两条：

- 有限值照旧钳位、照旧往返（既有语义不许被这次改动改掉）；
- NaN / ±inf 一律抛 ``ValueError``，任何一格都不例外。
"""

from __future__ import annotations

import math
import unittest

import _sdkpath  # noqa: F401
from litegrip.can.protocol import float_to_uint, pack_mit_frame, uint_to_float

NON_FINITE = (float("nan"), float("inf"), float("-inf"))


class TestFloatToUint(unittest.TestCase):

    def test_finite_values_round_trip_within_one_count(self):
        for bits, lo, hi in ((16, -12.5, 12.5), (12, -30.0, 30.0),
                             (12, 0.0, 500.0), (12, -10.0, 10.0)):
            count = (1 << bits) - 1
            for value in (lo, hi, 0.0, lo + (hi - lo) / 3,
                          hi - (hi - lo) / 7):
                back = uint_to_float(float_to_uint(value, lo, hi, bits),
                                     lo, hi, bits)
                self.assertLessEqual(abs(back - value), (hi - lo) / count,
                                     f"{value} in [{lo}, {hi}] @ {bits} bit")

    def test_out_of_range_finite_values_still_clamp(self):
        """钳位是编解码器的既定语义，这次改动只针对非有限值。"""
        self.assertEqual(float_to_uint(99.0, -12.5, 12.5, 16), 0xFFFF)
        self.assertEqual(float_to_uint(-99.0, -12.5, 12.5, 16), 0x0000)
        self.assertEqual(float_to_uint(9.0, 0.0, 5.0, 12), 0xFFF)

    def test_endpoints_are_the_ends_of_the_range(self):
        self.assertEqual(float_to_uint(-12.5, -12.5, 12.5, 16), 0x0000)
        self.assertEqual(float_to_uint(12.5, -12.5, 12.5, 16), 0xFFFF)

    def test_non_finite_is_refused_for_every_field_width(self):
        for bad in NON_FINITE:
            for bits, lo, hi in ((16, -12.5, 12.5), (12, 0.0, 500.0)):
                with self.assertRaises(ValueError):
                    float_to_uint(bad, lo, hi, bits)

    def test_the_refusal_names_the_value_it_refused(self):
        with self.assertRaises(ValueError) as ctx:
            float_to_uint(float("nan"), -12.5, 12.5, 16)
        self.assertIn("nan", str(ctx.exception).lower())


class TestPackMitFrame(unittest.TestCase):

    #: 一帧完全落在量程内的合法指令。
    GOOD = dict(q=0.5, dq=1.0, kp=100.0, kd=2.0, tau=1.0)

    def test_pack_returns_eight_bytes(self):
        frame = pack_mit_frame(**self.GOOD)
        self.assertIsInstance(frame, bytes)
        self.assertEqual(len(frame), 8)

    def test_every_field_refuses_non_finite(self):
        """逐格验：任何一格非有限，整帧都不许编出来。

        每次只毒化一格，其余保持合法 —— 这样断言的是那一格自己的拒绝路径，
        而不是被别的格顺带拦下。
        """
        for name in self.GOOD:
            for bad in NON_FINITE:
                with self.assertRaises(ValueError):
                    pack_mit_frame(**dict(self.GOOD, **{name: bad}))

    def test_nan_is_refused_instead_of_being_answered_with_the_range_end(self):
        """钉住这次改动拦下的具体后果：NaN 曾经被钳到量程上端。

        量程端点的**有限**值至今照样钳到端点（编解码器的既定语义），所以这里
        摆出对面那一帧作对照：``q = +12.5`` 给的正是 NaN 曾经得到的那一帧
        （16 bit 全 1），``tau = +10`` 同理（12 bit 全 1）。非有限值现在不再
        走那条路，而是抛错。
        """
        top = pack_mit_frame(**dict(self.GOOD, q=12.5, tau=10.0))
        self.assertEqual(top[0:2], b"\xff\xff", "q 量程上端不是 16 bit 全 1")
        self.assertEqual(((top[6] & 0x0F) << 8) | top[7], 0xFFF,
                         "tau 量程上端不是 12 bit 全 1")

        for field in ("q", "tau"):
            with self.assertRaises(ValueError):
                pack_mit_frame(**dict(self.GOOD, **{field: float("nan")}))

    def test_finite_in_range_frames_are_unaffected(self):
        """合法帧照旧编出来，q 那一格还能解回去 —— 闸门只加在非有限值上。"""
        frame = pack_mit_frame(**self.GOOD)
        q_uint = (frame[0] << 8) | frame[1]
        self.assertTrue(math.isfinite(
            uint_to_float(q_uint, -12.5, 12.5, 16)))
        self.assertAlmostEqual(
            uint_to_float(q_uint, -12.5, 12.5, 16), self.GOOD["q"], delta=1e-3)


if __name__ == "__main__":
    unittest.main()
