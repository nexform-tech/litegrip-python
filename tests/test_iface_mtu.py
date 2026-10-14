"""``_iface_mtu()`` 交给内核的缓冲区必须装得下整个 ``struct ifreq``。

``SIOCGIFMTU`` 回写的是一整个 ``struct ifreq``（16 字节名字 + 一个必须装得下
所有成员的联合体，64 位 Linux 上 40 字节），不是「名字 + mtu」那 18 字节。
多写出去的部分在 Python 3.14 之前没人看 —— 3.14 起 ``fcntl.ioctl()`` 会在参数
末尾压一枚哨兵字节，内核写过去就抛 ``SystemError: buffer overflow``
（cpython c2eaeee3）。于是 18 字节的缓冲区在 3.14 上直接让
``CanTransport.open()`` 炸掉：真机连不上，而模拟路径根本不走 ``_iface_mtu()``，
所以自检全绿、夹爪失联，报出来的还只是上层那句笼统的「连接失败」。

所以这里钉两条：交给 ioctl 的缓冲区够大，以及真 ioctl 仍然读得出网卡的 MTU。
第一条用假 ioctl 量缓冲区长度，任何解释器上都测得出这个回归；第二条要真内核，
而 3.14 上它抛的是 ``SystemError``（不是 ``OSError``），一路冒到 ``connect()``。
"""

from __future__ import annotations

import errno
import socket
import struct
import sys
import unittest
import unittest.mock

import _sdkpath  # noqa: F401
from litegrip.can import transport

_SIOCGIFMTU = 0x8921


class TestIfreqBufferSize(unittest.TestCase):
    """交给内核的缓冲区按整个 ifreq 给。"""

    def _sock(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sock.close)
        return sock

    def test_buffer_holds_a_whole_ifreq(self):
        """内核照原样回写整个 ifreq，所以缓冲区得有那么长。"""
        seen = {}

        def fake_ioctl(fd, request, arg):
            seen["request"] = request
            seen["len"] = len(arg)
            seen["iface"] = struct.unpack("16s", arg[:16])[0].rstrip(b"\x00")
            # 内核把整结构回写：ifr_mtu 是 ifr_ifru 顶上的那个 int。
            return arg[:16] + struct.pack("=i", 72).ljust(len(arg) - 16, b"\x00")

        with unittest.mock.patch.object(transport.fcntl, "ioctl", fake_ioctl):
            mtu = transport._iface_mtu(self._sock(), "can0")

        self.assertEqual(seen["request"], _SIOCGIFMTU)
        self.assertEqual(seen["iface"], b"can0")
        self.assertGreaterEqual(
            seen["len"], 40,
            "SIOCGIFMTU 回写整个 struct ifreq（64 位 40 字节，32 位 32 字节）；"
            "缓冲区短了内核就写到尾巴外面去，3.14 上判 buffer overflow")
        # 回写比传进去的长不出来，但传进去的长了就得切回前面 18 字节。
        self.assertEqual(mtu, 72)

    def test_returns_none_when_the_ioctl_fails(self):
        """读不出来只是没有提示，不该把 connect() 带下去。"""

        def failing_ioctl(fd, request, arg):
            raise OSError(errno.ENODEV, "no such device")

        with unittest.mock.patch.object(transport.fcntl, "ioctl", failing_ioctl):
            self.assertIsNone(transport._iface_mtu(self._sock(), "can0"))


@unittest.skipUnless(sys.platform.startswith("linux"), "SIOCGIFMTU 是 Linux 的东西")
class TestAgainstTheRealKernel(unittest.TestCase):
    """真 ioctl 这一条。"""

    def _sysfs_mtu(self, iface: str) -> int:
        with open("/sys/class/net/%s/mtu" % iface, "r") as fh:
            return int(fh.read())

    def test_every_interface_reads_back_its_mtu(self):
        """网卡挨个读一遍：3.14 上旧写法在这里就炸，而 3.10 看不出来。"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sock.close)

        checked = 0
        for _, iface in socket.if_nameindex():
            expected = self._sysfs_mtu(iface)
            mtu = transport._iface_mtu(sock, iface)
            self.assertIsNotNone(mtu, "%s: ioctl 没读出来" % iface)
            # ifr_mtu 是 int，而这里只取低 16 位：环回的 65536 读成 0。
            if expected <= 0xFFFF:
                self.assertEqual(mtu, expected, iface)
            checked += 1
        self.assertGreater(checked, 0, "if_nameindex() 一个网卡都没报出来")


if __name__ == "__main__":
    unittest.main()
