"""硬件无关的假 CAN 层 —— 让 actions 引擎能在没有夹爪的情况下被验证。

模型是**纯运动学**的：位置朝指令走（有限速度上限），可选机械挡块、粘滑量化
和故障注入。不建动力学、不建阻尼，``tau`` 只按 ``kp*(q - pos) + tau_ff``
（限 ±10 Nm）报出来 —— 够用来守住「指令领先上限 ⇒ 力矩有界」这条逻辑，
守不住真实动力学（那部分的数值来源是真机实测）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import _sdkpath  # noqa: F401  (把 src/ 插进 sys.path)
from litegrip import LiteGrip, MotionConfig

# 用户真机上跑出来的标定值
POS_CLOSED_RAD = 0.104334
POS_OPEN_RAD = -1.513123
RAD_TO_MM = 74.19

# 假 CAN 一帧的时间步长（引擎的 --frame-interval 默认值）
DT = 0.005


def tick_clock(step: float):
    """每次调用前进 ``step`` 的假单调时钟（给 hold 的截止时间用）。

    真 ``time.monotonic`` 配合 ``sleep_fn=lambda _: None`` 会让保力循环
    在几毫秒内空转成千上万圈，所以要一个会走的假钟。
    """
    t = [0.0]

    def fn() -> float:
        t[0] += step
        return t[0]

    return fn


@dataclass
class Frame:
    """一帧下发的 MIT 指令 + 该帧执行后的实测值。"""

    q: float
    kp: float
    kd: float
    dq: float
    tau_ff: float
    pos_after: float
    tau_nm: float


class FakeMotor:
    """运动学电机：``pos`` 朝 ``q`` 走，被挡块/速度上限约束。"""

    TAU_MAX = 10.0
    VMAX = 5.0          # rad/s
    GAIN = 50.0         # 1/s，纯跟随增益

    def __init__(self, pos: float = 0.0, block_rad: Optional[float] = None,
                 sticky_rad: float = 0.0, err: int = 1,
                 limit_lo: Optional[float] = None,
                 limit_hi: Optional[float] = None):
        self.pos = pos
        self.vel = 0.0
        self.tau = 0.0
        self.err = err
        self.block_rad = block_rad
        self.sticky_rad = sticky_rad
        # 两端机械限位（真实夹爪本来就有）。block_rad 是单向的「障碍物」，
        # 用来模拟行程中途被挡住。
        self.limit_lo = limit_lo
        self.limit_hi = limit_hi
        self._pos0 = pos
        self._block_dir = 0.0
        if block_rad is not None:
            self._block_dir = 1.0 if block_rad > pos else -1.0

    def reported_pos(self) -> float:
        """上报位置（可量化 —— 模拟闭合侧约 0.0103 rad 的粘滑死区）。"""
        if self.sticky_rad > 0:
            return round(self.pos / self.sticky_rad) * self.sticky_rad
        return self.pos

    def set_block(self, block_rad: Optional[float]) -> None:
        """在当前位置的前方装/拆机械挡块。"""
        self.block_rad = block_rad
        self._block_dir = (0.0 if block_rad is None
                           else (1.0 if block_rad > self.pos else -1.0))

    def step(self, q: float, kp: float, dq: float, tau_ff: float, dt: float) -> None:
        v = dq + self.GAIN * (q - self.pos)
        v = max(-self.VMAX, min(self.VMAX, v))
        new = self.pos + v * dt
        if self.block_rad is not None:
            if self._block_dir > 0:
                new = min(new, self.block_rad)
            else:
                new = max(new, self.block_rad)
        if self.limit_lo is not None:
            new = max(new, self.limit_lo)
        if self.limit_hi is not None:
            new = min(new, self.limit_hi)
        self.vel = (new - self.pos) / dt
        self.pos = new
        self.tau = max(-self.TAU_MAX,
                       min(self.TAU_MAX, kp * (q - self.pos) + tau_ff))


class FakeLiteGripCAN:
    """鸭子类型的 ``LiteGripCAN`` —— 只实现 LiteGrip 会调到的那几个方法。"""

    def __init__(self, pos: float = 0.0, block_rad: Optional[float] = None,
                 sticky_rad: float = 0.0, err: int = 1,
                 initialize_results: Optional[List[bool]] = None,
                 limit_lo: Optional[float] = None,
                 limit_hi: Optional[float] = None):
        self.motor = FakeMotor(pos=pos, block_rad=block_rad,
                               sticky_rad=sticky_rad, err=err,
                               limit_lo=limit_lo, limit_hi=limit_hi)
        self.frames: List[Frame] = []
        self.initialize_results = list(initialize_results or [])
        self.initialize_calls = 0
        self.clear_fault_calls = 0
        self.disconnected = False
        self.error_when_holding: Optional[int] = None
        self.holding = False

    # ── 运动 ───────────────────────────────────────────────────────────
    def control_mit(self, q_target, kp, kd, dq_target=0.0,
                    tau_feedforward=0.0) -> bool:
        self.motor.step(q_target, kp, dq_target, tau_feedforward, DT)
        self.frames.append(Frame(
            q=q_target, kp=kp, kd=kd, dq=dq_target, tau_ff=tau_feedforward,
            pos_after=self.motor.pos, tau_nm=self.motor.tau))
        self.holding = tau_feedforward != 0.0
        return True

    # ── 状态 ───────────────────────────────────────────────────────────
    def poll(self, timeout_s: float = 0.0) -> bool:
        return True

    def update_state(self, timeout_s: float = 0.05) -> bool:
        return True

    def get_position(self) -> float:
        return self.motor.reported_pos()

    def get_velocity(self) -> float:
        return self.motor.vel

    def get_torque(self) -> float:
        return self.motor.tau

    def get_error(self) -> int:
        if self.error_when_holding is not None and self.holding:
            return self.error_when_holding
        return self.motor.err

    def get_temperature(self):
        return 30, 35

    # ── 使能 ───────────────────────────────────────────────────────────
    def disable(self) -> bool:
        self.motor.err = 0
        return True

    def clear_fault(self) -> bool:
        self.clear_fault_calls += 1
        self.motor.err = 0
        return True

    def initialize(self) -> bool:
        """按 ``initialize_results`` 脚本回答；成功时把 err 置 1。

        没给脚本就当成一台好电机（返回 True）。
        """
        self.initialize_calls += 1
        ok = (self.initialize_results.pop(0)
              if self.initialize_results else True)
        if ok:
            self.motor.err = 1
        return ok

    def read_param(self, rid, timeout_s: float = 0.5) -> float:
        return 0.0

    def disconnect(self, disable: bool = True) -> None:
        self.disconnected = True


def make_gripper(
    start_rad: float = POS_OPEN_RAD,
    block_rad: Optional[float] = None,
    sticky_rad: float = 0.0,
    err: int = 1,
    motion: Optional[MotionConfig] = None,
    initialize_results: Optional[List[bool]] = None,
    stops: bool = False,
):
    """造一个「已连接、已使能」的 LiteGrip，底层换成 :class:`FakeLiteGripCAN`。

    默认用用户真机标定值（``POS_CLOSED_RAD`` / ``POS_OPEN_RAD`` / ``RAD_TO_MM``），
    ``sleep_fn`` 置空 —— 整条斜坡瞬间跑完。

    ``stops=True`` 在标定出的两端装机械限位 —— 真夹爪本来就有，而
    ``open``/``close`` 现在靠撞它来结束运动，所以这类用例必须开。

    Returns:
        ``(gripper, fake_can)``
    """
    g = LiteGrip("vcan0", can_id=0x08)
    cfg = g.config
    cfg.pos_closed_rad = POS_CLOSED_RAD
    cfg.pos_open_rad = POS_OPEN_RAD
    cfg.rad_to_mm = RAD_TO_MM

    fake = FakeLiteGripCAN(pos=start_rad, block_rad=block_rad,
                           sticky_rad=sticky_rad, err=err,
                           initialize_results=initialize_results,
                           limit_lo=POS_OPEN_RAD if stops else None,
                           limit_hi=POS_CLOSED_RAD if stops else None)
    g._can = fake
    g._connected = True
    g._enabled = True
    g.motion_config = motion if motion is not None else MotionConfig(
        sleep_fn=lambda _: None)
    return g, fake


def planned_steps(g, speed_mm_s: float, toward: str = "close", press: bool = False):
    """按引擎的算术复算一遍 ``(目标 rad, ramp_steps, settle_steps, 总帧数)``。

    ``press=True`` 要跟着走 :func:`press_target` —— open/close 已改成顶限位，
    用 ``limit_target`` 复算会得到错的帧数。

    测试里当「对照答案」用。
    """
    from litegrip.actions import limit_target, press_target

    cfg = g.motion_config
    gcfg = g.config
    if press:
        target, _, _, _ = press_target(gcfg, toward, cfg.press_overshoot)
    else:
        target, _, _, _ = limit_target(gcfg, toward, cfg.margin)
    start = g._can.get_position()
    dist_mm = abs(target - start) * gcfg.rad_to_mm
    ramp_steps = max(1, int(round((dist_mm / speed_mm_s) / cfg.frame_interval)))
    settle_steps = max(1, int(round(cfg.settle_s / cfg.frame_interval)))
    return target, ramp_steps, settle_steps, ramp_steps + settle_steps
