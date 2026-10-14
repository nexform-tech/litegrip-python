"""硬件无关的假 CAN 层 —— 让 actions 引擎能在没有夹爪的情况下被验证。

模型是**纯运动学**的：位置朝指令走（有限速度上限），可选机械挡块、粘滑量化
和故障注入。不建动力学、不建阻尼，``tau`` 只按 ``kp*(q - pos) + tau_ff``
（限 ±10 Nm）报出来 —— 够用来守住「指令领先上限 ⇒ 力矩有界」这条逻辑，
守不住真实动力学（那部分的数值来源是真机实测）。

工件有两种：硬挡块（``block_rad``，顶住就不动），和受载后缓慢让位的工件
（``yield_rad_s`` + ``yield_tau_nm`` —— 载荷过门槛就按给定速度退让，载荷掉回
门槛以下就自己停）。后者用来守住「保力段不给增益，所以力不随工件屈服而衰减」
这条逻辑。

长度上还有一条**纯前馈力矩下的蠕变**（``creep_rad_s_per_nm``，默认 0）：位置项
按 ``kp`` 缩放，所以保力帧（``kp = kd = 0``）在默认模型里一点不动 —— 而真机上
闭合侧没有限位时，恒定前馈力矩会把夹爪一路推走（2026-10-10 那台走了 104.9 mm）。
它和 ``yield_rad_s`` 同一路数：给定速率，不建动力学。0 为默认，所以别的用例逐位
不变；打开它是为了让力控段的位置护栏能被测到。
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import List, Optional

import _sdkpath  # noqa: F401  (把 src/ 插进 sys.path)
from litegrip import LiteGrip, MotionConfig

# 用户真机上跑出来的标定值（正装：rad 增大 = 闭合）
POS_CLOSED_RAD = 0.104334
POS_OPEN_RAD = -1.513123
RAD_TO_MM = 74.19

# 两端机械限位在**物理上**是固定的一对，换个装法只是把哪个数值叫「闭合」换过来。
LIMIT_LO_RAD = POS_OPEN_RAD      # 数值小的一端
LIMIT_HI_RAD = POS_CLOSED_RAD    # 数值大的一端

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
    KP_NOMINAL = 100.0  # GAIN 对应的位置刚度

    def __init__(self, pos: float = 0.0, block_rad: Optional[float] = None,
                 sticky_rad: float = 0.0, err: int = 1,
                 limit_lo: Optional[float] = None,
                 limit_hi: Optional[float] = None,
                 yield_rad_s: float = 0.0, yield_tau_nm: float = 0.0,
                 creep_rad_s_per_nm: float = 0.0,
                 report_lag_ticks: int = 0):
        # 读数延迟（帧）。真机上 ``get_state(wait=False)`` 拿的是缓存读数，可能
        # 落后几帧；而力控接近段的指令是 ``min(排程, 读数 + 领先上限)``，领先只
        # 有零点几毫米时，``速度 × 延迟`` 就把它吃光了 —— 夹爪于是推不动排程。
        # 默认 0：不延迟，别的用例逐位不变。
        self.report_lag_ticks = max(0, int(report_lag_ticks))
        # 环形缓冲：最近 ``lag + 1`` 个物理位置，末位是当前值。``pos`` 是带
        # setter 的属性，写它就会推进来 —— 测试里直接改 ``motor.pos`` 也走这条
        # 路，否则上报值会停在上一次 ``step`` 的快照上。
        self._hist: List[float] = [pos] * (self.report_lag_ticks + 1)
        self._pos = pos
        self.vel = 0.0
        self.tau = 0.0
        self.err = err
        self.block_rad = block_rad
        self.sticky_rad = sticky_rad
        # 工件受载让位：在 block_rad 处被顶住、净力矩还在往里压、且大小过了
        # yield_tau_nm 时，block_rad 按 yield_rad_s 朝受力方向退让。0 = 硬挡块。
        self.yield_rad_s = yield_rad_s
        self.yield_tau_nm = yield_tau_nm
        # 纯前馈力矩下的蠕变 rad/s per Nm：真机上闭合侧没有限位时，恒定前馈
        # 力矩会把夹爪一路推走。0 = 不动（保力帧在默认模型里的行为）。
        self.creep_rad_s_per_nm = creep_rad_s_per_nm
        # 两端机械限位（真实夹爪本来就有）。block_rad 是单向的「障碍物」，
        # 用来模拟行程中途被挡住。
        self.limit_lo = limit_lo
        self.limit_hi = limit_hi
        self._pos0 = pos
        self._block_dir = 0.0
        if block_rad is not None:
            self._block_dir = 1.0 if block_rad > pos else -1.0
        # 编码器置零（CMD 0xFE）后的偏置：上报值 = 物理 pos − 偏置。
        # 0xFE 把当下读数记成 0，所以置零时设成当时的 pos。
        self.zero_offset = 0.0

    @property
    def pos(self) -> float:
        """物理位置（真值，不是上报值）。"""
        return self._pos

    @pos.setter
    def pos(self, value: float) -> None:
        self._pos = value
        self._hist.append(value)
        while len(self._hist) > self.report_lag_ticks + 1:
            self._hist.pop(0)

    def set_zero(self) -> None:
        """把当前读数记成 0（模拟 0xFE）—— 不改物理位置，只改上报偏置。"""
        self.zero_offset = self.pos

    def reported_pos(self) -> float:
        """上报位置（可量化、可滞后 —— 模拟真机的读数路径）。

        ``sticky_rad`` 模拟闭合侧约 0.0103 rad 的粘滑死区；``report_lag_ticks``
        模拟缓存读数落后若干帧。两者都为 0 时就是当下的物理位置，逐位不变。
        """
        p = self._hist[0] - self.zero_offset
        if self.sticky_rad > 0:
            return round(p / self.sticky_rad) * self.sticky_rad
        return p

    def set_block(self, block_rad: Optional[float]) -> None:
        """在当前位置的前方装/拆机械挡块。"""
        self.block_rad = block_rad
        self._block_dir = (0.0 if block_rad is None
                           else (1.0 if block_rad > self.pos else -1.0))

    def step(self, q: float, kp: float, dq: float, tau_ff: float, dt: float) -> None:
        # 位置项按 kp 缩放：kp=0（失力 / stop）时不产生任何跟随速度 —— 真电机
        # 零刚度就是这样，可被外力推动。kp=KP_NOMINAL 时与原来完全一致。
        v = dq + self.GAIN * (kp / self.KP_NOMINAL) * (q - self.pos)
        # 纯前馈力矩下的蠕变（见类上方的说明）。默认 0 时不参与运算。
        v += self.creep_rad_s_per_nm * tau_ff
        v = max(-self.VMAX, min(self.VMAX, v))
        new = self.pos + v * dt
        if self.block_rad is not None:
            if self._block_dir > 0:
                new = min(new, self.block_rad)
            else:
                new = max(new, self.block_rad)
            # 接触是保持的：只要本帧的净力矩还在朝工件压，已经顶在工件上的
            # 夹爪就退不出来（真机上是工件把它顶住）。净力矩过了门槛时工件
            # 还会朝受力方向持续让位，夹爪跟着走 —— 这就是「设定力下工件屈服」
            # 那个工况。让位速度是给定的，不建动力学；载荷一掉回门槛以下就让
            # 位自己停下来，所以它不会跑飞。
            net = kp * (q - self.pos) + tau_ff
            if self.pos == self.block_rad and self._block_dir * net > 0.0:
                new = self.block_rad
                if (self.yield_rad_s > 0.0
                        and abs(net) >= self.yield_tau_nm):
                    self.block_rad += self._block_dir * self.yield_rad_s * dt
                    new = self.block_rad
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
                 limit_hi: Optional[float] = None,
                 yield_rad_s: float = 0.0, yield_tau_nm: float = 0.0,
                 creep_rad_s_per_nm: float = 0.0,
                 report_lag_ticks: int = 0):
        self.motor = FakeMotor(pos=pos, block_rad=block_rad,
                               sticky_rad=sticky_rad, err=err,
                               limit_lo=limit_lo, limit_hi=limit_hi,
                               yield_rad_s=yield_rad_s,
                               yield_tau_nm=yield_tau_nm,
                               creep_rad_s_per_nm=creep_rad_s_per_nm,
                               report_lag_ticks=report_lag_ticks)
        self.frames: List[Frame] = []
        self.initialize_results = list(initialize_results or [])
        self.initialize_calls = 0
        self.clear_fault_calls = 0
        self.disconnected = False
        self.error_when_holding: Optional[int] = None
        self.holding = False
        #: 命令序列 —— 按发生顺序记下 disable / set_zero / initialize，
        #: 用来守「先失能、再发 0xFE、后重使能」这一步序。
        self.calls: List[str] = []
        #: 0xFE 是否被接受（False = 帧没发出/没注册，set_zero() 返回 False）。
        self.set_zero_ok = True
        #: 0xFE 是否真的改写偏置（False = 电机忽略了它，比如仍在使能态：
        #: set_zero() 照报 True，但读数不变）。
        self.set_zero_applies = True
        #: 下一次下发要报的失败原因 —— 非 None 时 ``control_mit`` 一帧都不发，
        #: 直接回 False，并把它记进 ``last_send_error``。真 CAN 层就是这样把
        #: 「没连上」和「总线写失败」分开报的。
        self.send_error: Optional[Exception] = None
        #: ``LiteGripCAN.last_send_error`` 的对应物：上一次下发失败的原因。
        self.last_send_error: Optional[Exception] = None
        #: 总线上还有没有状态帧 —— False = 空闲：没被驱动，电机不主动开口，
        #: poll / update_state 都回 False，读数就是上一次缓存的那份。
        self.delivers = True
        #: ``refresh_status()`` 的 0xCC 发出去了没有（False = 写失败）。
        self.refresh_ok = True
        #: ``refresh_status()`` 被调了几次。
        self.refresh_calls = 0
        #: 要到的那一帧还没被读走 —— 下一次 poll / update_state 会读到它。
        self.refresh_pending = False
        #: 最近一帧的两个时间戳。真层也是两条钟：``last_update`` 是 monotonic
        #: （量 age 用），``last_frame_time`` 是 wall clock（人读的 timestamp
        #: 用）。None = 还没有过帧，和真层 ``MotorState`` 的 0.0 一个意思。
        self.last_update: Optional[float] = None
        self.last_frame_time: Optional[float] = None

    # ── 运动 ───────────────────────────────────────────────────────────
    def control_mit(self, q_target, kp, kd, dq_target=0.0,
                    tau_feedforward=0.0) -> bool:
        if self.send_error is not None:
            self.last_send_error = self.send_error
            return False
        self.motor.step(q_target, kp, dq_target, tau_feedforward, DT)
        self.frames.append(Frame(
            q=q_target, kp=kp, kd=kd, dq=dq_target, tau_ff=tau_feedforward,
            pos_after=self.motor.pos, tau_nm=self.motor.tau))
        self.holding = tau_feedforward != 0.0
        self.last_send_error = None
        return True

    def control_mit_stream(self, q_target, kp, kd, duration_s=0.5,
                           interval_s=0.005, dq_target=0.0,
                           tau_feedforward=0.0) -> bool:
        """把 ``control_mit`` 连发一“段平台”，和真 CAN 层同名同形。

        ``send_error`` 置上时逐帧失败，于是和真层一样回 False —— 真层在整段
        一帧都没发出去时抛 ``CommError``，这里只回 False：替身不模拟那条
        「一帧都没出去」的阈值。
        """
        if interval_s <= 0:
            return True
        ok = True
        for _ in range(max(1, int(duration_s / interval_s))):
            ok = self.control_mit(
                q_target, kp, kd, dq_target, tau_feedforward) and ok
        return ok

    # ── 状态 ───────────────────────────────────────────────────────────
    def poll(self, timeout_s: float = 0.0) -> bool:
        return self._frame_available()

    def update_state(self, timeout_s: float = 0.05) -> bool:
        return self._frame_available()

    def _frame_available(self) -> bool:
        """读到一帧了吗 —— 被驱动时总有，空闲时只有 ``refresh_status()`` 要到过。

        读到就把那对时间戳更新成「刚刚」；没读到就原样留着，于是读数仍然是
        上一帧的，age 会如实涨上去。
        """
        if self.delivers or self.refresh_pending:
            self.refresh_pending = False
            self.last_update = time.monotonic()
            self.last_frame_time = time.time()
            return True
        return False

    def refresh_status(self) -> bool:
        """0xCC：电机收到就回一帧。空闲总线上这是唯一让它开口的办法。"""
        self.refresh_calls += 1
        if self.refresh_ok:
            self.refresh_pending = True
        return self.refresh_ok

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
        self.calls.append("disable")
        self.motor.err = 0
        return True

    def set_zero(self) -> bool:
        """0xFE：把当前读数记成 0（只改偏置，不改物理位置）。

        ``set_zero_ok=False`` 模拟帧没发出去（返回 False）；
        ``set_zero_applies=False`` 模拟电机忽略了 0xFE（返回 True 但偏置不动）。
        """
        self.calls.append("set_zero")
        if not self.set_zero_ok:
            return False
        if self.set_zero_applies:
            self.motor.set_zero()
        return True

    def clear_fault(self) -> bool:
        self.clear_fault_calls += 1
        self.motor.err = 0
        return True

    def initialize(self) -> bool:
        """按 ``initialize_results`` 脚本回答；成功时把 err 置 1。

        没给脚本就当成一台好电机（返回 True）。
        """
        self.calls.append("initialize")
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
    start_rad: Optional[float] = None,
    block_rad: Optional[float] = None,
    sticky_rad: float = 0.0,
    err: int = 1,
    motion: Optional[MotionConfig] = None,
    initialize_results: Optional[List[bool]] = None,
    stops: bool = False,
    reverse: bool = False,
    yield_rad_s: float = 0.0,
    yield_tau_nm: float = 0.0,
    creep_rad_s_per_nm: float = 0.0,
    report_lag_ticks: int = 0,
):
    """造一个「已连接、已使能、已标定」的 LiteGrip，底层换成 :class:`FakeLiteGripCAN`。

    默认用用户真机标定值（``POS_CLOSED_RAD`` / ``POS_OPEN_RAD`` / ``RAD_TO_MM``），
    ``sleep_fn`` 置空 —— 整条斜坡瞬间跑完。

    ``reverse=True`` 把两个限位数值对调（反装）。物理限位是同一对，换的只是
    「哪个数值叫闭合」。

    ``stops=True`` 在两端装机械限位 —— 真夹爪本来就有，而 ``open``/``close``
    现在靠撞它来结束运动，所以这类用例必须开。

    ``yield_rad_s`` / ``yield_tau_nm`` 把 ``block_rad`` 处的工件改成**会屈服**
    的：被顶住且载荷过门槛就按 ``yield_rad_s`` 退让，用来复现「夹到工件后工件
    缓慢让位」的保力场景。

    ``creep_rad_s_per_nm`` 让夹爪在**纯前馈力矩**下被推着走 —— 保力帧（``kp=0``）
    在默认模型里一点不动，而真机上闭合侧没有限位时恒定力矩会把夹爪一路推走。
    配合 ``stops=False`` 复现那条失控路径，用来验证力控段的位置护栏。

    ``report_lag_ticks`` 让上报位置落后若干帧 —— 真机上 ``get_state(wait=False)``
    拿的是缓存读数。力控接近段的指令是 ``min(排程, 读数 + 领先上限)``，领先只有
    零点几毫米时延迟就把它吃光了，夹爪因此推不动排程。默认 0，不影响别的用例。

    ``start_rad`` 默认停在张开侧（两种装法各自的那一端）。

    Returns:
        ``(gripper, fake_can)``
    """
    g = LiteGrip("vcan0", can_id=0x08)
    cfg = g.config
    if reverse:
        cfg.pos_closed_rad, cfg.pos_open_rad = POS_OPEN_RAD, POS_CLOSED_RAD
    else:
        cfg.pos_closed_rad, cfg.pos_open_rad = POS_CLOSED_RAD, POS_OPEN_RAD
    cfg.rad_to_mm = RAD_TO_MM
    cfg.calibrated = True

    if start_rad is None:
        start_rad = cfg.pos_open_rad        # 张开侧

    fake = FakeLiteGripCAN(pos=start_rad, block_rad=block_rad,
                           sticky_rad=sticky_rad, err=err,
                           initialize_results=initialize_results,
                           limit_lo=LIMIT_LO_RAD if stops else None,
                           limit_hi=LIMIT_HI_RAD if stops else None,
                           yield_rad_s=yield_rad_s,
                           yield_tau_nm=yield_tau_nm,
                           creep_rad_s_per_nm=creep_rad_s_per_nm,
                           report_lag_ticks=report_lag_ticks)
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
