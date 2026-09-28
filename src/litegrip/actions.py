"""LiteGrip 动作层 —— open / close / grasp / zero / enable / disable。

高层 ``LiteGrip`` 的 ``open()`` / ``close()`` / ``grasp()`` 等方法直接转发到
这里的 :class:`GripperActions`。单独成模块是为了让「怎么动」这套逻辑只写一遍：
ROS 2 桥接、RPC 服务、产品代码都能直接调 :attr:`LiteGrip.actions`，而不用各自
重写一遍斜坡和堵转判据。

为什么不用 ``goto_rad()`` / ``control_mit_stream()``
---------------------------------------------------
``control_mit_stream`` 在整段时长里反复下发**同一个** ``q_target``：伺服几十
毫秒就贴上去，剩下时间空转 —— 慢速时表现为一顿一停。这里的做法和 SDK 自己的
``move_at_speed`` 一样，按固定帧间隔推进一条线性斜坡，并给 ``dq_target`` 速度
前馈，所以是匀速连续运动。

指令领先实测位置的部分由 :attr:`MotionConfig.max_lead_mm` 封顶。不封顶的话，
被挡住时误差会一直累积、力矩顶到危险值；封顶后静摩擦靠满额力矩破，力矩却始
终有界（约 ``kp × lead_cap_rad``）。也不能改成「相对实测加一块」——那样一旦夹
住，指令跟着实测冻结，误差永远涨不上去，会误判堵转。

堵转判据是软件侧的位置增量判据（电机本身没有堵转保护）：每
:attr:`MotionConfig.sample_interval` 采一次位置，连续
:attr:`MotionConfig.stall_cycles` 次采样的**窗口净位移**小于阈值即判堵转。
阈值 = ``max(stall_delta, stall_ratio × 窗口内本该走的距离)``。不看单点，因为
闭合侧有约 0.010 rad 的机械死区，慢速粘滑时单点忽大忽小。斜坡走完的保压段
不判（那时夹爪本来就该不动）。

本模块**不 print**：进度通过 ``progress`` 回调交给调用方（CLI 打印、ROS 节点
记日志、RPC 服务转发都行）。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Optional, Tuple

from .constants import UnitConversion
from .exceptions import CommandError, LiteGripError
from .models import CalibrationData, GripperConfig, GripperState

if TYPE_CHECKING:                                    # 避免运行时循环 import
    from .gripper import LiteGrip

log = logging.getLogger("litegrip")


# ═══════════════════════════════════════════════════════════════════════════
# 可调量
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class MotionConfig:
    """open / close / grasp / zero / enable 的全部可调量。

    默认值即真机上调好的那组（50 mm/s、5 ms 帧、4 mm 领先上限等），
    不改就能用。速度单位一律 mm/s（开口量），内部按
    ``GripperConfig.rad_to_mm`` 换算成 rad。

    ``sleep_fn`` / ``monotonic_fn`` 是给测试和仿真留的缝：引擎内部一律走
    这两个，不直接调 ``time.*``。测试里传 ``sleep_fn=lambda _: None`` 就能让
    整条斜坡瞬间跑完，不必 monkeypatch ``time.sleep``。
    """

    # ── 运动 ───────────────────────────────────────────────────────────
    speed_mm_s: float = 50.0            # open/close 速度
    grasp_speed_mm_s: float = 50.0      # grasp 闭合段速度
    margin: float = 0.05                # 距标定限位留下的行程余量比例
    frame_interval: float = 0.005       # 200 Hz 斜坡帧间隔 s
    sample_interval: float = 0.05       # 20 Hz 堵转采样间隔 s
    settle_s: float = 0.3               # 斜坡后原地保目标时长 s（不判堵转）
    reach_tol: float = 0.02             # 到位容差 rad（闭合侧死区 0.0103）

    # ── 堵转判据 ───────────────────────────────────────────────────────
    stall_cycles: int = 5               # 窗口采样点数
    stall_ratio: float = 0.2            # 窗口净位移 / 本该走的距离
    stall_delta: float = 0.0015         # 阈值下限 rad

    # ── 力矩 / 指令上限 ────────────────────────────────────────────────
    max_lead_mm: float = 4.0            # 指令领先实测的上限 mm（≈ kp × 上限）

    # ── 保力 ───────────────────────────────────────────────────────────
    force_n: float = 20.0               # 默认夹持力 N（= 2.0 Nm）
    hold_interval: float = 0.2          # 保力分片时长 s
    hold_kp: float = 150.0              # 保力刚度（对齐 set_force）
    hold_kd: float = 2.0                # 保力阻尼

    # ── 使能 ───────────────────────────────────────────────────────────
    enable_retries: int = 3             # 使能重试次数
    enable_retry_interval: float = 0.2  # 使能重试间隔 s

    # ── zero() 标定探测 ────────────────────────────────────────────────
    calib_kp: float = 60.0              # 低刚度更温和
    calib_kd: float = 2.0
    calib_step_rad: float = 0.1         # 每步步进 rad
    calib_stall_delta: float = 0.0015   # 标定堵转判据 rad
    calib_stall_cycles: int = 5         # 标定连续堵转次数
    calib_max_iter: int = 80            # 单向步数上限

    # ── 测试/仿真缝 ────────────────────────────────────────────────────
    sleep_fn: Callable[[float], None] = field(
        default=time.sleep, repr=False, compare=False)
    monotonic_fn: Callable[[], float] = field(
        default=time.monotonic, repr=False, compare=False)


# ═══════════════════════════════════════════════════════════════════════════
# 结果类型
# ═══════════════════════════════════════════════════════════════════════════
#
# 都实现 __bool__，所以 ``if gripper.open():`` 这种老写法继续可用。

@dataclass
class MoveProgress:
    """一次进展快照，交给 ``progress`` 回调。"""

    phase: str                          # "move" 或 "hold"
    i: int                              # 当前帧号 / 保力片号
    total_steps: int                    # 总帧数（保力段为 0）
    cmd_rad: float                      # 本帧下发的指令位置
    pos_rad: float                      # 本帧实测位置
    delta_rad: float                    # 距上一次采样的位置增量
    win_delta_rad: Optional[float]      # 窗口净位移（采样点数不够时为 None）
    torque_nm: float                    # 实测力矩
    temperature_coil: int = 0           # 线圈温度 °C


@dataclass
class MoveResult:
    """open / close 的结果。"""

    reached: bool                       # 末端是否在 reach_tol 内到目标
    stalled: bool                       # 是否判到堵转
    state: GripperState                 # 末端状态
    target_rad: float                   # 目标位置
    final_cmd_rad: float                # 最后一帧下发的指令位置
    steps: int                          # 实际走了多少帧

    def __bool__(self) -> bool:
        return self.reached and not self.stalled


@dataclass
class GraspResult:
    """grasp 的结果。"""

    ok: bool                            # 保力是否正常结束（没被故障/中止打断）
    reached: bool                       # 闭合段是否到空载目标位置
    stalled: bool                       # 闭合段是否判到堵转（= 夹到工件）
    state: GripperState                 # 末端状态
    target_rad: float                   # 闭合段的空载目标位置
    force_n: float                      # 实际用的夹持力 N
    cycles: int                         # 保力片数

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class EnableResult:
    """enable 的结果。"""

    ok: bool                            # 状态帧是否回读到 err == 1
    state: Optional[GripperState]       # 最后一次状态
    tries: int                          # 实际尝试次数

    def __bool__(self) -> bool:
        return self.ok


# ═══════════════════════════════════════════════════════════════════════════
# 目标位置
# ═══════════════════════════════════════════════════════════════════════════

def limit_target(
    config: GripperConfig,
    toward: str,
    margin: float,
) -> Tuple[float, float, float, float]:
    """算出一端的目标位置：标定限位往行程内侧退 ``margin`` 比例的余量。

    不让夹爪真顶到机械限位（那里 kp=100 会压出 ~4 Nm），而是停在限位内侧。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` / ``pos_open_rad``）。
        toward: ``"close"`` 或 ``"open"``。
        margin: 行程余量比例，0.05 = 两端各留 5%。

    Returns:
        ``(目标位置, 标定限位, 余量 rad, 行程 rad)``

    Raises:
        CommandError: 位置约定不成立（``pos_open_rad >= pos_closed_rad``）。
            这多半是没加载标定，用了 :class:`GripperConfig` 的默认值。
    """
    if config.pos_open_rad >= config.pos_closed_rad:
        raise CommandError(
            f"位置约定不成立：pos_open_rad={config.pos_open_rad} 应小于 "
            f"pos_closed_rad={config.pos_closed_rad}（全开在数值更小的一侧）。"
            f"多为没加载标定 —— 先 load_calibration() 或跑一次 zero()。")

    travel = abs(config.pos_closed_rad - config.pos_open_rad)
    margin_rad = margin * travel
    if toward == "close":
        limit = config.pos_closed_rad
        return limit - margin_rad, limit, margin_rad, travel
    limit = config.pos_open_rad
    return limit + margin_rad, limit, margin_rad, travel


# ═══════════════════════════════════════════════════════════════════════════
# 动作层
# ═══════════════════════════════════════════════════════════════════════════

class GripperActions:
    """夹爪的六个动作：``open`` / ``close`` / ``grasp`` / ``zero`` /
    ``enable`` / ``disable``。

    一般不直接构造，用 :attr:`LiteGrip.actions`::

        with LiteGrip("can0") as g:
            g.load_calibration()
            g.actions.enable()
            g.actions.open()
            g.actions.grasp(force_n=20.0, hold_s=3.0)

    进度回调 ``progress`` 收到 :class:`MoveProgress`，本模块自己不打印任何东西。
    """

    def __init__(self, gripper: "LiteGrip", config: Optional[MotionConfig] = None):
        self._g = gripper
        self.config = config if config is not None else MotionConfig()

    # ═══════════════════════════════════════════════════════════════════
    # 六个接口
    # ═══════════════════════════════════════════════════════════════════

    def open(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """全开：走到「张开侧限位内侧留 margin」的位置。"""
        speed = self.config.speed_mm_s if speed_mm_s is None else speed_mm_s
        return self._move_to_limit("open", speed, progress=progress)

    def close(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """全合：走到「闭合侧限位内侧留 margin」的位置。"""
        speed = self.config.speed_mm_s if speed_mm_s is None else speed_mm_s
        return self._move_to_limit("close", speed, progress=progress)

    def grasp(
        self,
        force_n: Optional[float] = None,
        hold_s: float = 0.0,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> GraspResult:
        """夹取：先闭合到堵转（= 夹住工件），再持续输出 ``force_n`` 大小的力。

        Args:
            force_n: 夹持力 N，``None`` = 用 :attr:`MotionConfig.force_n`。
                按 SDK 近似换算 1 N = 0.1 Nm。
            hold_s: 保力时长 s。``0`` = 一直保到出错或 Ctrl+C。
            progress: 进度回调。

        Returns:
            :class:`GraspResult`。夹住工件时 ``stalled=True`` 且
            ``reached=False``（压不到空载目标位置是正常的）。
        """
        cfg = self.config
        force = cfg.force_n if force_n is None else force_n

        move = self._move_to_limit(
            "close", cfg.grasp_speed_mm_s, progress=progress)
        ok, cycles, st = self._hold_force(force, hold_s, progress=progress)
        return GraspResult(
            ok=ok,
            reached=move.reached,
            stalled=move.stalled,
            state=st if st is not None else move.state,
            target_rad=move.target_rad,
            force_n=force,
            cycles=cycles,
        )

    def zero(self) -> CalibrationData:
        """完整标定：探闭合 + 张开两个限位，算出行程与换算系数，并存盘。

        过程中夹爪会主动顶住两端机械限位（低刚度探测）。确保行程内无物。

        Returns:
            :class:`CalibrationData`。
        """
        cfg = self.config
        data = self._g.calibrate(
            kp=cfg.calib_kp,
            kd=cfg.calib_kd,
            step_rad=cfg.calib_step_rad,
            stall_delta=cfg.calib_stall_delta,
            stall_cycles=cfg.calib_stall_cycles,
            max_iter=cfg.calib_max_iter,
        )
        self._g.save_calibration()
        return data

    def enable(self, retries: Optional[int] = None) -> EnableResult:
        """反复 enable 直到状态帧回读到 ``err == 1``（真使能）。

        为什么需要重试：``enable`` 是单向命令、无确认，CAN 上丢一帧就白发了。
        这里把「发 enable → 回读状态帧 → 不是 1 就重发」做成显式重试。
        ``err`` 属于真实故障（非 0/1）时先 ``clear_fault()`` 再重试。

        Args:
            retries: 重试次数，``None`` = 用 :attr:`MotionConfig.enable_retries`。

        Returns:
            :class:`EnableResult`（``ok`` / ``state`` / ``tries``）。
        """
        cfg = self.config
        tries_max = cfg.enable_retries if retries is None else retries
        st: Optional[GripperState] = None

        for i in range(1, tries_max + 1):
            try:
                self._g._enable_once()
            except LiteGripError as e:
                log.warning("enable() 第 %d/%d 次抛错：%s: %s",
                            i, tries_max, type(e).__name__, e)

            st = self._g.get_state()
            if st.error_code == 1:
                return EnableResult(ok=True, state=st, tries=i)

            if st.error_code not in (0, 1):
                # 真实故障（欠压/过流/过温等）：先清故障再重试
                try:
                    self._g.clear_fault()
                except LiteGripError as e:
                    log.warning("clear_fault() 失败：%s", e)

            if i < tries_max:
                log.warning("第 %d/%d 次未使能（状态帧 err=%d，0=未使能）"
                            "—— %.2fs 后重发 enable",
                            i, tries_max, st.error_code,
                            cfg.enable_retry_interval)
                cfg.sleep_fn(cfg.enable_retry_interval)

        return EnableResult(ok=False, state=st, tries=tries_max)

    def disable(self) -> bool:
        """失能电机。"""
        return self._g._disable_once()

    # ═══════════════════════════════════════════════════════════════════
    # 引擎
    # ═══════════════════════════════════════════════════════════════════

    def _emit(self, q: float, dq: float = 0.0, tau: float = 0.0,
              kp: Optional[float] = None, kd: Optional[float] = None) -> None:
        """下发一帧 MIT 并等待一帧的时间。"""
        cfg = self.config
        g = self._g
        sent = g.send_mit_frame(
            q,
            g.config.kp if kp is None else kp,
            g.config.kd if kd is None else kd,
            dq=dq,
            tau=tau,
        )
        if not sent:
            raise CommandError("MIT 帧下发失败（未连接或未使能）")
        cfg.sleep_fn(cfg.frame_interval)

    def _move_to_limit(
        self,
        toward: str,
        speed_mm_s: float,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """走到「限位内侧留余量」的位置：整段是一条 frame_interval 一格的
        连续斜坡（MIT 帧 + 速度前馈），边走边按位置判堵转。

        返回 :class:`MoveResult`。
        """
        cfg = self.config
        g = self._g
        g._check_enabled()
        gcfg = g.config

        target, _limit, _margin_rad, _travel = limit_target(
            gcfg, toward, cfg.margin)

        before = g.get_state()
        dist_rad = target - before.position_rad
        dist_mm = abs(dist_rad) * gcfg.rad_to_mm
        sign = 1.0 if dist_rad >= 0 else -1.0
        speed_rad_s = speed_mm_s / gcfg.rad_to_mm

        interval = cfg.frame_interval
        ramp_s = dist_mm / speed_mm_s if speed_mm_s > 0 else 0.0
        ramp_steps = max(1, int(round(ramp_s / interval)))
        settle_steps = max(1, int(round(cfg.settle_s / interval)))
        total_steps = ramp_steps + settle_steps

        sample_every = max(1, int(round(cfg.sample_interval / interval)))
        win = max(cfg.stall_cycles, 1)
        win_s = win * sample_every * interval
        win_expect_rad = speed_rad_s * win_s
        win_thresh_rad = max(cfg.stall_delta, cfg.stall_ratio * win_expect_rad)
        lead_cap_rad = max(cfg.max_lead_mm / gcfg.rad_to_mm,
                           speed_rad_s * interval)

        log.info("%s %.4f → %.4f rad（%.1f mm/s，%.1f mm，%d+%d 帧，"
                 "堵转阈值 %.5f rad，领先上限 %.5f rad）",
                 "闭合" if toward == "close" else "张开",
                 before.position_rad, target, speed_mm_s, dist_mm,
                 ramp_steps, settle_steps, win_thresh_rad, lead_cap_rad)

        hist = [before.position_rad]
        stalled = False
        st = before
        last_cmd = before.position_rad
        last_i = 0

        for i in range(1, total_steps + 1):
            st = g.get_state(wait=False)
            pos = st.position_rad
            if i <= ramp_steps:
                q_sched = before.position_rad + dist_rad * (i / ramp_steps)
                dq = sign * speed_rad_s
            else:
                q_sched = target                   # 保压：原地顶住目标
                dq = 0.0
            lead = (q_sched - pos) * sign
            cmd = pos + sign * lead_cap_rad if lead > lead_cap_rad else q_sched
            last_cmd = cmd
            self._emit(cmd, dq, 0.0)
            last_i = i

            if i % sample_every and i != total_steps:
                continue
            hist.append(pos)
            win_delta = (abs(hist[-1] - hist[-1 - win])
                         if len(hist) > win else None)
            if progress is not None:
                progress(MoveProgress(
                    phase="move",
                    i=i,
                    total_steps=total_steps,
                    cmd_rad=cmd,
                    pos_rad=pos,
                    delta_rad=hist[-1] - hist[-2],
                    win_delta_rad=win_delta,
                    torque_nm=st.torque_nm,
                    temperature_coil=st.temperature_coil,
                ))
            # 保压段 (i > ramp_steps) 本来就该不动，不判堵转
            if (win_delta is not None and i <= ramp_steps
                    and win_delta < win_thresh_rad):
                stalled = True
                log.info("堵转：最近 %d 次采样(%.2f s)净位移仅 %.5f rad "
                         "(< %.5f)，停在 %+.5f rad",
                         win, win_s, win_delta, win_thresh_rad, pos)
                break

        st = g.get_state()                         # 阻塞等一帧新状态再判到位
        reached = abs(st.position_rad - target) < cfg.reach_tol
        return MoveResult(
            reached=reached,
            stalled=stalled,
            state=st,
            target_rad=target,
            final_cmd_rad=last_cmd,
            steps=last_i,
        )

    def _hold_force(
        self,
        force_n: float,
        hold_s: float,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> Tuple[bool, int, Optional[GripperState]]:
        """持续输出夹持力：每 :attr:`MotionConfig.hold_interval` 读一次位置，
        然后用 ``hold_kp`` / ``hold_kd`` + 前馈力矩把当前位置顶住。

        ``hold_s <= 0`` 表示不限时长（直到出错或 Ctrl+C）。

        Returns:
            ``(是否正常结束, 保力片数, 最后一次状态)``。
        """
        cfg = self.config
        g = self._g
        tau_nm = force_n * UnitConversion.N_TO_NM

        deadline = None if hold_s <= 0 else cfg.monotonic_fn() + hold_s
        frames_per_slice = max(1, int(round(cfg.hold_interval
                                            / cfg.frame_interval)))
        cycles = 0
        st = g.get_state()
        pos = st.position_rad

        while deadline is None or cfg.monotonic_fn() < deadline:
            # 用上一次读到的位置顶住一片时长，再回读状态查故障
            for _ in range(frames_per_slice):
                self._emit(pos, 0.0, tau_nm,
                           kp=cfg.hold_kp, kd=cfg.hold_kd)
            cycles += 1

            st = g.get_state()
            if st.error_code != 1:
                log.warning("保力中断：状态帧 err=%d"
                            "（1=使能中；0=被禁用；其他=故障）", st.error_code)
                return False, cycles, st
            pos = st.position_rad

            if progress is not None:
                progress(MoveProgress(
                    phase="hold",
                    i=cycles,
                    total_steps=0,
                    cmd_rad=pos,
                    pos_rad=pos,
                    delta_rad=0.0,
                    win_delta_rad=None,
                    torque_nm=st.torque_nm,
                    temperature_coil=st.temperature_coil,
                ))

        return True, cycles, st
