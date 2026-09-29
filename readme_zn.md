# litegrip-python

[English](README.md) | [简体中文](readme_zn.md)

**LiteGrip 轻量机械爪系列**的 Python SDK（主要公开接口）。通过经典 CAN 上用 MIT 控制
协议驱动夹爪的达妙 DM4310 电机。

## 适用范围

| | |
| --- | --- |
| 产品 | LiteGrip 轻量机械爪系列 |
| 本仓角色 | Python SDK（主要公开接口） |
| 状态 | 活跃 —— 源码、打包与测试均已就位 |
| 电机 | 达妙 DM4310，默认 CAN ID `0x08`，MIT 模式，经典 CAN 1 Mbit/s |
| 平台 | 仅 Linux（SocketCAN） |
| Python | 3.8 及以上 |
| 运行时依赖 | 无，只用标准库 |

## 安装

尚未发布到 PyPI。从代码仓安装：

```bash
git clone https://github.com/nexform-tech/litegrip-python.git
cd litegrip-python
python3 -m pip install .
```

也可以完全不安装，直接把 `PYTHONPATH` 指向源码树：

```bash
PYTHONPATH=/path/to/litegrip-python/src python3 your_script.py
```

运行前先把 CAN 接口拉起来：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 快速上手

```python
from litegrip import LiteGrip

with LiteGrip(channel="can0", can_id=0x08) as gripper:
    gripper.load_calibration()   # 本通道自己的标定文件，没有就回退出厂标定
    gripper.enable()             # 反复重试，直到状态帧回读到 err == 1

    gripper.open()                                     # 50 mm/s 走到张开侧
    gripper.close()                                    # 50 mm/s 走到闭合侧
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # 闭合到夹住，再保 20 N 力
    print(result.reached, result.stalled, result.cycles)
```

`with` 退出时会调 `disconnect()`，默认顺带失能电机。如果想让夹爪在离开代码块后仍保持
使能，传 `LiteGrip(..., disable_on_disconnect=False)`，或设
`gripper.disable_on_disconnect = False`。

## 反装，以及一台电脑带两台

夹爪闭合时电机往哪个方向数，不是固定的：电机反着装的夹爪就是**反装**，对它来说闭合是 rad
减小。SDK 两种都不预设 —— 方向由两个标定限位的**数值顺序**推出来（见
`GripperConfig.close_sign`），所以两种装法都能用。SDK 不肯做的是**猜**：没加载标定之前，
运动接口一律抛 `CommandError`。

方向靠**名字**声明。`list_templates()` 按顺序给出可选名字，给 UI 做下拉框用；选中后
用 `mount=` 加载：

```python
from litegrip import LiteGrip, list_templates

list_templates()                       # ["normal", "reverse"] —— 给 UI 用

with LiteGrip("can1", mount="reverse") as gripper:
    print(gripper.mount)               # "reverse" —— 从限位数值读回
    gripper.enable()
    gripper.zero()                     # 可选：实测真实行程并存盘
```

同一个选择有四个入口：`LiteGrip(..., mount="reverse")`、`gripper.load_template("reverse")`、
`gripper.load_calibration(template="reverse")`，以及已有路径时的
`load_calibration(CALIB_TEMPLATES["reverse"])`；四者加载的是同一个文件。两份模板都只是
**标称**值：用处是声明方向、给一个像样的行程，随后 `zero()` 会把行程换成实测值。名字不在
`list_templates()` 里会抛 `CommandError` 并列出合法名字；模板读不出来时**直接抛**、不回退 ——
出厂文件是正装，拿它回答「反装」的请求，正是这个按名字选的入口要防的失败。

判断是哪一种只需要看一眼：让夹爪走一小段，看指爪往哪边动。选错装法不是无声的 —— 加载时会
把推导出的方向打进日志，第一次 `close()` 也会朝错的方向走。随时可以用 `gripper.mount` 读回来
（`"normal"` / `"reverse"`，未标定时是 `None`）。

所有 LiteGrip 共用同一个 CAN ID（`0x08`），所以一台电脑上放两台时，**通道是唯一的身份键**。
标定**按通道各存各的**：`~/.litegrip/<channel>_calibration.json`，两台互不覆盖。无参
`load_calibration()` 先读本通道这一份，再读旧的单文件位置，最后才读出厂标定；自动链里凡是
声明了**别的**通道的候选文件一律跳过 —— 所以 `can1` 自己没有标定时会**大声失败**（返回
`False`，随后运动接口抛 `CommandError`），而不是悄悄采纳 `can0` 的方向。想让所有通道共用一个
显式路径，设 `LITEGRIP_CALIB`。

## 主从遥操

两台夹爪可以联动，一台跟着另一台动。**主夹爪**（leader）的电机卸力 —— 你用手掰它的爪子，
它按循环频率把「张开程度」发出去；**从夹爪**（follower）收到后驱动自己的爪子跟到位。线上传的
是归一化到 `[0, 1]` 的张开度，不是角度，所以两端不需要相同的标定、装法或零点。

```python
from litegrip import LiteGrip

# 主端：把本夹爪的张开度发到 192.168.1.20 的从端。
with LiteGrip("can0") as master:
    master.load_calibration()
    master.enable()
    master.teleop_start("master", host="192.168.1.20")

# 从端：绑定端口，先对齐首帧，然后跟随。
with LiteGrip("can0") as slave:
    slave.load_calibration()
    slave.enable()
    slave.teleop_start("slave", host="0.0.0.0")
    while True:
        print(slave.teleop_status())   # frames, openness, loop_hz, stale, ...
```

`examples/teleop.py` 可以在命令行跑其中一端：

```bash
# A 机 —— 你用手掰的主夹爪：
python3 examples/teleop.py --mode master --channel can0 --host 192.168.1.20
# B 机 —— 从夹爪：
python3 examples/teleop.py --mode slave  --channel can0 --host 0.0.0.0
```

两端必须共用 `master_id`（默认 `master`），且都已连接、已使能。遥操是互斥的：后台循环独占 CAN
读写，在 `teleop_stop()` 之前不要再从调用方驱动夹爪。`teleop_start` 返回初始的
`teleop_status()`；`teleop_status()` 报告 `active`、`mode`、`topic`、`frames`、
`last_frame_age_ms`、`stale`、`openness`、`loop_hz`。

- **传输是明文 UDP**，无鉴权、无加密，只用在可信网络里。要给自定义传输，传 `transport=` 一个
  `TeleopTransport`；注入的传输不会被 SDK 关闭。
- **从端与主端失联时是「持位」，不是「卸力」。** 超过 `watchdog_s`（默认 `0.2`）没有新帧后，
  它仍按跟随增益顶着上一个目标继续发帧 —— 于是 `stale` 变真，但爪子停在原地，可能夹住中间的
  东西。
- **从端会把收到的张开度夹到 `[0, 1]`**，也就是夹在自己的标定行程内，坏帧无法把它指到限位之外。
- **停止后是持位**，不是卸力：主端在 `teleop_stop()` 时退出零重力模式，爪子按配置增益持位。
- 跟随增益默认 `kp=100.0`、`kd=2.0`，用 `kp=` / `kd=` 覆盖。

## 六个动作接口

要让夹爪动起来就用这六个。每一个都会自己校验结果再报成功，所以调用方不必再重写斜坡和
堵转判据。

| 方法 | 行为 | 返回 |
| --- | --- | --- |
| `open(speed_mm_s=None)` | 按斜坡**越过**标定的张开侧限位，由机械限位结束这趟运动。 | `MoveResult` |
| `close(speed_mm_s=None)` | 同上，朝闭合侧。 | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | 闭合到堵转（即夹住），然后持续输出 `force_n`。`hold_s=0` 表示不限时长。 | `GraspResult` |
| `zero()` | 完整标定：探测两端机械限位，算出行程与 `rad_to_mm`，并存盘。它沿用已加载标定声明的方向 —— 堵转分不出撞到的是哪一端。 | `CalibrationData` |
| `enable(retries=None)` | 下发使能并回读状态帧，反复重试直到回读到 `err == 1`。 | `EnableResult` |
| `disable()` | 失能电机（零力矩，可用手推动）。 | `bool` |

这六个也都挂在 `gripper.actions`（一个 `GripperActions` 实例）上 —— 上面那些高层方法
就是转发到它的。要在它之上再包一层驱动（比如 ROS 2 节点），直接用 `gripper.actions`。

### 进度回调

运动接口自己不打印任何东西，而是通过 `progress` 回调上报：它按采样频率被调用，收到一个
`MoveProgress` 快照。`zero()` 是个例外 —— 它转发给 `calibrate()`，而后者会把自己的探测
进度打到 stdout。

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

`grasp` 保力期间 `p.phase` 是 `"hold"`，此时 `p.total_steps` 为 `0`。

## 调参：MotionConfig

所有可调量都收在一个 dataclass 里，即 `MotionConfig`。默认值就是在真机上调好的那组。
可以按实例通过 `gripper.motion_config` 覆盖，也可以按调用覆盖。

```python
from litegrip import LiteGrip, MotionConfig

with LiteGrip("can0") as gripper:
    gripper.motion_config = MotionConfig(speed_mm_s=25.0, force_n=10.0)
    gripper.close()

    gripper.motion_config.force_n = 5.0     # 也可以逐个字段改
    gripper.grasp()
```

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `speed_mm_s` | `50.0` | `open` / `close` 的开口速度 |
| `grasp_speed_mm_s` | `50.0` | `grasp` 闭合段速度 |
| `margin` | `0.05` | 距限位保留的行程比例，只有 `grasp` 用 |
| `frame_interval` | `0.005` | 斜坡帧间隔 s（200 Hz） |
| `sample_interval` | `0.05` | 堵转采样间隔 s（20 Hz） |
| `settle_s` | `0.3` | 斜坡走完后原地保目标的时长（这段不判堵转） |
| `reach_tol` | `0.02` | 判定「到位」的位置容差 rad |
| `stall_cycles` | `5` | 堵转窗口的采样点数 |
| `stall_ratio` | `0.2` | 窗口位移低于「本该走的距离」的这个比例即为堵转 |
| `stall_delta` | `0.0015` | 堵转阈值下限 rad |
| `max_lead_mm` | `4.0` | 行进段指令最多领先实测位置多少 |
| `press_overshoot` | `0.05` | `open` / `close` 的目标**越过**限位的行程比例 |
| `press_zone_mm` | `2.0` | 距限位这么近，领先上限就降到 `stop_lead_mm` |
| `stop_lead_mm` | `0.7` | 压紧段的领先上限，压紧力矩约 `kp × stop_lead_mm` |
| `stop_tol` | `0.02` | 停稳位置距标定限位多近才算顶到位 rad |
| `force_n` | `20.0` | `grasp` 默认夹持力 |
| `hold_interval` | `0.2` | 保力的分片时长 s |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | 保力时用的刚度 / 阻尼 |
| `enable_retries` / `enable_retry_interval` | `3` / `0.2` | 使能重试次数与间隔 |
| `calib_kp` / `calib_kd` | `20.0` / `2.0` | `zero()` 探测用的刚度 / 阻尼 |
| `calib_step_rad` | `0.05` | `zero()` 的探测步长，同时也是指令领先实测位置的上限 |
| `calib_tau_limit` | `2.0` | 探测的力矩上限 Nm —— `\|tau\|` 一到就停 |
| `calib_stall_delta` / `calib_stall_cycles` / `calib_max_iter` | `0.0015` / `5` / `200` | 探测的堵转判据 |
| `sleep_fn` / `monotonic_fn` | `time.sleep` / `time.monotonic` | 给测试和仿真留的缝 |

`sleep_fn` 与 `monotonic_fn` 是官方推荐的仿真入口：引擎内部只调这两个，所以传
`sleep_fn=lambda _: None` 就能让整条斜坡不花任何真实时间跑完。`tests/` 下的测试正是
这么在没有任何硬件的情况下跑起一套假 CAN 的。

## 结果类型

结果类型都是 dataclass，同时实现了 `__bool__`，所以老的 `if gripper.open():` 写法仍然能编译。
但 `open` 和 `close` 的**含义**变了 —— 见下。

| 类型 | 字段 | `bool()` 等于 |
| --- | --- | --- |
| `MoveResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`limit_rad`、`final_cmd_rad`、`steps` | `ok` |
| `GraspResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`force_n`、`cycles` | `ok` |
| `EnableResult` | `ok`、`state`、`tries` | `ok` |

`MoveResult.ok` 的含义随谁产生的而变：

- `open()` / `close()` 来的：成功就是**顶到机械限位**，所以成功时
  `ok=True`、`stalled=True`、`reached=False` —— 目标故意越过限位，夹爪永远走不到那儿。
  这两个接口的 `reached` 基本恒为 `False`，读 `ok`。堵转点离限位很远说明行程被挡住，
  这时 `ok=False`。
- `grasp()` 闭合段来的：成功 = `reached and not stalled`，与以前完全一致 —— 夹住工件
  本来就到不了空载目标位置。

有三种组合看着吓人，其实是对的：

- **`grasp` 夹到真实工件**会返回 `stalled=True, reached=False`，而 `ok=True`。夹住东西
  本来就到不了空载目标位置，所以要读 `ok`，别读 `reached`。
- **`close` 空载**报 `ok=True, stalled=True, reached=False` —— 夹爪已经停在机械限位上。
  改之前同样这一下报的是 `reached=True, stalled=False`。
- **`grasp` 闭合段空载**可能报 `reached=True`，但夹爪实际停在 `target_rad` 前约
  `0.010 rad` 处。这是闭合侧的粘滑死区，也正是 `reach_tol` 默认取 `0.02` 的原因 ——
  容差再紧一点就会误报失败。

## 运动逻辑

如果夹爪表现异常，这一节值得一读。

- **连续斜坡，不用 `goto_rad`。** `goto_rad` 在整段时长里反复下发同一个目标，伺服几十
  毫秒就贴上去、剩下时间空转 —— 慢速时表现为一顿一停。这六个接口按 `frame_interval`
  推进一条线性斜坡，并给关节速度前馈，和 SDK 自己的 `move_at_speed` 是同一套做法。
- **指令领先量有上限，分两档。** 指令是一条绝对斜坡，但领先实测位置的部分被封顶：行进段
  用 `max_lead_mm`，足够破静摩擦；距限位 `press_zone_mm` 之内收到 `stop_lead_mm`，压紧力矩
  就停在 `kp × stop_lead_mm` 附近，不会一路涨到 `kp × press_overshoot`。保压段也要切 ——
  那时指令停在限位外侧，不切的话领先会顶到约 8 Nm。不封顶的话，被挡住时误差会一直累积、
  力矩顶到危险值；封顶后力矩始终有界，约为 `kp × 领先上限`。反过来也不能全局调小：领先量
  太小破不了静摩擦，夹爪会在半路报假堵转。更不能改成「相对实测位置加一块」：那样一旦夹住，
  指令就跟着实测冻结，误差永远涨不上去，堵转判据会误判。
- **`open` 和 `close` 的目标在限位外侧。** 目标是标定限位再加 `press_overshoot` 比例的行程，
  由堵转判据在机械限位上结束这趟运动。于是标定出来的极限值只决定**往哪个方向走**和 mm 显示
  读多少 —— 标定偏一点不再影响终点位置。顺带也不再依赖 `margin` 猜得准，不用再跟闭合侧
  那点粘滑死区较劲。`grasp` 不一样：它必须停在**工件**上，闭合段仍旧瞄限位内侧的 `margin`。
- **堵转判据是软件侧的窗口逻辑。** DM4310 本身没有堵转保护，所以引擎每 `sample_interval`
  采一次位置，当连续 `stall_cycles` 次采样的净位移低于
  `max(stall_delta, stall_ratio × 本该走的距离)` 时判堵转。只看单点会被闭合侧的粘滑死区
  带偏。斜坡走完的保压段不判，因为那时夹爪本来就该不动。
- **`enable` 是单向命令，所以要校验。** 使能只是一帧 CAN，没有确认，丢帧了也不会报错，
  电机就静默地没使能。所以 `enable()` 下发后会回读状态帧，只有 `err == 1` 才算成功，
  最多重试 `enable_retries` 次，遇到真实故障会先清故障再重试。

## 力矩、力与安全

- **N 到 Nm 的换算是近似值。** `UnitConversion.N_TO_NM` 为 `0.1`，SDK 自己也标注了
  approximate；真实夹持力还受指爪几何影响。把 `force_n` 当成一个可复现的设定值，而不是
  标定过的测量值。
- **DM4310 的限制**：额定 3 Nm、峰值 7 Nm、协议/固件上限 10 Nm。默认 20 N（前馈 `2.0 Nm`）
  在额定值以内。
- **方向约定是数据，不是开关。** 两端极限都存在标定文件里，**哪个数值更大**就说明闭合往哪个
  方向走（`GripperConfig.close_sign`）。所以反装是一份完全正常的配置，不是错误。真正会被
  `CommandError` 拒绝的是「从没标定过」（`GripperConfig.calibrated` 仍为 `False`）和「两个
  限位相等」，因为那时方向全是猜的。
- **压紧力矩是有界的。** 行进段的 `max_lead_mm` 上限约 5 Nm；压紧段的 `stop_lead_mm` 上限
  约 `kp × stop_lead_mm / rad_to_mm` —— 默认（`kp=100`、`stop_lead_mm=0.7`、`rad_to_mm≈74`）
  约 `0.94 Nm`，约为额定 3 Nm 的三分之一。上限再往下调没有意义：低于一帧的位移
  （`speed_mm_s × frame_interval`，默认 0.25 mm）就会把斜坡自己那一格切掉。压不实就往上调
  `stop_lead_mm`（表现为夹爪冲进来后压不住、停稳位置超出 `stop_tol`，于是 `open`/`close`
  报 `ok=False`），调到能稳定压住、又听不到撞击声为止；默认值只是算术结果，不是真机实测值，
  必须在真机上确认。
- **持续压紧会让线圈发热。** `open` 和 `close` 现在每次都顶着限位走完保压段，连续跑要留意
  线圈温度。
- **`open`、`close`、`grasp`、`zero` 会主动撞向机械限位或持续施力。** 除非就是要夹它，
  否则手和物体别留在行程里；跑 `zero()` 前先把夹爪托住。
- **断连默认会失能**，所以进程崩了不会留下电机一直顶着力。只有在别的东西还在管着夹爪时，
  才把 `disable_on_disconnect=False`。

## 从 2.2.0 的接口迁移

运动方法整个重写了，所以签名变了。返回值从裸 `bool` 变成 dataclass；`__bool__` 保住了
真假判断，但旧的位置参数写法会直接抛 `TypeError`，而不是悄悄做出另一种行为。

| 2.2.0 | 现在 |
| --- | --- |
| `open(kp=None, kd=None, duration=1.0)` | `open(speed_mm_s=None)` |
| `close(kp=None, kd=None, force_n=None, duration=1.0)` | `close(speed_mm_s=None)`，力移到 `grasp()` |
| `grasp(force_n=10.0, kp=150.0, kd=2.0, duration=3.0, stall_threshold=0.001, stall_cycles=5)` | `grasp(force_n=None, hold_s=0.0)`，默认力改为 20 N |
| `enable()`，`err` 为 `0` **或** `1` 都返回 `True` | `enable()`，返回 `EnableResult`，只有 `err == 1` 才算成功 |
| `disable() -> bool` | `disable() -> bool`（不变） |
| 每次调用传增益，如 `close(kp=150.0)` | 增益归 `GripperConfig`，运动可调量归 `MotionConfig` |
| — | `zero()`、`gripper.actions`、`gripper.motion_config`、`disable_on_disconnect` 都是新增 |
| 假定闭合 = rad 增大 | 两种顺序都行 —— `close_sign` 自动推导，装法由 `CALIB_TEMPLATES` 声明 |
| `pos_open_rad >= pos_closed_rad` 抛 `CommandError` | 这个顺序就是反装；改为拒绝「从没标定过」（`GripperConfig.calibrated`） |

除了签名，还有这些行为变化要注意：

- `enable()` 现在会如实报失败。以前只要状态帧是 `0` 或 `1` 它就返回 `True`，于是「根本没
  使能」的电机看起来是使能的。那些忽略返回值继续往下跑的代码，现在会在启动时看到
  `HardwareError`。
- `zero()` 不等于 `calibrate()`。两者现在用同一套默认值、都把指令领先量限成一步、
  都在 `tau_limit` 到顶时停下；区别只在于 `zero()` 取 `MotionConfig.calib_*` 并**存盘**，
  而 `calibrate()` 直接收参数、不存盘。
- 探测在硬限位处是安全的。旧实现无条件外推目标，顶住之后指令每拍继续领先，`kp × 误差`
  随之上涨直到结构崩掉。现在每个循环都从实测位置重新算目标（领先量 ≤ `calib_step_rad`），
  且 `|tau|` 一到 `calib_tau_limit` 立刻中止 —— 这道护栏不依赖位置式堵转判据，后者在结构
  还在让位时永远不会成立。
- `MoveResult.__bool__` 以前是 `reached and not stalled`，现在是 `ok`。对 `grasp` 闭合段
  两者一致；对 `open` 和 `close` 恰好相反 —— 成功顶到限位是 `stalled=True,
  reached=False`，所以 `if gripper.close():` 的含义变了，尽管类型没变。
- 运动接口现在拒绝在「从没标定过」的配置上运行。以前依赖 `GripperConfig` 占位默认值就能动
  夹爪的代码，现在会抛 `CommandError`，必须先 `load_calibration()`。那两个默认值本身也对调了，
  现在读起来是正装而不是反装。
- 压紧段的领先上限 `stop_lead_mm` 从 `1.0` 降到 `0.7` mm，所以 `open` / `close` 顶限位的力矩
  从约额定值的一半降到约三分之一。哪台压不实就把它调回去。
- `save_calibration()` / `load_calibration()` 的默认路径改成**按通道**：
  `~/.litegrip/<channel>_calibration.json`，不再是单个 `litegrip_calibration.json`。旧文件仍
  作为回退读取，`LITEGRIP_CALIB` 仍覆盖一切，但 `can0` 的标定不再回答 `can1` 的加载。
- 自动加载链会跳过声明了**别的**通道的候选文件。`can1` 自己没有标定时，以前会悄悄加载
  `can0` 的，现在返回 `False`、运动接口抛 `CommandError`。大声失败好过朝错的方向走。
- 按名字加载（`mount=` / `load_template`）**绝不**回退出厂文件。模板读不出来就抛
  `CommandError`：出厂文件是正装，回退等于拿「正装」回答「反装」的请求。
- 两份模板现在只带方向与几何（限位、`rad_to_mm`、电机型号），不带 `can_id` / `mst_id` / 增益。
  加载装法不会再改写你传的 CAN ID 或调好的 `kp`/`kd`。`CALIB_TEMPLATES` 及其键名不变。

## 开发

测试套件与硬件无关：它通过 `tests/fake_can.py` 里的运动学假 CAN 驱动引擎，所以在没有夹爪、
没有 CAN 接口的机器上也能跑。

```bash
python3 -m unittest discover -s tests -t tests -v
```

`litegrip.__version__` 是从已安装发行版的元数据里读的，所以装出来的 wheel 报的就是真正
发布的那个版本。从没 `pip install` 过的源码树没有元数据可读，会报 `0.0.0+source` —— 这是
预期行为，不是构建坏了。

## 相关仓库

| 仓库 | 角色 |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 驱动 |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 驱动 |

## 仓库规范

本仓遵循 NEXFORM ROBOTICS 的共享仓库规范：见 [AGENTS.md](AGENTS.md) 中的 agent 操作规则、
Conventional Commits，以及每次合并到 `main` 时自动执行的 semantic-release 版本管理。

## 许可

Copyright © 2026 NEXFORM ROBOTICS。基于 [Apache License 2.0](LICENSE) 许可。
