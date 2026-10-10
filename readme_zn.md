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
| 可选扩展 | `litegrip[zenoh]` —— 点对点 zenoh 遥操链路 |

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
`load_calibration(CALIB_TEMPLATES["reverse"])`；四者加载的是同一个文件。两份模板带的都是出厂
这台机器的几何，只有「哪个限位算闭合」互换：它们负责声明方向，随后 `zero()` 会把限位和比例
换成你自己测的。名字不在 `list_templates()` 里会抛 `CommandError` 并列出合法名字；模板读不出来
时**直接抛**、不回退 —— 出厂文件是正装，拿它回答「反装」的请求，正是这个按名字选的入口要防的
失败。

`zero()` 唯一需要你给的、SDK 自己量不出来的数，是两爪的行程（mm，卡尺读数）。放进
`GripperConfig.max_stroke_mm` —— 默认 85 mm，就是出厂这台：

```python
gripper.config.max_stroke_mm = 85.0     # 你的卡尺读数
gripper.zero()                          # 探两端限位，推出 rad_to_mm，写盘
```

它推出的比例是**行程加上探针的压入量**，除以探到的跨度：

```text
rad_to_mm = (max_stroke_mm + STOP_INSET_MM) / 探到的跨度_rad
```

压入量不是凑出来的数。探针是**压进**张开限位约 1 mm 才找到它的，所以记录下来的两端比两爪真正
能走的行程宽出这么多（`GripperGeometry.SPAN_MM` 86 mm，对 `JAW_TRAVEL_MM` 85 mm）。搞错它会
得到两个等大反向的误差：只拿卡尺读数除以探到的跨度，每个按 mm 走的目标都短 1.2%（满行程丢
1 mm）；而把探到的跨度填进 `max_stroke_mm`，等于把那 1 mm 又加了一遍。出厂这台用默认值重推，
结果和出厂文件自带的比例分毫不差：`86 / 1.409552 = 61.0123 mm/rad`。

只有当你量到的行程不是 85 mm 时，才需要设 `max_stroke_mm`。这是 SDK 唯一要求的单机几何量，而且任何
标定文件都不带它 —— 文件自带的 `rad_to_mm` 是照读的，所以这个数只在重新标定时起作用。

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

主端并不是一上来就卸力：它先**带着增益顶着**自己的爪子，并从第一拍就照常发布张开度，直到从端
报告「已就位」。这样操作者就无法在一个还在移动的从端下面把目标推走。

### 传输

链路是**点对点 zenoh** —— 与真机上跑的遥操同一套结构。两端都是 `mode="peer"`，**关掉全部
发现机制**（无多播、无 gossip），所以两端只能靠显式端点互相找到：主端监听一个 TCP 端口，从端
连到主端的地址。话题是共用的 litearm 命名空间 `litearm/v4/{grip_id}/gripper_teleop`，帧格式
与 litearm 那套逐字节相同 ⇒ 两边可以互通。

就绪握手走同一个会话上的**第二个兄弟话题** —— `litearm/v4/{grip_id}/gripper_ready`，只带一个
字节 —— 于是遥操帧仍与 litearm 那套逐字节相同。发布方是从端，订阅方是主端：主端本来就监听，
所以反向通道不需要新端口。

zenoh 是**可选依赖**，基础 SDK 仍是「标准库 + SocketCAN」：

```bash
pip install 'litegrip[zenoh]'
```

`link="udp"` 可切到明文 UDP 回退方案（仅限可信局域网，无鉴权、无加密）。要给自定义传输，传
`transport=` 一个 `TeleopTransport`；注入的传输不会被 SDK 关闭。

```python
from litegrip import LiteGrip

# 主端：监听并发布本夹爪的张开度。
with LiteGrip("can0") as master:
    master.load_calibration()
    master.enable()
    master.teleop_start("master")                      # zenoh，gripA，端口 17448

# 从端：连到主端，按固定速度走到首帧，然后跟随。
with LiteGrip("can0") as slave:
    slave.load_calibration()
    slave.enable()
    slave.teleop_start("slave", host="192.168.1.20")
    while True:
        print(slave.teleop_status())   # frames, openness, loop_hz, stale, ...
```

`examples/teleop.py` 可以在命令行跑其中一端：

```bash
# A 机 —— 你用手掰的主夹爪：
python3 examples/teleop.py --mode master --channel can0
# B 机 —— 从夹爪：
python3 examples/teleop.py --mode slave  --channel can0 --host 192.168.1.20
```

脚本走同一套握手：主端带增益保持到从端报到为止，`--ready-timeout` 给这段等待封顶（`0` 表示
一直等），`--no-require-ready` 退回「立即交还手控」。

两端必须共用 `grip_id`（默认 `gripA`），且都已连接、已使能。遥操是互斥的：后台循环独占 CAN
读写，在 `teleop_stop()` 之前不要再从调用方驱动夹爪。`teleop_start` 返回初始的
`teleop_status()`；`teleop_status()` 报告 `active`、`mode`、`topic`、`frames`、
`last_frame_age_ms`、`stale`、`openness`、`position_mm`、`force_n`、`dq_cmd`、`loop_hz`、
`rejected`、`send_failed`、`fault`、`torque_nm`、`over_torque`、`torque_trips`、`ready`、
`ready_rx`、`ready_pubs`、`ready_timed_out`，主端另有
`matching`。`position_mm` 与 `force_n` 是主端自己的状态，或从端刚刚跟随的那一帧里主端的值 ——
调用方不必再开一路 CAN 读就能显示夹爪。

只想验**一个**夹爪时，`--fake-leader` 用一个进程内总线上的假主端替掉远端：它的张开度按
张开 → 闭合 → 再张开 扫，于是可以拿单个夹爪去顶一个硬物，观察力矩保护跳闸与重新武装 —— 不需
要第二个夹爪，也不需要网络。先往两指之间放个硬物：没有东西可顶时从端会自由闭合，保护没有可
演示的对象。

```bash
python3 examples/teleop.py --mode slave --channel can0 --fake-leader --torque-limit 1.0
```

- **对齐是一段限速斜坡，而且对齐时任何一帧的指令都不会领先实测位置超过 `lead_cap_mm`。**
  对齐原本是一次 `goto_rad(..., duration=1.0)`。`duration` 看着像斜坡，但 CAN 层从**第一帧**起
  就发 `q = q_target`，只是在截止时间前一直重发，所以那一帧要求的力矩是 `kp` 乘上整个误差 ——
  出厂 `kp = 100.0` Nm/rad 下，误差超过约 0.1 rad 就把 DM4310 顶到饱和；真机上一次从端就是
  这样冲进闭合硬限位、把打印限位撞断的。现在的对齐是按 `align_speed_mm_s`（默认 `50` mm/s 行程
  速度）的匀速 schedule，并把同一速度前馈成 `dq`；其中每一帧都过一层领先上限：指令位置最多
  领先实测位置 `lead_cap_mm`（默认 `4` mm）。力矩是 `kp * (q_cmd - q_measured)`，所以这从构造上
  就给出了对齐的力矩上限 —— 配置默认值下（`kp = 100`、`rad_to_mm = 75.44`）约 5.30 Nm。
  封顶不等于慢下来：夹爪照样走，只是在追赶期间
  顶的力有界。**跟随环则故意不封顶** —— 它直接把主端的 openness 发出去，跟手才好；跟随的过载
  保护是 `torque_limit_nm`。`lead_cap_mm=0` 关闭对齐的封顶；`align_speed_mm_s` 必须 > 0。
  命令行对应 `--align-speed` 与 `--lead-cap`。对齐现在也走循环自己的发帧路径，所以
  `torque_limit_nm` 覆盖得到它 —— 对齐途中卡住会就地卸载，而不是一直顶到动作结束。
- **主端带增益顶着爪子，直到从端报告就绪。** 从端对齐上首帧、且不再 `stale`、未跳闸、并且与
  该帧目标相距不超过 `ready_tolerance_mm`（默认 `2.0`）之后，会在
  `litearm/v4/{grip_id}/gripper_ready` 上宣布自己就绪；主端直到这时才卸力进入零重力。它**全程
  照常发布张开度** —— 那些帧正是从端对齐的对象，不发就会把握手卡死 —— 被门控的只是「交还
  手控」这一步。`require_ready=False` 退回「立即卸力」，`ready_timeout_s`（默认 `10.0`；`0`
  表示一直等）给等待封顶，这样从不宣告就绪的旧版从端不会把主端卡住：超时后主端告警并照样卸力。
  传输无法订阅的主端会关掉这道门、和以前一样立刻卸力，明文 UDP 没有反向通路，门在那里等于
  失效。`teleop_status()` 里的 `ready`、`ready_rx`、`ready_pubs`、`ready_timed_out` 报告状态。
- **从端把主端的速度前馈下去。** 线上帧只带 openness，所以从端用相邻两帧的差分还原出速度，作为
  电机的 `dq` 目标下发 —— 机械臂遥操是直接发 `dq` 的。没有这一项，从端只能靠位置误差出力，会
  明显拖在运动中的主端后面（滞后量 ≈ 速度 / `kp`）。因为 `kd * dq` 是实打实的力矩项，这个估计
  是有界的：首帧（没有可差分的前一帧）、退化的时间间隔、以及超过 `MAX_FRAME_GAP_S` 的断流都取
  `dq = 0`，结果再钳到 `dq_max`（默认 `10.0` rad/s；`dq_max=0` 关闭前馈）。
- **从端与主端失联时是「持位」，不是「卸力」。** 超过 `watchdog_s`（默认 `0.2`）没有新帧后，
  它仍按跟随增益顶着上一个目标继续发帧 —— 于是 `stale` 变真，但爪子停在原地，可能夹住中间的
  东西。
- **可选的力矩保护会让顶得太狠的从端就地卸力。** 设了 `torque_limit_nm` 后，从端每拍都看自己
  的力矩（电机不上报原始电流，力矩由线圈电流导出）；达到或超过上限连续 `TORQUE_TRIP_CYCLES`
  （3 拍，50 Hz 下 60 ms）就把刚度和阻尼**就地**归零 —— 爪子不再顶，但循环不停、帧照发，电机
  不会因此锁存断流故障。要等主端**重新张开** `TORQUE_REARM_OPENNESS`（0.05）才重新武装，所以
  是「松手」而不是对着同一个障碍反复蹭。这个目标以全开限位封顶：跳闸点离全开不足 0.05 时没有
  可退的行程，退到全开限位即算重挂；没有这个封顶，那样的跳闸会把守护锁死到本次会话结束。默认
  值是 `0`，也就是**不设防、要显式打开**。数值要按机器定：它取决于两指之间那个件有多脆，而且
  跟随增益是 Nm/rad（`kp` 默认 `100.0`），所以很低的上限只对应一点点位置误差 —— 先在真实顶压
  时看 `torque_nm` 再定这个数。`teleop_status()` 里的 `over_torque` 与 `torque_trips` 报告状态。
- **非有限值帧一律丢弃，绝不夹位。** NaN 会原样穿过 `[0, 1]` 的钳位，再被折到某个端点 —— 静默
  地把从端指到全闭限位。两端都在**协议边界**上拒收 NaN / ±inf（包括对齐用的首帧），计入
  `rejected`，并改为持位。
- **从端每拍都把目标夹进自己的标定行程**，并且会看 SDK 的返回值：`send_mit_frame` 返回 `False`
  会计入 `send_failed`，夹爪自报的 `error_code` 不是「已使能」会记进 `fault` —— 都不吞掉。
- **停止后是持位**，不是卸力：主端在 `teleop_stop()` 时退出零重力模式，从端在收尾时按当前角度
  补发一帧 —— 两边都按配置增益持位，都不失能。唯一的例外是刚触发过力矩保护的从端：它的收尾帧
  保持零增益，因为重新加上增益就会去顶它刚刚松开的那个东西。
- 未标定、行程为零、或 `rad_to_mm == 0` 的夹爪会**拒绝启动**（`TeleopNotReady`），且在使能或
  驱动之前就拒掉。
- 跟随增益默认取标定里的 `kp` / `kd`（出厂是 `100.0` / `2.0`），用 `kp=` / `kd=` 覆盖。

## 轨迹录制与回放

你用手教一遍的动作可以录下来，之后反复重放。录制时电机进零重力，你直接掰爪子走完整个动作；
回放把录到的张开度按 MIT 指令帧发回去。存的是归一化到 `[0, 1]` 的张开度，和遥操线上传的是同一个
量，所以在一台夹爪上教出来的轨迹，换一台装法不同、标定不同的夹爪也能重放。

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:
    gripper.load_calibration()
    gripper.enable()

    taught = gripper.record(5.0)   # 手把手教 5 秒，期间爪子是卸力的
    taught.save("pick")            # ~/.litegrip/trajectories/pick.lgt
    gripper.play(taught)           # 重放
```

| 方法 | 行为 |
| --- | --- |
| `record(duration_s, rate_hz=100.0, zero_gravity=True)` | 阻塞式手把手录制，返回 `Trajectory`。 |
| `record_start(rate_hz=100.0, zero_gravity=True, max_samples=None)` | 后台录制，返回状态快照。 |
| `record_stop(allow_empty=False)` | 停止并返回录到的 `Trajectory`。 |
| `play(trajectory, speed=1.0, kp=None, kd=None, align=True)` | 阻塞式回放。`loop` 必须是 `False`。 |
| `play_start(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` | 后台回放。 |
| `play_stop(timeout=2.0)` | 停止回放，并让夹爪持位。 |
| `trajectory_status()` | 两个方向共用一个快照。`active`、`kind`、`samples`、`error` 一直都在；录制时另有 `rate_hz`、`zero_gravity`、`loop_hz`，回放时另有 `frames`、`speed`、`openness`、`completed`。 |

`examples/trajectory.py` 在命令行做同样的事：

```bash
python3 examples/trajectory.py --record 5 --save pick   # 手把手录一段再存盘
python3 examples/trajectory.py --list                   # 不需要接硬件
python3 examples/trajectory.py --play pick --repeat 3
```

`Trajectory.save("pick")` 写到 `~/.litegrip/trajectories/pick.lgt`；带路径分隔符的名字按原样
使用。目录可以用 `LITEGRIP_TRAJ_DIR` 改。`Trajectory.load("pick")` 读回来，`--list` 每个文件
打一行。格式是紧凑二进制，开头 8 字节魔数；文件长度和头部声明的采样数对不上的会被拒绝，而不是
解析出半截轨迹。

- **回放的是位置，不是力。** 录到的力矩只是诊断信息，不会前馈下发，所以对着物体挤出来的那段，
  重放时是一条位置轨迹，按 `kp` 顶上去 —— 你教的那个夹持力不会复现。力重要的话，回放完再调
  `grasp(force_n=...)`。
- **`record()` 期间独占，而且爪子全程卸力。** 它自己持续发零力矩帧，所以录制期间不要再从调用方
  驱动夹爪，也要用手扶着：这期间没有任何东西托着爪子。
- **有别的东西在驱动时，用非零重力模式录。** `record_start(zero_gravity=False)` 只读状态，调用方
  可以在另一个线程里跑 `grasp()` 或一串运动，把它录下来。要录程序化的动作就走这条路。
- **没录满会报错。** `record()` 会说清只录到几拍，而不是把一段短录制当成完整结果返回；采样循环
  死掉也不会被报成一次好录制。
- **阻塞式 `play()` 返回时只发了一帧持位。** 电机在停帧约 100 ms 后会因通信丢失自锁，所以要接着
  调下一个动作 —— 想要持续持位就用 `play_start(loop=True)` 配 `play_stop()`。只有一拍采样的轨迹
  是一个姿势、没有可循环的行程，循环它就等于一直保持那个张开度。
- **录制、回放、遥操三者互斥。** 它们都独占 CAN 读写，起第二个会抛 `TeleopBusyError` 或
  `TrajectoryBusyError`。`disconnect()` 会把正在跑的那个停掉。
- 录制和回放都要求已加载标定：没有标定，归一化的张开度算不出来。

## 六个动作接口

要让夹爪动起来就用这六个。每一个都会自己校验结果再报成功，所以调用方不必再重写斜坡和
堵转判据。

| 方法 | 行为 | 返回 |
| --- | --- | --- |
| `open(speed_mm_s=None)` | 按斜坡**越过**标定的张开侧限位，由机械限位结束这趟运动。设了 `GripperConfig.work_stroke_mm` 就改为停在那段开口处，不顶限位 —— 出厂标定把它设成 `80.0`，所以普通一次 `open()` 停在记录张开端内侧 6 mm 处，即两爪满行程前 5 mm。 | `MoveResult` |
| `close(speed_mm_s=None)` | 同上，朝闭合侧。`work_stroke_mm` 是张开侧的数，不改变闭合停在哪儿。 | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | 闭合到堵转（即夹住），然后爬到并持续输出 `force_n`。闭合段带着 `force_n` 当力矩预算走，所以撞上工件时压出的力不超过设定值。`hold_s=0` 表示不限时长。 | `GraspResult` |
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
| `max_lead_mm` | `4.0` | 不带设定力的行进段，指令最多领先实测位置多少 |
| `press_safety` | `0.9` | 带设定力的接近段允许花掉设定值的几分之几，让实测力落在设定值之下 |
| `press_overshoot` | `0.05` | `open` / `close` 的目标**越过**限位的行程比例 |
| `press_zone_mm` | `2.0` | 距限位这么近，领先上限就降到 `stop_lead_mm` |
| `stop_lead_mm` | `0.7` | 压紧段的领先上限，压紧力矩约 `kp × stop_lead_mm` |
| `stop_tol` | `0.02` | 停稳位置距标定限位多近才算顶到位 rad |
| `force_n` | `20.0` | `grasp` 默认夹持力 |
| `force_ramp_n_s` | `20.0` | 保力力矩爬到设定值的速率 N/s |
| `hold_interval` | `0.2` | 保力的分片时长 s |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | 已废弃 —— 保力不用增益，设了也不生效 |
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
- **带设定力的接近段，花的就是那个设定值。** `grasp` 的闭合段仍然是指令位置帧，所以撞上
  工件时力矩由驱动器自己算出来：`kp × 领先 + kd × 指令速度` —— 定这个数的是这趟走多快、
  引擎允许指令领先多少，而不是你要的那个力。配置默认值下光领先上限就是
  `kp × max_lead_mm / rad_to_mm` = 5.30 Nm（约 53 N），而且不管要 5 N 还是 40 N，
  撞上去都是这 53 N。于是这一段把自己的预算
  （`force_n × 0.1 × press_safety`）按顺序交给一帧能花掉它的三个项：本帧自己的位移
  （`kp × v × dt`，只有速度能覆盖它）、阻尼（`kd`）、最后才是领先。撞上工件时压出的力
  因此约为 `0.9 × force_n` —— 5 N 的夹取是 4.5 N，40 N 的是 36 N。有两个后果值得知道。
  领先跟着设定力变窄：40 N、50 mm/s 时是 1.7 mm，20 N 时是 0.34 mm。而在默认
  `grasp_speed_mm_s` 下低于约 `3.7 N` 时，**速度**由设定值决定 —— 那点速度走一帧就已经
  压满整个预算：1 N 的夹取以 13.4 mm/s 闭合，0.1 N 的以 1.3 mm/s。这是有意的：0.1 N
  本来就没法快速接近，所以别去要一个机构自身摩擦就能吃掉的力；想限时改用
  `grasp_speed_mm_s`。
- **`open` 和 `close` 的目标在限位外侧。** 目标是标定限位再加 `press_overshoot` 比例的行程，
  由堵转判据在机械限位上结束这趟运动。于是标定出来的极限值只决定**往哪个方向走**和 mm 显示
  读多少 —— 标定偏一点不再影响终点位置。顺带也不再依赖 `margin` 猜得准，不用再跟闭合侧
  那点粘滑死区较劲。`grasp` 不一样：它必须停在**工件**上，闭合段仍旧瞄限位内侧的 `margin`。
- **堵转判据是软件侧的窗口逻辑。** DM4310 本身没有堵转保护，所以引擎每 `sample_interval`
  采一次位置，当连续 `stall_cycles` 次采样的净位移低于
  `max(stall_delta, stall_ratio × 本该走的距离)` 时判堵转。只看单点会被闭合侧的粘滑死区
  带偏。斜坡走完的保压段不判，因为那时夹爪本来就该不动。
- **保力是指令力矩，不是指令位置。** `grasp` 的保力段和 `set_force` 下发的帧
  `kp = kd = 0`，所以电机的输出就是前馈力矩 `force_n × 0.1 Nm`，和夹爪停在哪里无关。
  力之所以不随工件走，就靠这一点：软工件屈服时夹爪跟着走，夹持力还是设定值。**不要**
  为了「顶得更硬」加增益 —— 夹爪一动，`kp × (q - 实测位置)` 就变成力误差，`kp = 150`
  时闭合侧约 `0.0103 rad` 的粘滑一格就是约 `15 N`。`hold_kp` / `hold_kd` 以前正是干
  这个的，现在已经不生效。
- **保力力矩是爬上去的，不是一步跳上去的。** 进入保力时，力矩按 `force_ramp_n_s`
  （20 N/s）从**飞行中的力矩** —— 也就是闭合压紧量，真机上约 `10 N` —— 每帧走一步爬
  到设定值，正好落在设定值上，而不是逼近它。一步踏进接触里就是隔着机构的一次冲击，
  指爪会被刚碰到的东西弹开；按速率爬，力才是均匀地涨。20 N/s 下，从约 `10 N` 的压紧
  力交接到 `20 N` 设定值要半秒，设定值最大到额定 `40 N` 要两秒。**不要**把爬升改写成
  「时长」：按时长定的斜坡第一拍最陡，等于带慢尾的阶跃。
- **`set_force` 的 `duration` 是爬升**之后**继续保力的时间，不含爬升本身。** 调用先爬到
  设定值，再在设定值上保 `duration` 秒，然后返回，所以它的墙钟时间是 `climb + duration`。
  从松开状态、默认 `duration=0.3` 调 `set_force(20.0)`：先爬约 `1.0 s`，再保 `0.3 s`，
  一共阻塞约 `1.3 s`，以前是 `0.3 s`。力始终会落到设定值，所以 `duration` 给短了照样
  是满力，只是保得短。`duration=0` 表示「爬到设定值就返回」，不保力。
- **`enable` 是单向命令，所以要校验。** 使能只是一帧 CAN，没有确认，丢帧了也不会报错，
  电机就静默地没使能。所以 `enable()` 下发后会回读状态帧，只有 `err == 1` 才算成功，
  最多重试 `enable_retries` 次，遇到真实故障会先清故障再重试。

## 力矩、力与安全

- **N 到 Nm 的换算是近似值。** `UnitConversion.N_TO_NM` 为 `0.1`，SDK 自己也标注了
  approximate；真实夹持力还受指爪几何影响。把 `force_n` 当成一个可复现的设定值，而不是
  标定过的测量值。
- **DM4310 的限制**：额定 3 Nm、峰值 7 Nm、协议/固件上限 10 Nm。默认 20 N（前馈 `2.0 Nm`）
  在额定值以内。
- **保力时夹爪是软的。** 保力帧没有刚度也没有阻尼，所以外力能把夹爪推回去，电机照样只
  按 `force_n` 顶；两爪之间没有东西时，`grasp()` 会以这个力一直合到机械限位。这是「工件
  移动时力也不漂」的代价。
- **方向约定是数据，不是开关。** 两端极限都存在标定文件里，**哪个数值更大**就说明闭合往哪个
  方向走（`GripperConfig.close_sign`）。所以反装是一份完全正常的配置，不是错误。真正会被
  `CommandError` 拒绝的是「从没标定过」（`GripperConfig.calibrated` 仍为 `False`）和「两个
  限位相等」，因为那时方向全是猜的。**每一个**运动方法都会拒绝这种配置 —— 不止上面六个
  动作，`goto`、`goto_rad`、`move_to`、`home`、`move_at_speed*` 同样拒绝。它们把目标
  clamp 进 `pos_closed_rad` / `pos_open_rad`，而在占位默认值上那个 clamp 不是安全网，是
  在描述不了这台机器行程的数字上做算术：全新未标定的对象上 `goto(40.0)` 会映射到
  `+0.61 rad`，越过真机闭合止点 `+0.05 rad`，夹爪被推过去压在那里（指令力矩约 56 Nm）。
  先加载标定 —— 这道闸门就只有这一条。
- **非有限值一律拒绝，绝不夹位。** `goto`、`goto_rad`、`move_at_speed*`、`grasp`、`set_force`
  的参数只要是 NaN 或 ±inf 就抛 `CommandError`；CAN 那一层同样拒收 MIT 帧里任何非有限的
  字段，不让它有机会变成字节。钳位给的答案其实是量程的一端 —— `q → +12.5 rad`、`kp → 500`、
  `tau → +10 Nm` —— 那是一个没人要过的真实指令；`set_force(nan)` 更曾卡在爬升循环里，一帧不
  发也永不返回。前馈力矩超出本型号的帧量程（DM4310 是 `±10 Nm`，DM4340 是 `±28 Nm`）同样
  拒绝：钳到端值等于换成了另一个力。有限的超量程值照旧钳位，与原来一致。
- **压紧力矩是有界的。** 对 `open` 和 `close` —— 那两个不带设定力、直接顶限位的动作 ——
  行进段的 `max_lead_mm` 上限约 `kp × max_lead_mm / rad_to_mm`，压紧段的 `stop_lead_mm` 上限约
  `kp × stop_lead_mm / rad_to_mm`。**`kp` 取的是当下生效的那个，而标定文件里带的 `kp` 会盖过
  `GripperConfig` 的默认值。** 按配置默认（`kp=100`、`max_lead_mm=4.0`、`stop_lead_mm=0.7`、
  `rad_to_mm=75.44`）是 `5.30 Nm` 和 `0.93 Nm`；加载出厂标定后是 `0.33 Nm` 和 `0.06 Nm`，
  因为那份文件把 `kp` 设成了 `5.0`。`grasp` 的闭合段则由它自己的设定值封顶，见「运动逻辑」那
  一节。上限再往下调没有意义：低于一帧的位移（`speed_mm_s × frame_interval`，默认 0.25 mm）
  就会把斜坡自己那一格切掉。压不实就往上调 `stop_lead_mm`（表现为夹爪冲进来后压不住、停稳位置
  超出 `stop_tol`，于是 `open`/`close` 报 `ok=False`），调到能稳定压住、又听不到撞击声为止。
  这里每个数都是上面两条公式的算术结果，不是真机实测值：某台机器在这几个数下压不压得实，必须
  在那台机器上确认；出厂标定的 `kp=5.0` 让它的 `0.06 Nm` 成为第一个要查的点。
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
