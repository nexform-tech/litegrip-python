# litegrip-python

[English](README.md) | [简体中文](readme_zn.md)

**LiteGrip 轻量机械爪系列**的 Python SDK，也是它主要的公开接口。本文帮你把 SDK 装上、
把夹爪跑起来，然后用你自己的程序驱动它。

你不需要懂 CAN，也不需要懂 DM4310 的协议：每一次运动、标定和力控都是一个方法调用，
每个调用在返回前都会自己核对结果。

## 你需要什么

| | |
| --- | --- |
| 电机 | 达妙 DM4310，CAN ID `0x08`，MIT 模式，经典 CAN 1 Mbit/s |
| 主机 | Linux + SocketCAN —— 这是唯一支持的平台 |
| Python | 3.8 及以上 |
| 依赖 | 无，只用标准库 |
| 可选 | `litegrip[zenoh]` —— 点对点遥操链路 |

## 1. 安装

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

## 2. 拉起 CAN 接口

夹爪是一个 CAN 设备。调用 SDK 之前，先把接口以 1 Mbit/s 拉起来，**每次重启后都要重新做**：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

两条命令都需要 root。如果 `can0` 根本不存在，那是 CAN 适配器的驱动还没加载。做产品时，
把这两条命令放进开机启动的 init 脚本，别让应用程序自己去起接口。

## 3. 只有反装才需要声明装法

出厂的都是**正装** —— 闭合时电机的 rad 读数是增大的 —— 而 SDK 随包发布的出厂标定描述的
就是它们。正装的夹爪**什么都不用声明**：下一节的 `load_calibration()` 会给出正确的方向，
第 5 步再确认一遍。

电机反着装的夹爪是**反装**，对它来说闭合是 rad **减小**。SDK 两种都不预设 —— 方向由两个
标定限位的**数值顺序**推出来。你的夹爪是反装的话，按名字声明：

```python
from litegrip import LiteGrip

with LiteGrip("can0", mount="reverse") as gripper:
    gripper.enable()
    print(gripper.mount)    # "reverse" —— 从限位数值读回
    gripper.close()         # 爪子应该合上，而不是张开
```

如果它反而张开，那这台其实就是正装 —— 把参数去掉即可。

`list_templates()` 返回 SDK 接受的全部名字，给 UI 做下拉框用。`gripper.mount` 读回
`"normal"`、`"reverse"`，未标定时是 `None`；名字不在 `list_templates()` 里会抛
`CommandError` 并列出合法名字。

即使几何还不是你这台的，用模板跑运动也是安全的：`open` 和 `close` 永远不会把指令发到标定
行程之外，所以方向选错也顶不出限位。它只会朝错的方向走，而这一点你立刻就能看见。

## 4. 量出行程

`zero()` 是 SDK 唯一需要你给一个它自己量不出来的数的地方：两爪的行程（mm，卡尺读数）。
把它填进 `GripperConfig.max_stroke_mm` —— 默认 `85.0`，就是出厂这台 —— 然后 `zero()` 会
探两端机械限位、推出比例并存盘：

```python
with LiteGrip("can0") as gripper:
    gripper.enable()
    gripper.config.max_stroke_mm = 87.5     # 你的卡尺读数
    gripper.zero()                          # 探两端限位，按通道存盘
```

只有当你量到的行程不是 85 mm 时才需要设 `max_stroke_mm`。这是唯一一个任何标定文件都不带的
单机几何量，而且它只在重新探测时才起作用。

`zero()` 会把爪子顶到两端的真实限位上，所以做的时候手和工件都要离开行程范围；如果夹爪的安装
朝向会让爪子掉下来，先把它托住。

`zero()` 量到的结果写进 `~/.litegrip/<channel>_calibration.json`。这个文件不会被自动读回：
第 5 步跑的是出厂标定，除非你用 `--calib ~/.litegrip/can0_calibration.json` 点名它，或者设
`LITEGRIP_CALIB`。

### 让标定一直可用

所有 LiteGrip 共用同一个 CAN ID（`0x08`），所以一台电脑上放两台时，**通道是唯一的身份键**。
`zero()` **按通道各存各的**：`~/.litegrip/<channel>_calibration.json`，两台互不覆盖。

**出厂标定才是标准值。** 无参 `load_calibration()` 只读这一份，本机量出来的标定绝不会被顺手
采纳 —— 包括 `zero()` 刚写下的那份。要用它就得点名：

```python
from litegrip import LiteGrip, default_calib_path

with LiteGrip("can0") as gripper:
    gripper.load_calibration(default_calib_path("can0"))    # zero() 存下的那一份
    gripper.enable()
```

`LITEGRIP_CALIB=/path/to/file.json` 不改代码就能对每个通道生效。无参加载只要跳过了本机的
文件、且没设 `LITEGRIP_CALIB`，告警就会点名它，并重复上面这两种用法。**不要指望它会自动
生效。**

`~/.litegrip/` 里 SDK 只认一个名字：`<channel>_calibration.json`，也就是 `zero()` 写下的
那个。放在那里的标定文件只要是别的名字，就根本不是候选 —— 不试、不合并、也不是靠后的回退
档。要留意的是 `can0_08_calibration.json` 这种：它是别的工具写的、不是本 SDK 写的，除非你
把路径交给 `--calib` 或 `load_calibration(path=...)`，否则一直不起作用。**不要因为文件就躺在
那个目录里，就以为它正在生效。**

出厂文件声明的是 `channel: can0`，所以 `can1` 根本没有隐式来源，会**大声失败**，而不是悄悄
采纳 `can0` 的方向。要给它一份，用 `LiteGrip("can1", mount="normal")`（模板按名字加载，不
理会通道）或者自己跑一次 `zero()`，再照上面的方式点名回来。

SDK 不猜方向：没有加载标定之前，任何运动接口都会抛 `CommandError`。

## 5. 确认夹爪能动

刚接好线的新设备跑一次，之后每次改标定或修完再跑一次：

```bash
python3 examples/stroke_test.py --channel can0
```

它先开到一端限位、再闭到另一端，打印每一步停在哪里、压了多大力，只有两步都成功才退出码 0。

```bash
python3 examples/stroke_test.py --dry-run                    # 只看计划，不动
python3 examples/stroke_test.py --cycles 3 --speed 10 --yes   # 无人值守
```

## 用 Python 驱动

```python
from litegrip import LiteGrip

with LiteGrip(channel="can0", can_id=0x08) as gripper:
    gripper.load_calibration()
    gripper.enable()                                   # 反复重试，直到状态帧回读到 err == 1

    gripper.open()                                     # 50 mm/s 走到张开侧限位
    gripper.close()                                    # 50 mm/s 走到闭合侧限位
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # 闭合到夹住，再保 20 N 力
    print(result.ok, result.stalled, result.cycles)
```

`with` 退出时会调 `disconnect()`，默认顺带失能电机。如果想让夹爪在离开代码块后仍保持使能，
传 `LiteGrip(..., disable_on_disconnect=False)`。

## 六个动作

这六个方法是驱动夹爪的受支持入口。

| 方法 | 行为 | 返回 |
| --- | --- | --- |
| `open(speed_mm_s=None)` | 朝标定的张开侧限位爬坡。指令被夹在标定行程内，所以爪子停在限位上，永远不会被发到限位之外。设了 `GripperConfig.work_stroke_mm` 时改停在那个开度 —— 出厂标定设的是 `80.0`，所以直接 `open()` 会停在满行程前 5 mm 处。 | `MoveResult` |
| `close(speed_mm_s=None)` | 同上，朝闭合侧，同样的指令夹子。`work_stroke_mm` 是张开侧的数，不改变闭到哪里。 | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | 闭合到堵转（即夹住），然后爬到 `force_n` 并保持。闭合段以 `force_n` 作为力矩预算，所以碰到工件时压得不会比设定值更狠。`hold_s=0` 表示一直保持 —— 直到故障、护栏触发，或 Ctrl+C。`force_n=None` 用 `MotionConfig.force_n`（20 N）。 | `GraspResult` |
| `zero()` | 完整标定：探两端机械限位，推出行程与 `rad_to_mm`，存盘。它保留已加载标定声明的方向 —— 堵转分辨不出是哪一端。 | `CalibrationData` |
| `enable(retries=None)` | 发送使能并回读状态帧，反复重试，直到回读到 `err == 1`。 | `EnableResult` |
| `disable()` | 失能电机 —— 力矩归零，可以用手掰动。 | `bool` |

这些方法也都能通过 `gripper.actions`（一个 `GripperActions` 实例）调用，高层方法就是转发到它。

### 其他标定方式

| 调用 | 什么时候用 |
| --- | --- |
| `zero()` | 默认方式。带电探两端限位并存盘。 |
| `calibrate_guided()` | 引导式两步探测，每到一端停下来等你确认。 |
| `calibrate_manual(duration=30.0)` | 手把手教：先卸力，你用手把爪子推完整个行程，它取两端的极值。 |
| `write_zero()` | 把**当前**编码器角度就地声明成电机零点。不探限位，也不动标定文件。 |

`write_zero()` 会把偏移写进电机的 flash，**无法撤销** —— 你已经量到的跨度仍然有效，但零点
永久移动了。只在你**有意**把爪子停到闭合位置时才用，`examples/zero_closed.py zero` 就是在
一个确认提示后面做这件事。

## 怎么读返回值

返回类型是 dataclass，同时定义了 `__bool__`，所以 `if gripper.close():` 可以用。但它**含义**
随动作而异，要看的字段也不同：

| 类型 | 字段 | `bool()` 是 |
| --- | --- | --- |
| `MoveResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`limit_rad`、`final_cmd_rad`、`steps` | `ok` |
| `GraspResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`force_n`、`cycles`、`travel_guard` | `ok` |
| `EnableResult` | `ok`、`state`、`tries` | `ok` |

**看 `ok`。** 下面三种组合看着吓人，其实都是对的：

- **空爪 `close()`** 返回 `ok=True, stalled=True, reached=False`。`open`/`close` 的成功判据是
  *堵转、且指令被夹在标定限位上、爪子停在离它 `stop_tol` 之内*。这里 `reached` 基本永远是
  `False`，因为目标本来就故意设在限位之外。
- **夹着东西的 `grasp()`** 返回 `stalled=True, reached=False`，`ok=True`。没走到空爪时的目标
  位置，正是夹住的定义。
- **空爪 `grasp()`** 可能返回 `reached=True`，而爪子离 `target_rad` 还差约 `0.010 rad` ——
  这是闭合侧的粘滑死区，也是 `reach_tol` 默认为 `0.02` 的原因。

在离限位很远的地方堵转，才会给 `ok=False`：有东西挡住了行程。

## 力控

`grasp(force_n, hold_s)` 先闭合，再保持一个力。`set_force(force_n, duration=0.3)` 在爪子
**当前**位置上保持一个力，做完返回。

保力段发的帧是 `kp = kd = 0` 的，所以驱动器输出**就是**前馈力矩 `force_n × 0.1 Nm`，
不管爪子在哪儿。正因如此，这个力与工件无关：软工件让位时爪子跟着走，夹持力仍停在设定值上。

三件值得知道的事：

- **这个力是近似的。** `1 N = 0.1 Nm` 在 SDK 里就标着「近似」，真实夹持力还取决于你指爪的
  几何。把 `force_n` 当成一个**可重复**的档位，不要当成标定过的测量值。
- **爪子是柔顺的。** 外力可以把它们推回去，而电机仍用 `force_n` 顶着。空爪 `grasp()` 就是
  用这个力顶到机械限位上。这就是「工件移动时力不漂」的代价。
- **小于约 3.7 N 时，接近速度会变成限制项**，因为 `grasp_speed_mm_s` 下走一帧就已经花光了
  整个力矩预算。不要要求一个机构自身摩擦就能吃掉的力；要限制的是接近时间，改
  `grasp_speed_mm_s`。

### 失控监视

保力帧不带位置误差，所以任何位置限位都管不到它 —— 唯一可用的手段是**撤掉力矩**。因此保力段
的护栏是**判据，不是夹子**：一旦实测位置越过闭合侧标定限位 `hold_runaway_rad`（`0.05 rad`，
约 3 mm），`grasp()` 和 `set_force` 就连发零力矩释放帧并返回 —— `GraspResult.travel_guard`
置位，或 `set_force` 返回 `False`。

只判闭合侧，因为前馈力矩就是朝那边压的。越过张开限位会被同一份力矩自己推回来。

这是**失控检测，不是精度上限**。顶着工件时爪子停在工件上；没顶工件时停在机械限位上，而那
就在标定限位处。只有当**确实夹着东西**的爪子被释放时，才去调大 `hold_runaway_rad`。未标定时
监视不启用：占位默认值描述的不是任何真实行程，拿它当窗口比没有更糟。

需要比默认更大的力？`MotionConfig.force_n` 是 `20.0` N（`2.0 Nm`），在 DM4310 的 `3 Nm`
额定之内。它的峰值是 `7 Nm`，协议上限是 `10 Nm`。

## 进度回传

运动接口从不打印，它们通过 `progress` 回调回传：按采样频率收到一个 `MoveProgress` 快照。

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

`grasp` 保力时 `p.phase` 是 `"hold"`，`p.total_steps` 是 `0`。`zero()` 是唯一的例外：
它自己往 stdout 打印探测进度。

## 调参

所有可调项都在一个 dataclass 上：`MotionConfig`。默认值就是在真机上验证过的值。可以按实例
通过 `gripper.motion_config` 覆盖，也可以逐次调用传参。

```python
from litegrip import LiteGrip, MotionConfig

with LiteGrip("can0") as gripper:
    gripper.motion_config = MotionConfig(speed_mm_s=25.0, force_n=10.0)
    gripper.close()

    gripper.motion_config.force_n = 5.0     # 也可以逐字段改
    gripper.grasp()
```

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `speed_mm_s` | `50.0` | `open` / `close` 的爪速 |
| `grasp_speed_mm_s` | `50.0` | `grasp` 接近段的闭合速度 |
| `force_n` | `20.0` | `grasp` 的默认力 |
| `force_ramp_n_s` | `20.0` | 保力爬到设定值的速率，N/s |
| `hold_interval` | `0.2` | 保力段每片的长度，秒 |
| `hold_runaway_rad` | `0.05` | 越过闭合限位这么多，保力段就撤力矩并停止 |
| `margin` | `0.05` | 离限位保留的行程比例，只被 `grasp` 使用 |
| `max_lead_mm` | `4.0` | 不带力设定值的运动里，指令允许领先实测位置的上限 |
| `stop_lead_mm` | `0.7` | 接近限位时的同一上限，用来压住压紧力矩 |
| `press_safety` | `0.9` | 带力的接近段最多能花掉设定值的多少 |
| `reach_tol` | `0.02` | 判定 `reached` 的位置容差，rad |
| `stop_tol` | `0.03` | 要算「压到位」，爪子得停到离标定限位多近 |
| `settle_s` | `0.3` | 爬坡结束后在目标上停留的时间 |

其余字段 —— 堵转窗口、探测参数、帧时序 —— 都写在 dataclass 自己的注释里，连同默认值背后的
理由。

`sleep_fn` 和 `monotonic_fn` 是做仿真时受支持的接缝：引擎只通过这两个函数取时间，所以
`sleep_fn=lambda _: None` 能把整段爬坡压成不占墙上时间。测试套件就是这样在没有硬件的
假 CAN 上跑的。

## 两台夹爪：主从遥操

两台夹爪可以联动，一台跟着另一台动。**主夹爪**（leader）的电机卸力 —— 你用手掰它的爪子，
它发布自己张开到多少；**从夹爪**（follower）驱动自己的爪子跟到位。线上传的是归一化到
`[0, 1]` 的张开度，不是角度，所以两端不需要相同的标定、装法或零点。

```python
from litegrip import LiteGrip

# 主端：监听并发布本爪的张开度。
with LiteGrip("can0") as master:
    master.load_calibration()
    master.enable()
    master.teleop_start("master")                      # zenoh，gripA，端口 17448

# 从端：连上主端，爬到第一帧，然后跟随。
with LiteGrip("can0") as slave:
    slave.load_calibration()
    slave.enable()
    slave.teleop_start("slave", host="192.168.1.20")
    while True:
        print(slave.teleop_status())   # frames、openness、loop_hz、stale……
```

链路是**点对点 zenoh**，属于可选依赖：

```bash
pip install 'litegrip[zenoh]'
```

两端都用 `mode="peer"` 并关掉全部发现机制，所以两端只能靠显式端点互相找到。`link="udp"` 是
面向可信局域网的纯 UDP 回退；它没有认证也没有加密，无论用哪种，都跑在可信网络上。

`examples/teleop.py` 从命令行跑其中一端 —— 每台机器一个进程：

```bash
python3 examples/teleop.py --mode master --channel can0
python3 examples/teleop.py --mode slave  --channel can0 --host 192.168.1.20
```

只台架测试**一台**夹爪时，`--fake-leader` 用同一个进程内的合成主端替掉真实主端。先在爪子
之间放一个硬物 —— 没有可顶的东西时，从端会自由闭合，力矩护栏就没有可演示的对象。

```bash
python3 examples/teleop.py --mode slave --channel can0 --fake-leader --torque-limit 1.0
```

两端必须共用 `grip_id`（默认 `gripA`），并且都已连接、已使能。遥操是独占的：后台循环占着
CAN I/O，所以在 `teleop_stop()` 之前不要再从调用方驱动夹爪。

- **从端报告就位之前，主端一直带着增益顶着自己的爪子**，所以操作者无法在一个还在移动的从端
  下面把目标推走。等待由 `ready_timeout_s`（默认 `10.0`）兜底；`require_ready=False` 恢复
  立即交还。
- **可选的力矩护栏会放掉压得太狠的从端。** 设了 `torque_limit_nm` 之后，超过限值 60 ms 它就
  **就地**把刚度和阻尼降到零，等主端重新张开后重新武装。默认是 `0` —— **不显式要求就不启用**。
  这个值要按机器挑：它取决于爪子之间那个工件有多脆。
- **跟丢主端的从端保持原位，不会卸力。** 超过 `watchdog_s`（默认 `0.2`）没有新帧，`stale`
  置真、爪子停在原处 —— 它仍然夹得住中间的东西。
- **从端把主端的速度前馈进去**（对相邻两帧做差分），所以不会拖在移动的主端后面。`dq_max=0`
  可以关掉。
- **非有限值帧直接丢弃，绝不夹取**，两端都是。
- `teleop_status()` 报告 `active`、`mode`、`frames`、`last_frame_age_ms`、`stale`、
  `openness`、`position_mm`、`force_n`、`loop_hz`、`rejected`、`send_failed`、`fault`、
  `torque_nm`、`over_torque`、`torque_trips`、`ready` 等。`position_mm` 和 `force_n` 是
  主端自己的状态，或从端所跟随那一帧带来的主端值 —— 所以调用方不必再开一个 CAN 读取器就能
  显示爪子状态。

## 教一遍，再放一遍

用手教过一遍的动作可以录下来，之后重复。录制会把电机切成零重力，让你把爪子推完整个动作；
回放把录到的张开度当指令帧发出去。存的是归一化到 `[0, 1]` 的张开度，和遥操线上传的完全一样，
所以在一台夹爪上教的动作，可以在装法或标定不同的另一台上回放。

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:
    gripper.load_calibration()
    gripper.enable()

    taught = gripper.record(5.0)   # 手把手教 5 秒；这段时间爪子是松的
    taught.save("pick")            # ~/.litegrip/trajectories/pick.lgt
    gripper.play(taught)           # 重复一遍
```

| 方法 | 行为 |
| --- | --- |
| `record(duration_s, rate_hz=100.0, zero_gravity=True)` | 阻塞式手把手教。返回 `Trajectory`。 |
| `record_start(rate_hz=100.0, zero_gravity=True, max_samples=None)` | 后台录制；返回状态快照。 |
| `record_stop(allow_empty=False)` | 停止并返回录到的 `Trajectory`。 |
| `play(trajectory, speed=1.0, kp=None, kd=None, align=True)` | 阻塞式回放。`loop` 必须是 `False`。 |
| `play_start(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` | 后台回放。 |
| `play_stop(timeout=2.0)` | 停止回放，爪子保持在当前位置。 |
| `trajectory_status()` | 两个方向共用的一份快照：`active`、`kind`、`samples`、`error`，录制时还有 `rate_hz` / `loop_hz`，回放时还有 `frames` / `speed` / `openness` / `completed`。 |

```bash
python3 examples/trajectory.py --record 5 --save pick   # 手把手教，然后存起来
python3 examples/trajectory.py --list                   # 不需要硬件
python3 examples/trajectory.py --play pick --repeat 3
```

- **回放发的是位置，不是力。** 录到的力矩只为诊断而存，从不前馈，所以对着工件录下的「捏」
  回放出来只是一条位置轨迹。夹持力重要的话，在回放之后接一次 `grasp(force_n=...)`。
- **`record()` 是独占的，而且整个过程爪子都是松的。** 手别离开夹爪：没有任何东西托着它。
- **没录满的采集会抛错**，不会把一段残缺的录制当完整的返回。
- **由别的东西驱动时，用非零重力录制。** `record_start(zero_gravity=False)` 只**读**状态，
  所以调用方可以从另一个线程跑 `grasp()` 或一串运动，把它录下来。这是录程序化动作的路子。
- `LITEGRIP_TRAJ_DIR` 可以改轨迹目录。录制、回放和遥操三者互斥；`disconnect()` 会停掉正在
  跑的那个。

## 在它上面做产品

- **稳定的接口面是 `LiteGrip` 和 `gripper.actions`。** 高层方法都转发到一个 `GripperActions`
  实例，那一层就是写别的驱动（ROS 2 节点、服务、状态机）的落点。再往上都不是必需的。
- **CAN I/O 同时只能有一个主人。** 遥操、录制和回放互斥，会抛 `TeleopBusyError` /
  `TrajectoryBusyError`。你的进程必须保持响应时，用后台版本（`teleop_start`、`record_start`、
  `play_start`），再轮询对应的 `*_status()`。
- **检查 `ok`，需要诊断时再读字段。** `if gripper.close():` 能编译，含义就是 `ok`；它为假时，
  要记进日志的是 `final_cmd_rad`、`steps` 和 `state`。
- **异常是分类的，从不被吞掉。** `LiteGripError` 及其子类 —— `CommError`、`ConnectError`、
  `CommandError`、`CANTimeoutError`、`HardwareError`、`NotInitializedError` —— 外加
  `TeleopError` 和 `TrajectoryError`。`describe_error(code)` 把电机自己的错误码翻成文字。
- **崩溃安全是默认行为。** `disconnect()` 会失能电机，所以进程死掉不会留下一个还在出力的
  电机。只有在别的东西接得住这台夹爪时，才设 `disable_on_disconnect=False`。
- **存储位置是可预期的。** `~/.litegrip/<channel>_calibration.json` 和
  `~/.litegrip/trajectories/`。`LITEGRIP_CALIB` 和 `LITEGRIP_TRAJ_DIR` 可以改掉它们。标定
  文件要显式点名才会被读，见第 4 步。
- **配好 `logging` 就能看见 SDK 的判断。** 它走标准 `logging` 模块记录：加载了哪份标定、推出
  的方向、以及每一次触发的护栏。
- **引擎可以脱机测试。** `MotionConfig.sleep_fn` 和 `monotonic_fn` 是它仅有的时间接缝，测试
  套件就是靠这个在 CI 里跑假 CAN 的。

## 安全与限制

- **做任何运动前先加载标定。** 在那之前，每个运动方法都抛 `CommandError`。这就是全部的闸门：
  在占位默认值上，`goto(40.0)` 会映射到 `+0.61 rad`，越过真实的闭合限位，并且会压在那里。
- **`open`、`close`、`grasp` 和 `zero()` 期间，手和工件都要离开行程范围。** `open` 和 `close`
  停在标定限位上；`zero()` 带电探真实的机械限位。
- **标定错了只会缩短行程，不会让爪子冲出去。** `open` 和 `close` 把发出的指令夹在标定的极端
  位置上，所以爪子到了那里之后残余误差就是零，稳态压紧力矩也是零。
- **保住的力会一直压着。** 保力段没有任何位置约束：帧里没有增益，爪子是柔顺的，电机不管它们
  走到哪儿都用 `force_n` 顶着。失控监视是唯一能停住它的东西，而它是兜底，不是工作限位。
- **`write_zero()` 不可撤销。** 它把编码器偏移写进电机 flash。
- **持续压着会加热线圈。** `open` 和 `close` 每次都会在限位上压过整个稳定段，连续跑请留意
  线圈温度。
- **非有限值被拒绝，绝不被夹取。** `goto`、`goto_rad`、`move_at_speed*`、`grasp` 和
  `set_force` 遇到 NaN 或 ±inf 参数会抛 `CommandError`，CAN 边界在任何一个字段变成字节之前
  就会拒绝它。把 NaN 夹一下等于用行程的一端来回答它 —— 一条没人要求过的真实指令。

## 排错

| 现象 | 含义 |
| --- | --- |
| `CommandError` 提到标定 | 这个通道没加载标定。走第 4 步。 |
| `ConnectError`，或一直收不到状态帧 | `can0` 没起，或波特率不是 1 Mbit/s。见第 2 步。 |
| `enable()` 抛 `HardwareError` | 电机始终没报 `err == 1`。先查供电，再 `clear_fault()`。 |
| 爪子往错的方向动 | 这台是反装，不是正装。见第 3 步。 |
| `close()` 返回 `ok=False` | 爪子停的位置离限位超过 `stop_tol`。调大 `stop_lead_mm` 直到它压到位。 |
| `travel_guard` 置位，或 `set_force` 返回 `False` | 失控监视触发了：爪子越过了闭合侧限位。检查标定是否描述真实行程，以及限位有没有被拆掉。 |
| `can1` 的 `load_calibration()` 返回 `False` | 无参加载读的是出厂文件，而它声明的是 `channel: can0`，所以这个通道没有隐式来源。给它跑一次 `zero()`，再用 `path=` 或 `LITEGRIP_CALIB` 点名那份结果。 |
| 保住的力掉到设定值以下 | 你改了保力增益，或有东西以快于前馈的速度把爪子推回去。`hold_kp` / `hold_kd` 已废弃并被忽略。 |

## 示例

下面每个脚本都能从仓根直接跑，并会打印它量到的结果。

| 脚本 | 回答什么问题 |
| --- | --- |
| `examples/stroke_test.py` | 这台设备能用吗？走完行程两端，成功才退出 0。 |
| `examples/grasp_test.py` | `grasp()` 是否还说到做到 —— 空爪和夹着工件两种？ |
| `examples/hold_force_test.py` | 保住的力是保住了，还是漂到更小的值？ |
| `examples/zero_closed.py` | 台架工具：`status`、`release`、`goto`，以及不可撤销的 `zero`。 |
| `examples/teleop.py` | 主从链路中的一端。 |
| `examples/trajectory.py` | 手把手教一个动作，存起来，再放一遍。 |

## 从 2.2.0 升级

运动方法重写过，所以签名变了。返回值现在是 dataclass，不再是裸 `bool`。旧的位置参数调用会
抛 `TypeError`，不会悄悄地做另一件事。

- `grasp(force_n=10.0, kp=150.0, kd=2.0, duration=3.0, ...)` 变成
  `grasp(force_n=None, hold_s=0.0)`，默认力现在是 `20 N`。
- 增益从调用参数搬到了 `GripperConfig` / `MotionConfig` 上。
- `enable()` 现在返回 `EnableResult` 并如实报告失败 —— 过去只要状态帧里是 `0` **或** `1`
  它就返回 `True`，所以一台从没通电的电机也会显示成已使能。
- **`MoveResult.__bool__` 的含义变了。** 过去是 `reached and not stalled`，现在是 `ok`。
  对 `grasp` 的闭合段两者一致；对 `open` 和 `close` 则相反：成功压到限位是
  `stalled=True, reached=False`，所以 `if gripper.close():` 的含义和以前不同了，尽管类型
  一直没变。
- `open` 和 `close` 不再把指令发到标定限位之外，并且运动接口在未标定的配置上会拒绝运行。
- 标定**按通道**分开：`~/.litegrip/<channel>_calibration.json`，不再是单一文件
  `litegrip_calibration.json`。旧文件仍能加载，但必须点名：无参 `load_calibration()` 只读
  出厂标定。`LITEGRIP_CALIB` 仍然覆盖它。

## 开发

测试套件不依赖硬件：它通过 `tests/fake_can.py` 里一个运动学假 CAN 驱动引擎，所以没有夹爪、
没有 CAN 接口也能到处跑。CI 会 lint 两份 README 并跑这套测试：

```bash
python3 -m unittest discover -s tests -t tests -v
```

`litegrip.__version__` 从已安装分发包的元数据读取，所以装好的 wheel 报的是真正发布出去的
版本。从没 `pip install` 过的源码树会报 `0.0.0+source`；这是预期行为，不是构建坏了。

仓库遵循 Conventional Commits，并在每次合入 `main` 时由 semantic-release 自动发版；贡献者
需要遵守的规则见 [AGENTS.md](AGENTS.md)。

## 相关仓库

| 仓库 | 角色 |
| --- | --- |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 驱动 |
| [litegrip-ros1](https://github.com/nexform-tech/litegrip-ros1) | ROS 1 驱动 |

## 许可证

Copyright © 2026 NEXFORM ROBOTICS. 以 [Apache License 2.0](LICENSE) 授权。
