# litegrip-python

[English](README.md) | [简体中文](readme_zn.md)

**LiteGrip 轻量机械爪系列**的 Python SDK，也是它主要的公开接口。本文是一份接口参考：
每个接口是什么，以及怎么用。

你不需要懂 CAN，也不需要懂 DM4310 的协议：每一次运动、标定和力控都是一个方法调用，
每个调用在返回前都会自己核对结果。下面每个签名都是真签名，默认值也是真默认值。

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

## 3. 快速上手

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:            # 进入时 connect()，退出时 disconnect()
    gripper.load_calibration()               # 随包发布的出厂标定 —— 不带参数
    gripper.enable()                         # 重试到状态帧报 err == 1 为止

    gripper.open()                           # 以 50 mm/s 走向张开侧限位
    gripper.close()                          # 以 50 mm/s 走向闭合侧限位
    result = gripper.grasp(force_n=20.0, hold_s=3.0)
    print(result.ok, result.stalled, result.cycles)
```

往下看之前，先知道两件事：

- **出厂标定就是默认值。** 不带参数的 `load_calibration()` 读的是 SDK 里随包发布的那份标定，
  它描述的就是出厂的机器（正装）。只有你手上另有一份标定时，才需要指定 —— 见[标定](#标定)。
- **没加载标定之前，什么都不会动。** 未标定的配置上，每个运动方法都抛 `CommandError`，
  因为占位限位描述的行程不是任何真实行程。

`with` 退出时会调 `disconnect()`，默认会把电机失能。想让退出后保持上电，传
`LiteGrip(..., disable_on_disconnect=False)`。

## 接口

### LiteGrip

```python
LiteGrip(channel="can0", can_id=0x08, mst_id=None, canfd_mode=None,
         motor_type=MotorType.DM4310, config=None, motion_config=None,
         disable_on_disconnect=True, mount=None)
```

- `channel` —— CAN 接口名。每台 LiteGrip 都共用 CAN ID `0x08`，所以一台机器上装两台时，
  接口名是唯一能区分它们的键。
- `mount` —— `"normal"` 或 `"reverse"`，装载对应模板的限位。出厂的都是正装，出厂标定描述的
  也是正装，所以**不要传**，除非有台机器是反装的（你对它调 `close()`，爪子反而张开）。
- `mst_id` —— 主站 ID；`None` 自动探测。
- `config` / `motion_config` —— 构造时就整份替换这两个 dataclass，而不是构造完再改字段。
- `disable_on_disconnect` —— `disconnect()` 是否失能电机（崩溃安全）。只有当**别的东西**在
  接着控制夹爪时才设 `False`。

| 属性 | 是什么 |
| --- | --- |
| `channel`、`can_id`、`mst_id` | 这个实例驱动的身份 |
| `config` | `GripperConfig` —— 限位、比例、增益 |
| `motion_config` | `MotionConfig` —— 全部可调项；可整份替换，也可逐字段改 |
| `actions` | `GripperActions` —— 高层方法转发到的引擎 |
| `mount` | `"normal"`、`"reverse"`，未标定时是 `None`（由限位数值推出） |
| `is_connected`、`is_enabled` | 会话状态 |
| `session` | CAN 会话，进阶用 |

### 连接与状态

| 方法 | 做什么 |
| --- | --- |
| `connect()` → `bool` | 打开 CAN 会话。`with` 块已经替你做了。 |
| `disconnect()` → `None` | 关闭会话；除非 `disable_on_disconnect=False`，否则会失能电机。 |
| `poll(timeout_s=0.0)` → `bool` | 读一帧；读到返回 `True`。 |
| `get_state(wait=True)` → `GripperState` | 位置、速度、力矩、温度、错误位。`wait=True` 阻塞等一帧新数据。 |
| `get_position()` / `get_position_rad()` → `float` | 实测开口，单位 mm / rad。 |
| `get_force()` → `float` | `get_torque()` × 10，单位 N。 |
| `get_torque()` → `float` | 实测力矩，Nm。 |
| `get_temperature()` → `tuple[int, int]` | `(MOS 温度, 线圈温度)`。 |
| `get_error()` → `int` | 电机上报的状态码：`1` 已使能、`0` 已失能、`0x9`–`0xC` 是故障。`describe_error(code)` 给出文字。 |
| `get_info()` → `GripperInfo` | 静态设备信息。 |
| `is_moving()` / `is_grasped()` → `bool` | 运动与夹持判据。 |
| `wait_for_ready(timeout=5.0)` → `bool` | 阻塞到电机可以接受指令为止。 |

### 使能与故障

| 方法 | 做什么 |
| --- | --- |
| `enable(retries=None)` → `EnableResult` | 使能，然后回读状态帧，直到它报 `err == 1`。`retries=None` 用 `MotionConfig.enable_retries`（3）。 |
| `disable()` → `bool` | 力矩归零，可以用手掰动爪子。 |
| `clear_fault()` → `bool` | 清除锁存的电机故障。 |
| `stop()` → `None` | 发一帧零力矩。 |

### 运动

| 方法 | 做什么 |
| --- | --- |
| `open(speed_mm_s=None, *, progress=None)` → `MoveResult` | 爬坡走向标定的张开侧限位。若 `GripperConfig.work_stroke_mm` 有值，则停在那段行程处 —— 出厂标定设的是 `80.0`，对着这台机器 85 mm 的行程，所以裸调 `open()` 会在距张开端 5 mm 处停下。 |
| `close(speed_mm_s=None, *, progress=None)` → `MoveResult` | 一样，方向朝闭合侧。`work_stroke_mm` 是张开侧的数字，不改变 `close()` 的落点。 |
| `home()` → `MoveResult` | 爬坡回到标定的闭合位置。 |
| `grasp(force_n=None, hold_s=0.0, *, progress=None)` → `GraspResult` | 闭到堵转（即夹住），再爬到 `force_n` N 并保持。闭合段就把 `force_n` 当力矩预算，所以顶到工件时压得不会比设定值更狠。`hold_s=0` 表示保持到故障、失控监视触发或 Ctrl+C。`force_n=None` 用 `MotionConfig.force_n`（20 N）。 |
| `set_force(force_n, duration=0.3)` → `bool` | 在爪子**当前**位置保持 `force_n`，持续 `duration` 秒。 |
| `goto(position_mm, kp=None, kd=None, duration=0.5)` → `MoveResult` | 爬坡走到毫米目标。 |
| `goto_rad(position_rad, kp=None, kd=None, dq_target=0.0, tau_feedforward=0.0, duration=0.5)` → `MoveResult` | 一样的，单位 rad。`dq_target` 与 `tau_feedforward` 必须是 0：速度前馈由斜坡按自己的排程给出，所以这两个值一律拒绝并抛 `CommandError`，不是静默忽略。 |
| `move_to(target_rad, kp=None, kd=None, tau_feedforward=0.0, duration=1.0)` → `MoveResult` | 和 `goto_rad` 一样，只是 `duration` 的默认值更长。 |
| `move_at_speed(target_mm, speed_mm_s=30.0, kp=None, kd=None)` → `MoveResult` | 按 `speed_mm_s` 爬坡走到毫米目标。 |
| `move_at_speed_rad(target_rad, speed_rad_s=0.5, kp=None, kd=None)` → `MoveResult` | 一样的，单位 rad。 |

凡是收 `kp` / `kd` 的调用，都回落到 `GripperConfig.kp`（`100.0`）和 `.kd`（`2.0`）。
所有定位调用 —— `goto` 一族与 `move_at_speed` 一族都是 —— 走的都是同一条斜坡，每
`MotionConfig.frame_interval` 一帧，而 **`duration` 就是这条斜坡走多久**，不是截止时间：
40 mm 的目标配 `duration=0.5` 就是 80 mm/s、走 0.5 s，之后原地保目标 `settle_s`（`0.3`）秒
用于判到位。两族之间的差别只有速度怎么给（给时长、给 mm/s，还是给 rad/s）。

这条斜坡**有上限**，上限正是它的意义所在：每一帧的指令相对实测位置的领先不超过
`MotionConfig.max_lead_mm`（4 mm），于是这一帧的力矩被压在 `kp × 4 mm` 附近 —— 按出厂标定
`rad_to_mm = 61.01` 算约 6.6 N·m，在 DM4310 的 ±10 N·m 帧量程之内。一步跳到目标则没有这个
上界：出厂 `kp` 是 `100 Nm/rad`，误差一过约 `0.1 rad` 就把帧量程顶到饱和，而饱和的那一帧会
冲过目标，不是停在目标上。`open` 和 `close` 是**故意**压上标定限位的：它们把发出的指令夹在
标定极值上，所以爪子停在限位处，永远不会被驱过限位。NaN 或 ±inf 位置一律拒绝并抛
`CommandError`，不会被夹取。

### 夹持与力控

`grasp(force_n, hold_s)` 先闭再保；`set_force(force_n, duration)` 在爪子**当前**位置保持。
两者发的帧都是 `kp = kd = 0`，所以驱动器输出**就是**前馈力矩 `force_n × 0.1 Nm`。
正因如此，这个力与工件无关：软工件让位时爪子跟着走，夹持力仍停在设定值上。

- **这个力是近似的。** `1 N = 0.1 Nm` 在 SDK 里就标着「近似」，真实夹持力还取决于你指爪的
  几何。把 `force_n` 当成一个**可重复**的档位，不要当成标定过的测量值。
- **爪子是柔顺的。** 外力可以把它们推回去，而电机仍用 `force_n` 顶着；空爪 `grasp()` 就是用
  这个力顶到机械限位上。这就是「工件移动时力不漂」的代价。
- **小于约 3.7 N 时，接近速度会变成限制项**，因为 `grasp_speed_mm_s` 下走一帧就已经花光了
  整个力矩预算。要限制的是接近时间、改 `grasp_speed_mm_s`，不要去要求一个机构自身摩擦就能
  吃掉的力。

**失控监视。** 保力帧不带位置误差，所以任何位置限位都管不到它 —— 唯一可用的手段是撤掉力矩。
一旦实测位置越过闭合侧标定限位 `hold_runaway_rad`（`0.05 rad`，约 3 mm），`grasp()` 和
`set_force` 就连发零力矩释放帧并返回 —— `GraspResult.travel_guard` 置位，或 `set_force`
返回 `False`。只判闭合侧，因为前馈力矩就是朝那边压的。这是**失控检测，不是精度上限**：
只有当**确实夹着东西**的爪子被释放时，才去调大 `hold_runaway_rad`。未标定时监视不启用。

需要比默认更大的力？`MotionConfig.force_n` 是 `20.0` N（`2.0 Nm`），在 DM4310 的 `3 Nm`
额定之内。它的峰值是 `7 Nm`，协议上限是 `10 Nm`。

### 标定

SDK 里随包发布的出厂标定就是标准值，不带参数的加载只读它：

```python
from litegrip import LiteGrip

with LiteGrip("can0") as gripper:
    gripper.load_calibration()      # 随包发布的那份，描述一台正装的机器
    gripper.enable()
```

| 调用 | 读什么、写什么 |
| --- | --- |
| `load_calibration()` → `bool` | 随包发布的出厂标定；设了 `LITEGRIP_CALIB` 时再加上它（环境变量优先）。 |
| `load_calibration(path="/abs/file.json")` → `bool` | 读那个文件。只有当它给不出可用标定时才回落到出厂文件，通道对不上会告警。 |
| `load_calibration(template="normal")` / `load_template(name)` → `bool` | 那个模板的限位，不管在哪个通道上。严格：名字不认识就抛 `CommandError`。 |
| `list_templates()` → `List[str]` | `['normal', 'reverse']` —— SDK 接受的全部名字，给 UI 做下拉框用。这是模块级函数，要按名字导入。 |
| `default_calib_path(channel="can0")` → `str` | 存标定的落点：`~/.litegrip/<channel>_calibration.json`，设了 `$LITEGRIP_CALIB` 时是它。 |
| `save_calibration(path=None)` → `str` | 把当前配置写到这个路径。 |
| `zero()` → `CalibrationData` | 探两端机械限位，推出比例，存盘。 |
| `calibrate_guided(kp=20.0, kd=2.0, step_rad=0.05, ...)` → `CalibrationData` | 引导式两步探测，每到一端停下来等你确认。 |
| `calibrate_manual(duration=30.0, settle_time=2.0, sample_interval=0.01)` → `CalibrationData` | 手把手教：先卸力，你用手把爪子推完整个行程，它取两端的极值。 |
| `write_zero()` → `WriteZeroResult` | 把**当前**编码器角度就地声明成电机零点。会写电机的 flash，**无法撤销**。 |

由此推出几条：

- **你自己机器上量出来的标定，不会被顺手读回来。** `zero()` 写到
  `~/.litegrip/<channel>_calibration.json`，而那个文件只有你**点名**时才会读：
  `load_calibration(path=default_calib_path("can0"))`，或者用
  `LITEGRIP_CALIB=/path/to/file.json` 一次覆盖所有通道。**不要指望它被自动加载。**
- **那个目录里只有 `<channel>_calibration.json` 被认。** 换个名字的标定文件 —— 比如
  `can0_08_calibration.json` —— 连候选都不是：不会被尝试、不会被合并、也不是更靠后的回落
  阶段。每当不带参数的加载从一份本机文件旁边走过，告警都会点名它，并重复这两种用法。
- `zero()` 会把爪子驱到两端真实限位上，所以别把手和工件留在行程里；如果装法会让爪子往下坠，
  托住它。指爪不是 85 mm 时改 `GripperConfig.max_stroke_mm`（`85.0`，出厂硬件）：这是唯一一项
  没有写进任何标定文件的单机尺寸，而且只在你重新探测时才有意义。
- 出厂文件声明的是 `channel: can0`，所以 `can1` 上的机器没有隐式来源，它会直接失败 ——
  `load_calibration()` 返回 `False` —— 而不是悄悄沿用 `can0` 的方向。给它一个模板，或者
  按上面的说法把它自己那份 `zero()` 点名回来。

### 怎么读返回值

返回类型是 dataclass，同时定义了 `__bool__`，所以 `if gripper.close():` 可以用。但它**含义**
随动作而异，要看的字段也不同：

| 类型 | 字段 | `bool()` 是 |
| --- | --- | --- |
| `MoveResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`limit_rad`、`final_cmd_rad`、`steps`、`protected`、`released` | `ok` |
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

在离限位很远的地方堵转，才会给 `ok=False`：有东西挡住了行程。为假时该记的，就是那几个诊断
字段 —— `final_cmd_rad`、`steps`、`state`。

定位调用的读法不同，因为它的目标是行程内的**一点**，不是一端的限位：`goto`、`goto_rad`、
`move_to`、`home` 和 `move_at_speed*` 只有在**走到了且没堵转**（`reached and not stalled`）
时才算 `ok`。半路被挡住时返回 `ok=False, stalled=True`，并且 `released=True` —— 意思是堵转
之后把驱动器的力**撤掉**了（`kp = kd = tau = 0`），不是留在那儿一直压着工件。
`protected=True` 是移动提前结束的另一条路：行进段那条力矩保护（约 7 N，见
`MotionConfig.stop_torque_nm`）触发了。那条保护自己就会撤力，所以 `protected` 时
`released` 仍是 `False`，而它一定让 `ok=False`。

### 进度回传

运动接口从不打印，它们通过 `progress` 回调回传：按采样频率收到一个 `MoveProgress` 快照。

```python
def show(p):
    print(f"[{p.phase}] {p.i}/{p.total_steps} cmd={p.cmd_rad:+.4f} "
          f"pos={p.pos_rad:+.4f} tau={p.torque_nm:+.3f}")

gripper.open(progress=show)
```

`grasp` 保力时 `p.phase` 是 `"hold"`，`p.total_steps` 是 `0`。`zero()` 是唯一的例外：
它自己往 stdout 打印探测进度。

### 调参

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
`sleep_fn=lambda _: None` 能把整段爬坡压成不占墙上时间。

### 主从遥操

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

| 方法 | 做什么 |
| --- | --- |
| `teleop_start(mode, *, transport=None, link="zenoh", host=None, port=17448, grip_id="gripA", kp=None, kd=None, align=True, watchdog_s=0.2, dq_max=10.0, torque_limit_nm=0.0, require_ready=True, ...)` → `dict` | 为 `mode="master"` 或 `"slave"` 起后台循环。两端必须共用 `grip_id`，并且都已连接、已使能。 |
| `teleop_stop(timeout=2.0)` → `dict` | 停掉循环，爪子保持在当前位置。 |
| `teleop_status()` → `dict` | `active`、`mode`、`frames`、`last_frame_age_ms`、`stale`、`openness`、`position_mm`、`force_n`、`loop_hz`、`rejected`、`send_failed`、`fault`、`torque_nm`、`over_torque`、`torque_trips`、`ready` 等。 |

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

- **从端报告就位之前，主端一直带着增益顶着自己的爪子**，所以操作者无法在一个还在移动的从端
  下面把目标推走。等待由 `ready_timeout_s`（默认 `10.0`）兜底；`require_ready=False` 恢复
  立即交还。
- **可选的力矩护栏会放掉压得太狠的从端。** 设了 `torque_limit_nm` 之后，超过限值 60 ms 它就
  **就地**把刚度和阻尼降到零，等主端重新张开后重新武装。默认是 `0` —— **不显式要求就不启用**。
  这个值要按机器挑：它取决于爪子之间那个工件有多脆。
- **跟丢主端的从端保持原位，不会卸力。** 超过 `watchdog_s`（默认 `0.2`）没有新帧，`stale`
  置真、爪子停在原处。
- **从端把主端的速度前馈进去**（对相邻两帧做差分），所以不会拖在移动的主端后面。`dq_max=0`
  可以关掉。
- **非有限值帧直接丢弃，绝不夹取**，两端都是。
- 遥操运行期间独占 CAN I/O：在 `teleop_stop()` 之前不要再从调用方驱动夹爪。

### 录制与回放

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
| `record(duration_s, rate_hz=100.0, zero_gravity=True)` → `Trajectory` | 阻塞式手把手教。 |
| `record_start(rate_hz=100.0, zero_gravity=True, max_samples=None)` → `dict` | 后台录制。 |
| `record_stop(allow_empty=False)` → `Trajectory` | 停止并返回录到的东西。 |
| `play(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` → `dict` | 阻塞式回放。`loop=True` 抛 `ValueError` —— 阻塞式回放永远回不来。 |
| `play_start(trajectory, speed=1.0, kp=None, kd=None, loop=False, align=True)` → `dict` | 后台回放。 |
| `play_stop(timeout=2.0)` → `dict` | 停止回放，爪子保持在当前位置。 |
| `trajectory_status()` → `dict` | 两个方向共用的一份快照：`active`、`kind`、`samples`、`error`，录制时还有 `rate_hz` / `loop_hz`，回放时还有 `frames` / `speed` / `openness` / `completed`。 |

```bash
python3 examples/trajectory.py --record 5 --save pick   # 手把手教，然后存起来
python3 examples/trajectory.py --list                   # 不需要硬件
python3 examples/trajectory.py --play pick --repeat 3
```

- **回放发的是位置，不是力。** 录到的力矩只为诊断而存，从不前馈，所以对着工件录下的「捏」
  回放出来只是一条位置轨迹。夹持力重要的话，在回放之后接一次 `grasp(force_n=...)`。
- **`record()` 是独占的，而且整个过程爪子都是松的。** 手别离开夹爪：没有任何东西托着它，
  爪子中间的东西会掉。
- **没录满的采集会抛错**，不会把一段残缺的录制当完整的返回。
- **由别的东西驱动时，用非零重力录制。** `record_start(zero_gravity=False)` 只**读**状态，
  所以调用方可以从另一个线程跑 `grasp()` 或一串运动，把它录下来。
- `LITEGRIP_TRAJ_DIR` 可以改轨迹目录。录制、回放和遥操三者互斥，会抛
  `TrajectoryBusyError` / `TeleopBusyError`；`disconnect()` 会停掉正在跑的那个。

### 异常

| 异常 | 什么时候抛 |
| --- | --- |
| `LiteGripError` | 基类。有错误码时，`str()` 会把它附成 `[0xXXXX]`。 |
| `ConnectError` | CAN 接口不可用，或电机始终不回复。 |
| `CommError` | 会话中途 CAN 读写失败。 |
| `CANTimeoutError` | 超时内没有回复。 |
| `CommandError` | 指令被拒：参数不对、配置未标定、模板不认识，或者值是 NaN / ±inf。 |
| `HardwareError` | 电机报了故障码 —— 比如 `enable()` 始终没读到 `err == 1`。 |
| `NotInitializedError` | 没连接，或者没使能。 |
| `TeleopError` / `TrajectoryError` | 遥操与录制 / 回放；各自带 busy、not-active、format 等子类。 |

异常是分类型的，而且从不被吞掉。`describe_error(code)` 把电机自己的错误码变成文字。SDK 通过
标准 `logging` 模块记录日志 —— 加载了哪份标定、推出了什么方向、哪条护栏触发了；给 `litegrip`
logger 挂个 handler 就能看到，注意即使不挂 handler，告警也会到 stderr。

### 进阶接口

| 接口 | 用来做什么 |
| --- | --- |
| `gripper.actions` | 高层方法背后的 `GripperActions` 引擎 —— 想在它上面再写一层驱动就从这开始。 |
| `send_mit_frame(q, kp, kd, dq=0.0, tau=0.0)` | 发一帧裸 MIT，想自己组控制律时用。 |
| `read_param(rid, timeout_s=0.5)` | 读一个电机寄存器。 |
| `enter_zero_gravity(duration=0.0)` / `exit_zero_gravity()` | 先把电机卸力用手掰，再收回来。 |
| `clamp_to_travel(config, q)` | 把一个角度夹进标定行程 —— `open` 和 `close` 发出的指令遵循的就是这条规则。 |
| `limit_target`、`press_target`、`work_limit_target` | `open`、`close` 与工作行程限位背后的目标计算。 |
| `force_approach_terms(kp, kd, speed_rad_s, frame_interval, budget_nm, lead_ceiling_rad)` | 带力的接近段能花掉的速度、阻尼和前导量。 |
| `closing_travel_excursion_rad(config, q)` | 一个位置越过闭合侧限位多少。 |
| `CALIB_TEMPLATES`、`UnitConversion`、`GripperParams`、`GripperGeometry`、`ErrorCode` | 模板路径、单位换算、电机限值、几何与告警码。 |
| `litegrip.can` | 裸 SocketCAN 传输与 DM 协议编解码，多电机场合用。 |

`clamp_to_travel`、`limit_target`、`press_target`、`work_limit_target`、`force_approach_terms`
和 `closing_travel_excursion_rad` 是模块级函数 —— 从 `litegrip` 导入。它们把配置作为第一个
参数收进来，而不是挂在夹爪上调用。

## 排错

| 现象 | 说明 |
| --- | --- |
| `CommandError` 里提到标定 | 这个通道还没加载标定。调 `load_calibration()`。 |
| `ConnectError`，或者始终没有状态帧 | `can0` 没起来，或者波特率不是 1 Mbit/s。见第 2 节。 |
| `enable()` 抛 `HardwareError` | 电机始终没报 `err == 1`。先查供电，再 `clear_fault()`。 |
| 爪子朝反方向动 | 这台机器是反装，不是正装。构造时传 `mount="reverse"`。 |
| `close()` 返回 `ok=False` | 爪子停的位置离限位超过了 `stop_tol`。这个缺口来自 `zero()` 探测时压得比 `close()` 更狠，所以调 `stop_lead_mm` 基本看不出变化 —— 先核对标定描述的是不是真实行程，如果这台确实停在限位上，再调大 `stop_tol`。 |
| `travel_guard` 置位，或 `set_force` 返回 `False` | 失控监视触发了：爪子走到了闭合侧限位之外。核对标定描述的是不是真实行程，以及限位还在不在。 |
| `can1` 上的机器 `load_calibration()` 返回 `False` | 不带参数的加载读的是出厂文件，而它声明的是 `channel: can0`，所以这台没有隐式来源。给它一个模板，或者用 `path=` / `LITEGRIP_CALIB` 点名它自己那份文件。 |
| 保住的力比设定值低 | 你改过保力增益，或者有东西把爪子推得比前馈还快。`hold_kp` / `hold_kd` 已弃用，会被忽略。 |
| `import litegrip` 拿到的代码比你在读的源码树旧 | 装过的副本不会跟着 `git pull` 自己更新。先看 `litegrip.__file__`，或者在代码仓里 `pip install -e .`。 |

## 安全与限制

- **任何运动之前先加载标定。** 在那之前每个运动方法都抛 `CommandError`。这就是全部的闸门：
  在占位默认值上，`goto(40.0)` 会映射到 `+0.61 rad`，越过了真实的闭合限位，然后就在那里压。
- **`open`、`close`、`grasp` 和 `zero()` 期间，别把手和工件留在行程里。** `open` 和 `close`
  停在标定限位上；`zero()` 是带电去探真实机械限位。
- **标定错了只会限制行程，不会让爪子越过它。** `open` 和 `close` 把发出的指令夹在标定极值上，
  所以爪子到了那里之后残差为零，持续压紧的力矩也是零。
- **保住的力会一直压。** 保力不受位置约束：那些帧不带增益，爪子是柔顺的，电机在它们最终停下
  的位置用 `force_n` 顶着。唯一能停它的是失控监视，而那是兜底，不是工作限值。
- **`write_zero()` 不可逆。** 它把编码器偏移写进电机的 flash。
- **持续压紧会让线圈发热。** `open` 和 `close` 每次都会在限位上压过整段稳定期，连着跑就查一下
  线圈温度。
- **非有限值一律拒绝，绝不夹取。** `goto`、`goto_rad`、`move_at_speed*`、`grasp` 和
  `set_force` 遇到 NaN 或 ±inf 参数就抛 `CommandError`，而 CAN 边界在它变成字节之前就会拒收
  帧里的这类字段。
- **`disconnect()` 默认会失能电机**，所以进程崩掉不会留下一个还吃着力矩的电机。只有当别的
  东西在接着控制夹爪时，才设 `disable_on_disconnect=False`。

## 示例

下面每个脚本都能从代码仓根目录直接跑，并且会打印它量到的东西。

| 脚本 | 回答什么 |
| --- | --- |
| `examples/stroke_test.py` | 这台机器能不能用？走完行程两端，成功时退出码 0。 |
| `examples/grasp_test.py` | `grasp()` 说的和做的还一致吗 —— 空爪和夹着东西各一遍？ |
| `examples/hold_force_test.py` | 保住的力是稳住了，还是漂到了更小的值？ |
| `examples/zero_closed.py` | 低层 CAN 台架工具，一个子命令一个操作：`status`、`release`、`goto`、`zero`、`open`、`close`、`enable`、`disable`。 |
| `examples/teleop.py` | 主从链路里的其中一端。 |
| `examples/trajectory.py` | 手把手教一个动作，存起来，再放一遍。 |

测试套件不依赖硬件：它让引擎跑在 `tests/fake_can.py` 里的运动学假 CAN 上，所以在任何没有
夹爪、没有 CAN 接口的地方都能跑。

```bash
python3 -m unittest discover -s tests -t tests -v
```

## 许可证

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the [Apache License 2.0](LICENSE).
