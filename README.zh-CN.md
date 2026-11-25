# litegrip-python

[English](https://github.com/nexform-tech/litegrip-python/blob/main/README.md) | [简体中文](https://github.com/nexform-tech/litegrip-python/blob/main/README.zh-CN.md)

**LiteGrip 轻量机械爪系列**的 Python SDK，也是它主要的公开接口。整个设备由
一个类 `Gripper` 暴露：使能/失能、张开/闭合、绝对定位、自适应力控夹取、标定和
带标定的反馈。

你不需要了解 CAN 或 DM4310 协议。每个动作都是一次阻塞式方法调用，调用自己在
返回成功前核对结果。**动作跑过但没到位时返回 `False` 并记录原因，绝不静默报成功。**

## 环境要求

| | |
| --- | --- |
| 电机 | 达妙 DM4310，CAN ID `0x08`，主站 ID `0x18`，MIT 模式，1 Mbit/s 经典 CAN |
| 主机 | Linux + SocketCAN（唯一支持平台） |
| Python | 3.10 或更新 |
| 依赖 | 无：只用标准库 |

## 安装

```bash
pip install litegrip-python
```

或从仓库安装：

```bash
git clone https://github.com/nexform-tech/litegrip-python.git
cd litegrip-python
python3 -m pip install .
```

也可以不安装，直接用源码树：

```bash
PYTHONPATH=/path/to/litegrip-python/src python3 your_script.py
```

示例脚本会自己把仓库的 `src/` 插进 `sys.path`，所以在裸解释器上
`python3 examples/gripper_example.py --help` 就能跑。

## 拉起 CAN 接口

夹爪是 CAN 设备。任何 SDK 调用之前先把接口按 1 Mbit/s 拉起，每次重启后都要
重做：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

两条命令都需要 root。如果 `can0` 根本不存在，是 CAN 适配器的驱动还没加载。产品
里应由 init 系统在开机时执行，而不是让应用自己拉起接口。

传输层请求经典 CAN；若接口报告 FD MTU 会自动升级。CAN FD 不属于公开配置，DM4310
的帧格式没有变化。

## 快速上手

```python
from litegrip import Gripper

with Gripper("can0") as gripper:          # 进入时 connect()，退出时 disconnect()
    gripper.enable()                     # 重试到状态帧回读 err == 1
    gripper.open()                       # 朝张开侧，默认 50 mm/s
    print(gripper.get_gripper_distance()) # 从闭合零点算起的 mm
    gripper.set_gripper_distance(40.0)   # 绝对开口 40 mm
    gripper.grasp(force_n=20.0, hold_s=3.0)
    print(gripper.get_motor_data())      # {"Position", "Speed", "Torque"}
```

`with` 退出时调用 `disconnect()`：先失能（零力矩）再关闭总线。

## 接口

### 构造函数

```python
Gripper(channel="can0", can_id=0x08, mst_id=0x18, motion_config=None)
```

| 参数 | 含义 |
| --- | --- |
| `channel` | SocketCAN 接口名。 |
| `can_id` | 电机 CAN ID。 |
| `mst_id` | 状态帧主站 ID。没有自动探测；默认比 `can_id` 大 0x10。 |
| `motion_config` | 可选 `MotionConfig`，调整速度、帧间隔、力矩预算等。 |

构造函数会**自动加载标定**：设置了 `$LITEGRIP_CALIB` 就用它；否则若存在
`~/.litegrip/<channel>_calibration.json` 就用它；再否则用随包的
`factory_calibration.json`。出厂文件带 `calibrated=true`，所以开箱即可用距离指令。
之后显式调用 `load_calibration(path)` 会覆盖这个选择。

诊断属性：`channel`、`can_id`、`mst_id`、`is_connected`、`is_enabled`、`config`、
`motion_config`，以及 `last_send_error`（上一帧没发出去的原因）。

### 生命周期

| 方法 | 返回 | 作用 |
| --- | --- | --- |
| `connect()` | `bool` | 打开 SocketCAN 并注册唯一的电机。重复调用返回 `True`。失败抛 `ConnectError` / `CommError`。 |
| `disconnect()` | `None` | 先失能，再关闭总线。幂等且不抛异常。 |
| `enable(retries=None)` | `bool` | 完整初始化：失能 → MIT 模式 → 使能 → 保持，并用一帧新状态确认 `err == 1`。失败重试，真故障先清。 |
| `disable()` | `bool` | 失能，夹爪可被手动推动。 |
| `clear_fault()` | `bool` | 清除锁存的欠压/过流/过温故障并重新使能；清不掉抛 `HardwareError`。 |
| `set_zero()` | `bool` | 通过 0xFE 把**当前编码器角度**记为零：先失能、再发、后重使能，并要求回读在 1e-3 rad 内。**不可逆**，会写电机 flash。 |
| `calibrate(save=True)` | `CalibrationData` | 低刚度探测两端机械限位并更新 `config`；`save=True` 写入本通道默认路径。阻塞；夹爪会动。 |
| `load_calibration(path=None)` | `bool` | 给了 `path` 就只读它，不回退；不给时依次找 `$LITEGRIP_CALIB`、本通道文件、出厂文件。全部读不到返回 `False`。 |
| `save_calibration(path=None)` | `str` | 保存当前标定，默认 `~/.litegrip/<channel>_calibration.json`，返回路径。 |
| `default_calibration_path(channel="can0")` | `str` | 模块级函数，返回该默认路径。 |

### 运动

所有运动都阻塞，并要求已连接、已使能、已标定。返回 `bool`：`True` 表示动作达到了
目标；`False` 表示跑了但没达到（原因写日志）。

| 方法 | 成功的含义 |
| --- | --- |
| `open(speed_mm_s=None)` | 顶到并停在标定张开限位附近；若 `work_stroke_mm` 小于行程且已设置，则停在工作行程处。 |
| `close(speed_mm_s=None)` | 顶到并停在标定闭合限位附近。 |
| `set_gripper_distance(distance_mm, speed_mm_s=None)` | 以 mm 为单位的绝对开口（0 = 闭合）普通定位，目标夹进标定行程；到位且未堵转即成功，不顶限位。 |
| `set_motor_angle(angle_rad, speed_mm_s=None)` | 同上，目标用电机弧度表示。 |
| `follow_gripper_distance(distance_mm, kp=None, kd=None, dq=0.0, tau=0.0, force_feedback=None)` | 已发出一帧 MIT 跟随帧（遥操用，不走斜坡）。见[遥操](#遥操)。 |
| `grasp(force_n=None, hold_s=0.0)` | 自适应闭合到堵转，然后按 `force_n × 0.1 Nm` 的纯力矩保持。靠近闭合限位时，交接前会做一次短验证性再夹；仍能移动则判为自由空间静摩擦死区并返回 `False`。`hold_s=0` 一直保到 `release()`、故障或 Ctrl+C。没咬住、越位护栏、堵转保护、保力中途通信失败均返回 `False`；接近段通信失败抛 `CommError`。 |
| `release(clearance_mm=10.0)` | 线程安全：结束进行中的夹取，并从**声明的夹持点**再张开 `clearance_mm`，夹到标定张开限位。`grasp(hold_s=0)` 阻塞在另一线程时也能调用。急停卸力请用 `disable()`。 |

默认速度是 `MotionConfig.speed_mm_s`（50 mm/s）；任何速度参数传 `None` 都取它。

### 遥操

遥操由调用方驱动：SDK 只提供一个流式原语，不跑后台循环。

```python
import time

while running:
    leader_mm = leader.get_gripper_distance()          # 或 Pika Sense
    follower.follow_gripper_distance(leader_mm)        # 一帧 MIT
    time.sleep(0.005)                                  # 200 Hz
```

从手示例默认 `--channel can1`（双机同主机的约定）；单机只有 `can0` 时请显式传
`--channel can0`。若主手是另一只 LiteGrip，读之前先调 `leader.refresh_status()` —— 否则它空闲时不上报
状态。开始遥操前让主手处于跟随端当前位置附近：这个原语不做斜坡、不做对齐，第一帧就
可能下发全部误差。

`follow_gripper_distance()` 只发一帧 MIT 并立即返回，不走斜坡、不 settle、不等待运动。
`distance_mm` 是从闭合零点算起的绝对开口，按**跟随端自己的标定**换算；因此即使主从标定
文件的零点、方向或比例不同，主手也可以直接发自己的 `get_gripper_distance()`。跟随端
行程更短时仍会被夹到自己的行程；行程不同时请由调用方先归一化。目标夹进标定行程。边界：kp 0–500、kd 0–5、`|dq| ≤ 10 rad/s`、`|tau| ≤ 3.5 Nm`；非法值在
发帧前抛 `CommandError`，返回 `False` 只表示传输失败（原因在 `last_send_error`）。主手
读取、循环节拍和看门狗由调用方负责：循环停下时请调用 `disable()` 或继续发保持帧 ——
电机在静默约 400 ms 后会锁存通信丢失故障（`TIMEOUT=8000` × 50 µs）；SDK 的阻塞等待会发 0xCC keepalive。

**力反馈（默认关闭）。** 主手可以把从手的实测夹持力渲染成阻力：

```python
leader.configure_force_feedback(enabled=True)      # 默认：关闭
while running:
    follower.follow_gripper_distance(
        leader.get_gripper_distance(),
        force_feedback=leader.render_force_feedback)
    time.sleep(0.005)
```

`follow_gripper_distance(..., force_feedback=cb)` 在成功发帧后调用
`cb(follower.get_gripper_force())`；传入主手的 `render_force_feedback` 即可。主手发的帧为
`kp=0`、`kd=damping`、`tau=-close_sign × min((F-threshold) × gain, max_tau)`。默认：阈值
2 N、增益 0.1 Nm/N、上限 1 Nm、阻尼 0.1、超时 0.3 s；超过超时的反馈会撤掉阻力和阻尼。
力值是近似值（电机力矩 × 10，不是力传感器）。`render_force_feedback` **默认关闭**，未配置
时不发帧。跨网络时请自行转发 `get_gripper_force()`，并把样本年龄传给
`render_force_feedback(force, age_s=...)`。

### 反馈

未连接时 getter 抛 `NotInitializedError`，绝不返回看似合理的零。除 `get_state(wait=True)`
外，其余 getter 读缓存、不阻塞；总线空闲时先调用 `refresh_status()`。

| 方法 | 返回 |
| --- | --- |
| `get_state(wait=True)` | `GripperState` 快照：`position_rad`、`velocity_rad_s`、`torque_nm`、温度、`error_code`、`fresh`、`data_age_s`、`timestamp`、`is_stale()`。`wait=True` 最多等 0.5 s 等一帧状态。 |
| `refresh_status()` | `bool` —— 发 0xCC 并短暂等待状态帧。不带动夹爪。 |
| `get_motor_data()` | `{"Position": rad, "Speed": rad/s, "Torque": Nm}` |
| `get_motor_status()` | `{"Status": "0x01", "DriverTemp": °C, "MotorTemp": °C}` —— MOS 与线圈。 |
| `get_motor_position()` | `float`，rad |
| `get_motor_speed()` | `float`，rad/s |
| `get_motor_torque()` | `float`，Nm |
| `get_gripper_distance()` | `float`，从闭合零点算起的开口 mm |
| `get_driver_temp()` | `int`，°C（MOS） |
| `get_motor_temp()` | `int`，°C（线圈） |
| `get_status_raw()` | `str`，状态/错误码的十六进制文本（如 `"0x01"`） |
| `get_gripper_force()` | `float`，实测夹持力 N（近似、缓存）—— 从手侧力反馈值 |
| `get_version()` | `str`，**SDK/包**版本 —— 不是设备固件（本协议读不到固件）。 |

## 单位与不变量

| 量 | 单位 |
| --- | --- |
| 开口/距离 | mm，从闭合零点起算。0 = 闭合，满行程 = 张开。 |
| 电机角度/位置 | rad |
| 速度 | 高层 API 用 mm/s，反馈用 rad/s |
| 力矩 | Nm |
| 力 | N，**近似**：1 Nm ≈ 10 N，未标定 |
| 温度 | °C |

标称几何：两爪行程 85 mm，探测压入量 1 mm，记录跨度 86 mm。状态过期阈值 0.5 s。
行进段指令领先上限 4 mm；默认 `kp=100`、`kd=2`。

`GripperState.position_mm` 与 `get_gripper_distance()` 用的是**开口量**（单侧从闭合
起算），不是两爪总间距。

## 标定

一份标定 = 两个限位 + 一个比例：`pos_closed_rad`、`pos_open_rad`、`rad_to_mm`。
两个限位谁大谁小就声明了安装方向，反装不需要额外的开关。

随包 `factory_calibration.json` 是**参考单元**，不是逐台/逐序列号标定。零点、行程和比例
每台都有差异；在依赖绝对 mm 精度或 `open()`/`close()` 的限位判据之前，必须对每台执行
`calibrate()`（或加载厂家提供的逐台标定文件）。

1. 清空行程。
2. `gripper.calibrate()` —— 低刚度探测两端，更新 `config`，写入
   `~/.litegrip/<channel>_calibration.json`。
3. 下次启动构造函数自动读取该文件（`LITEGRIP_CALIB` 优先）。

探测有两道独立护栏：指令领先量不超过 `calib_step_rad`（压紧力矩至多
`kp × step`）；`|力矩|` 一到 `MotionConfig.calib_tau_limit` 立即停止推进。只靠
位置式堵转判据停不下来 —— 顶住硬限位时编码器仍会因结构让位而缓慢变化。

`set_zero()` 不同：它把**当前**编码器角度记为零，不测量行程、不改标定文件，且
不可逆。

## 安全

- **标定行程是所有下发指令的外边界。** `open` / `close` 的目标略越限位，好让
  运动靠堵转结束，但 `clamp_to_travel` 把每一帧指令封顶在标定限位上。标定偏了
  只会限制行程，不会让夹爪走出去。
- **带设定力的接近段有力矩预算。** `grasp()` 把设定值（`force_n × 0.1 × 0.9 Nm`）
  分给帧里的速度、阻尼和领先，撞上工件时压出的力矩不超过设定值。
- **行进段力矩保护独立于位置窗口。** 硬停但结构缓慢让位时位置窗口可能永远填不满；
  持续力矩加上速度跟不上会触发保护，撤力并返回 `False`。
- **只有夹爪停住后才交接给纯力矩保力段**；从起点就没动过、且预算低于脱离力时
  报 `no_bite` 拒夹，而不是把它变成一次带力冲程。
- **保力段有失控护栏。** 纯力矩帧没有位置误差可夹，越出闭合侧标定限位超过
  `MotionConfig.hold_runaway_rad` 就撤力并返回 `False`。正常夹取（含空载）不会触发。
- **松开是协调的，不是第二个发帧者。** 夹取线程拥有帧时，`release()` 只记请求，
  由夹取线程完成张开并回报结果。
- **非有限值在发帧前拒绝。** NaN/±inf 的目标、速度、力、保持时间都抛
  `CommandError`。
- **未标定拒绝运动。** 占位限位描述的不是真实行程。
- **`open` / `close` / `grasp` / `calibrate()` 期间手和物体离开行程。**
- **持续顶住会加热线圈。** 连续动作时看 `get_motor_temp()`。
- **`disconnect()` 会失能电机**，进程被杀也不会留下保持力矩。

## 异常

| 异常 | 触发条件 |
| --- | --- |
| `LiteGripError` | 基类。有电机错误码时 `str()` 追加 `[0xXXXX]`。 |
| `ConnectError` | CAN 接口不可用，或电机注册失败。 |
| `CommError` | 会话中 CAN 读写失败，包括帧没发出去。 |
| `CANTimeoutError` | 超时无应答。 |
| `CommandError` | 参数或状态非法：未标定、非有限值、速度非法、负余量、标定文件格式错误。 |
| `HardwareError` | 电机报故障：使能到不了 `err == 1`、故障清不掉、0xFE 未被接受等。 |
| `NotInitializedError` | 需要连接/使能时尚未连接/使能。 |

SDK 走标准 `logging`，logger 名是 `litegrip`。动作失败、通道不一致、通信失败等
WARNING 即使没配 handler 也会到 stderr。`describe_error(code)` 把电机错误码转成文字。

帧发不出去时原因留在 `gripper.last_send_error`：缺连接/使能抛
`NotInitializedError`，传输失败抛带原始原因的 `CommError`；下层没记录原因时消息
会照实说「原因未记录」，不替它猜。

## 示例

从仓库根直接运行，不需要安装、不需要 `PYTHONPATH`：

| 脚本 | 作用 |
| --- | --- |
| `examples/gripper_example.py` | 连接、读状态、张开/闭合、绝对距离、夹取/松开。 |
| `examples/open_close.py` | 重复张开/闭合，每个动作打印结果。 |
| `examples/move_to.py` | 运动到绝对开口（`--distance MM`）或电机角度（`--angle RAD`）。 |
| `examples/stroke_test.py` | 全行程张开/闭合循环；每个动作都成功才 exit 0。 |
| `examples/calibrate.py` | 探测两端并保存本通道标定。 |
| `examples/teleop_master.py` | 主手：通过 UDP 发布开口；可选力反馈（默认关闭）。 |
| `examples/teleop_slave.py` | 从手：跟随 UDP 开口目标；回传夹持力；看门狗保持最后目标。 |

所有示例都支持无硬件 `--help`。常见的台架检查：

```bash
python3 examples/stroke_test.py --channel can0 --cycles 3 --speed 10 --yes
```

## 测试

测试套件不需要硬件：引擎跑在 `tests/fake_can.py` 的运动学假 CAN 上，没有夹爪和
CAN 接口也能跑。

```bash
python3 -m unittest discover -s tests -t tests -v
```

覆盖 DM4310 编解码 golden 向量、传输层发送失败、运动引擎（领先上限、堵转窗口、
力矩预算、交接、松开协调）、标定、状态新鲜度、公开门面和示例的 `--help` 引导。

## 排错

| 现象 | 含义 |
| --- | --- |
| 抛 `CommandError` 并提到标定 | 当前配置未标定。跑 `calibrate()`，或用 `LITEGRIP_CALIB` / `load_calibration(path=...)` 指向有效文件。 |
| `ConnectError`，或一直收不到状态帧 | `can0` 没起，或波特率不是 1 Mbit/s。 |
| `enable()` 抛 `HardwareError` | 状态帧一直不是 `err == 1`。检查 24 V 供电，再 `clear_fault()`。 |
| `close()` 返回 `False` | 停稳位置离标定限位超过 `stop_tol`。检查标定是否描述真实行程；缺口来自探测比 `close` 压得更深。 |
| `grasp()` 返回 `False`，日志说没咬住 | 接近段预算压不出静摩擦。提高 `force_n`；参考机下限约 2.2–3 N。 |
| 越位护栏触发后 `grasp()` 返回 `False` | 保力时闭合侧没有东西拦着。检查标定和机械限位；夹爪已在标定行程外，先 `disable()`、手动推回，再 `enable()`。 |
| `load_calibration()` 返回 `False` | `path`、`LITEGRIP_CALIB`、本通道文件、出厂文件都读不到。 |
| 保力读数往下掉 | 保力本来就是纯力矩；夹爪被外力带着走得比前馈快时读数会低。没有可调的保力增益。 |
| `import litegrip` 加载的是旧代码 | 安装副本不会随 `git pull` 更新。看 `litegrip.__file__`，或在仓库里 `pip install -e .`。 |

## 许可证

Copyright © 2026 NEXFORM ROBOTICS。Apache License 2.0，见
[LICENSE](https://github.com/nexform-tech/litegrip-python/blob/main/LICENSE)。
