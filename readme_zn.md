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
    gripper.load_calibration()   # 现场标定，没有就回退出厂标定
    gripper.enable()             # 反复重试，直到状态帧回读到 err == 1

    gripper.open()                                     # 50 mm/s 走到张开侧
    gripper.close()                                    # 50 mm/s 走到闭合侧
    result = gripper.grasp(force_n=20.0, hold_s=3.0)   # 闭合到夹住，再保 20 N 力
    print(result.reached, result.stalled, result.cycles)
```

`with` 退出时会调 `disconnect()`，默认顺带失能电机。如果想让夹爪在离开代码块后仍保持
使能，传 `LiteGrip(..., disable_on_disconnect=False)`，或设
`gripper.disable_on_disconnect = False`。

## 六个动作接口

要让夹爪动起来就用这六个。每一个都会自己校验结果再报成功，所以调用方不必再重写斜坡和
堵转判据。

| 方法 | 行为 | 返回 |
| --- | --- | --- |
| `open(speed_mm_s=None)` | 按斜坡**越过**标定的张开侧限位，由机械限位结束这趟运动。 | `MoveResult` |
| `close(speed_mm_s=None)` | 同上，朝闭合侧。 | `MoveResult` |
| `grasp(force_n=None, hold_s=0.0)` | 闭合到堵转（即夹住），然后持续输出 `force_n`。`hold_s=0` 表示不限时长。 | `GraspResult` |
| `zero()` | 完整标定：探测两端机械限位，算出行程与 `rad_to_mm`，并存盘。 | `CalibrationData` |
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
| `stop_lead_mm` | `1.0` | 压紧段的领先上限，压紧力矩约 `kp × stop_lead_mm` |
| `stop_tol` | `0.02` | 停稳位置距标定限位多近才算顶到位 rad |
| `force_n` | `20.0` | `grasp` 默认夹持力 |
| `hold_interval` | `0.2` | 保力的分片时长 s |
| `hold_kp` / `hold_kd` | `150.0` / `2.0` | 保力时用的刚度 / 阻尼 |
| `enable_retries` / `enable_retry_interval` | `3` / `0.2` | 使能重试次数与间隔 |
| `calib_kp` / `calib_kd` | `60.0` / `2.0` | `zero()` 探测用的刚度 / 阻尼 |
| `calib_step_rad` | `0.1` | `zero()` 的探测步长 |
| `calib_stall_delta` / `calib_stall_cycles` / `calib_max_iter` | `0.0015` / `5` / `80` | 探测的堵转判据 |
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
- **方向约定是数据，不是开关。** SDK 假定 rad 增大即闭合，两端极限都存在标定文件里。如果
  `pos_open_rad >= pos_closed_rad`，配置会被拒绝并抛 `CommandError`，因为此时 SDK 的
  clamp 是错的。
- **压紧力矩是有界的。** 行进段的 `max_lead_mm` 上限约 5 Nm；压紧段的 `stop_lead_mm` 上限
  约 `kp × stop_lead_mm / rad_to_mm` —— 默认（`kp=100`、`stop_lead_mm=1.0`、`rad_to_mm≈73.7`）
  约 `1.4 Nm`，远在额定 3 Nm 以内。压不实就往上调 `stop_lead_mm`，调到能稳定压住、又听不到
  撞击声为止；默认值只是算术结果，不是真机实测值，必须在真机上确认。
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

除了签名，还有三处行为变化要注意：

- `enable()` 现在会如实报失败。以前只要状态帧是 `0` 或 `1` 它就返回 `True`，于是「根本没
  使能」的电机看起来是使能的。那些忽略返回值继续往下跑的代码，现在会在启动时看到
  `HardwareError`。
- `zero()` 不等于 `calibrate()`。它按 `MotionConfig.calib_*` 的参数探测并保存结果，而
  `calibrate()` 保留着自己那套更旧的默认值，而且不存盘。
- `MoveResult.__bool__` 以前是 `reached and not stalled`，现在是 `ok`。对 `grasp` 闭合段
  两者一致；对 `open` 和 `close` 恰好相反 —— 成功顶到限位是 `stalled=True,
  reached=False`，所以 `if gripper.close():` 的含义变了，尽管类型没变。

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
