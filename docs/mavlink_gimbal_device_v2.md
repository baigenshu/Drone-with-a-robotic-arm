# 实机对接：MAVLink 云台协议 v2 设备端参考

本文件说明**如何把仿真里调通的机械臂控制，迁移到真实无人机**。

配套代码骨架：[`gimbal_device_v2_firmware.ino`](./gimbal_device_v2_firmware.ino)（Arduino/STM32）。

---

## 1. 为什么用"云台协议"

PX4 没有专门的机械臂控制器，我们复用了它的**云台(gimbal)子系统**：云台本质就是
「让一个 3 自由度负载指向给定姿态」，和 3 自由度机械臂是同一类问题。

协议标准：<https://mavlink.io/en/services/gimbal_v2.html>

## 2. 仿真组件 ↔ 实机组件的对应关系

| 仿真里 | 实机上 |
|---|---|
| Gazebo 关节 `cgo3_*_joint` | 真实舵机 / 电机 |
| `libgazebo_gimbal_controller_plugin.so` | **本单片机固件**（`gimbal_device_v2_firmware.ino`） |
| Gazebo UDP `13030`（插件发） | 飞控的一个 **串口 / UDP / CAN** MAVLink 通道 |
| `mavlink start -m gimbal`（PX4 SITL） | `MAV_x_CONFIG` 上启动 `-m gimbal` 的 MAVLink 实例 |
| `MNT_MODE_OUT=2` | 同左（保持 MAVLink 云台协议 v2） |
| `MNT_MODE_IN=3` | 同左（命令怎么进飞控） |

> 换句话说：**仿真调通后，把 Gazebo 插件换成真实单片机固件**即可，其余全部不变。

## 3. 链路与 PX4 参数

```
地面站/机载电脑 ──MAVROS MountControl──▶ 飞控(PX4 gimbal 模块)
                                              │  MNT_MODE_OUT=2
                                              │  MAVLink 云台协议 v2
                                              ▼
                                        机械臂控制板(单片机)
                                              │  PWM/总线
                                              ▼
                                            舵机
```

飞控侧需要配置：

| 参数 | 值 | 说明 |
|---|---|---|
| `MNT_MODE_IN` | `3`（`MAVLINK_DO_MOUNT`）或 `0`/`4` | 命令从哪进来（我们用 3，与仿真一致） |
| `MNT_MODE_OUT` | `2` | 输出 MAVLink 云台协议 v2 |
| `MNT_MAV_SYSID` | `1` | 云台设备所在系统（一般同飞控） |
| `MNT_MAV_COMPID` | `154` | 云台设备的 component id（`MAV_COMP_ID_GIMBAL`） |
| `MAV_x_CONFIG` / `MAV_x_BAUD` | 对应串口 | 把一路 MAVLink 指到机械臂控制板 |

并在启动脚本里以云台模式启动该 MAVLink 实例，例如：

```sh
mavlink start -d /dev/ttyS2 -b 115200 -m gimbal
```

## 4. 协议时序

```
设备(单片机)                          飞控(PX4)
   │  HEARTBEAT (MAV_TYPE_GIMBAL)        │   启动后周期性发送
   ├───────────────────────────────────▶ │
   │  GIMBAL_DEVICE_INFORMATION          │
   ├───────────────────────────────────▶ │   PX4 记录 gimbal_device_id
   │                                     │
   │        ◀── COMMAND_LONG:            │   PX4 主动请求设备信息
   │            MAV_CMD_REQUEST_MESSAGE  │   (param1 = GIMBAL_DEVICE_INFORMATION)
   │  GIMBAL_DEVICE_INFORMATION + ACK    │
   ├───────────────────────────────────▶ │
   │                                     │
   │        ◀── GIMBAL_DEVICE_SET_ATTITUDE (四元数 q + 角速度 + flags)
   │            (持续 20~50 Hz)           │   目标姿态
   │        ◀── AUTOPILOT_STATE_FOR_GIMBAL_DEVICE (机体姿态, 用于增稳)
   │                                     │
   │  GIMBAL_DEVICE_ATTITUDE_STATUS      │   可选：上报实际姿态
   ├───────────────────────────────────▶ │
```

要点：

- 设备**必须发 HEARTBEAT**，否则飞控的 MAVLink 路由不会接受它的消息。
- `gimbal_device_id` 由设备自己在 `GIMBAL_DEVICE_INFORMATION` 里给出（仿真里是 0）。
- `GIMBAL_DEVICE_SET_ATTITUDE.q` 是**目标姿态四元数**；`flags` 里可能有
  `NEUTRAL`（回中）、`RETRACT`、`YAW_LOCK`（yaw 为地理系）。
- 设备要实现**限位、限速、软启动**，并把实际可达范围写在
  `GIMBAL_DEVICE_INFORMATION` 的 `roll_min/max` 等字段里（**单位：弧度**）。

## 5. 固件骨架的关键逻辑

见 [`gimbal_device_v2_firmware.ino`](./gimbal_device_v2_firmware.ino)，核心只有四件事：

1. **收 `GIMBAL_DEVICE_SET_ATTITUDE`**
   四元数 → `roll/pitch/yaw`（ZYX）→ 夹到关节限位 → 作为目标角。

   ```c
   quatToRPY(sa.q, &roll, &pitch, &yaw);
   targetDeg[J_YAW]   = clampf(yaw   * 180/PI, LIM_MIN[J_YAW],   LIM_MAX[J_YAW]);
   targetDeg[J_ROLL]  = clampf(roll  * 180/PI, LIM_MIN[J_ROLL],  LIM_MAX[J_ROLL]);
   targetDeg[J_PITCH] = clampf(pitch * 180/PI, LIM_MIN[J_PITCH], LIM_MAX[J_PITCH]);
   ```

2. **平滑插值（限速）**：每 20 ms 让实际角向目标角靠拢，每次最多走
   `MAX_RATE_DPS * dt`，避免舵机跳变。

3. **应答 `MAV_CMD_REQUEST_MESSAGE`**：回 `GIMBAL_DEVICE_INFORMATION` + `COMMAND_ACK`。

4. **周期发送** `HEARTBEAT`（1 Hz）、`GIMBAL_DEVICE_INFORMATION`（1 Hz）、
   `GIMBAL_DEVICE_ATTITUDE_STATUS`（10 Hz）。

## 6. 关节映射与限位

| 协议轴 | 关节 | 仿真关节 | 限位（仿真） |
|---|---|---|---|
| yaw | 大臂回转 | `cgo3_vertical_arm_joint` | 无硬限位（软件建议 ±90°） |
| roll | 小臂开合 | `cgo3_horizontal_arm_joint` | **±45°**（±0.785 rad） |
| pitch | 夹爪俯仰 | `cgo3_camera_joint` | **−60.2° … +119.7°**（−1.05 … 2.09 rad） |

> 如果你的机械臂关节顺序不是「yaw→roll→pitch」，需要自己做一次基变换或逆运动学，
> 而不是像上面那样直接映射。

## 7. 安全清单（上实机前必看）

- [ ] **推力余量**：机械臂会显著增重、并使重心下移。重新标定 `MPC_THR_HOVER`，确认推重比足够。
  （仿真里就踩过坑：Iris 原厂电机带机械臂推重比只有 1.0，起不来。）
- [ ] **关节限位**：软件限位必须比机械限位更紧，并有硬限位开关/堵转保护。
- [ ] **限速与软启动**：上电先回中，再逐步放开速度；急停要能立刻切断舵机输出。
- [ ] **失联保护**：MAVLink 心跳超时后进入安全姿态（保持/回中/断电），不要让机械臂乱甩。
- [ ] **反作用力矩**：机械臂快速运动会扰动飞行姿态，先低速、低空验证。
- [ ] **供电**：舵机电流可能远超飞控供电能力，必须独立 BEC/电池供电。

## 8. 调试建议

- 用 `mavlink_shell.py`（PX4 自带）连飞控，执行 `gimbal status` 看是否发现设备、
  `gimbal test` 做单轴测试。
- 观察 PX4 是否收到 `GIMBAL_DEVICE_INFORMATION`（没收到会一直重发 `REQUEST_MESSAGE`）。
- 用 QGC 的 gimbal 页面或 `mavros_msgs/MountControl` 发指令验证。
- 先在地面、拆桨状态下验证机械臂全部动作范围，再装机试飞。
