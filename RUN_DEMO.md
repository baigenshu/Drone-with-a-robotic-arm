# 运行仿真（本机已验证可用）

本说明记录**在本机（Ubuntu 20.04 + ROS Noetic + Gazebo 11 + PX4 v1.17-alpha）**上
把原仓库的空中机械手跑起来所做的移植。原仓库的 `Firmware/` 补丁基于 PX4
**v1.11.3**，无法直接覆盖到 v1.17，因此这里采用「复用已编译的 v1.17，只移植机械臂
模型」的方案。

原模型是 **Typhoon H480 六轴**；现已按需求**原地改成四轴**：机身换成 PX4 自带的
**Iris 四轴**，把机械臂链和云台插件挂上去。启动命令、模型名、MAVROS 接口均不变。

## 改动清单

PX4 仓库 `~/PX4-Autopilot`（原文件均已备份到 `/tmp/`）：

| 文件 | 改动 |
|---|---|
| `.../models/typhoon_h480/typhoon_h480.sdf.jinja` | 以 PX4 `iris` 四轴机身为底座（4 电机 X 形），挂上原有 3 自由度机械臂 + 夹爪；调大云台 PID 增益/力限幅；上调电机推力参数（Iris 原厂推力带不动机械臂）；夹爪摩擦由 100 降到 1 |
| 同上 `models/typhoon_h480/meshes/` | 新增四轴机身/桨 `iris.stl`、`iris_prop_ccw.dae`、`iris_prop_cw.dae`；机械臂 `base.dae`/`link2.dae`/`pinza.dae`（来自本仓库 `Firmware/typhoon_h480/meshes/`） |
| `ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480` | 由六轴改为四轴：`MAV_TYPE 2`、`CA_ROTOR_COUNT 4` + 四轴 `CA_ROTOR*`；`MNT_MODE_IN 3`（legacy `DO_MOUNT`）、`MNT_MODE_OUT 2` |

改完后重新生成模型并重建固件（若只改模型 SDF，则无需重建 PX4，重启仿真即可）：

```bash
cd ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
python3 scripts/jinja_gen.py models/typhoon_h480/typhoon_h480.sdf.jinja .
cd ~/PX4-Autopilot && make px4_sitl_default
```

## 四轴改造说明

- 机身：`iris.stl`（PX4 自带 3DR Iris 四轴，4 电机 X 形）。
- 机械臂：`cgo3_mount_joint` 的 parent 仍是 `base_link`，pose `0 0 0`，从机身中心向下垂。
- Airframe：`CA_ROTOR_COUNT 4`，`CA_ROTOR0..3` 直接照抄 `10015_gazebo-classic_iris`，
  保证与 SDF 的 rotor 索引/转向（`motorNumber`/`turningDirection`）一一对应。
- **推力**：Iris 原厂电机（`motorConstant 5.84e-06`、`maxRotVelocity 1100`）最大升力
  约 23 N，而机身 1.5 kg + 机械臂约 0.8 kg 总重约 23 N，推重比只有 ~1.0 起不来。
  已把电机改为 `motorConstant 8.54858e-06`、`maxRotVelocity 1500`、
  `input_scaling 1500`，推重比 ~3.3，可稳定悬停。

## 控制原理

机械臂**没有独立控制器**，而是复用 PX4 的云台(gimbal)通道：

```
MAVROS /mavros/mount_control/command (MountControl, mode=2)
  -> PX4 gimbal 模块 (MNT_MODE_IN=3, InputMavlinkCmdMount)
  -> MAVLink gimbal v2 (GIMBAL_DEVICE_SET_ATTITUDE, UDP 13030)
  -> Gazebo libgazebo_gimbal_controller_plugin.so
  -> cgo3_vertical_arm_joint / cgo3_horizontal_arm_joint / cgo3_camera_joint
```

关节映射（见 SDF 里 `gimbal_controller` 插件）：

| 指令轴 | Gazebo 关节 |
|---|---|
| yaw | `cgo3_vertical_arm_joint` |
| roll | `cgo3_horizontal_arm_joint` |
| pitch | `cgo3_camera_joint`（夹爪） |

## 运行

**终端 1 — 仿真（带 GUI）**

```bash
~/VR-teleoperated-aerial-manipulator/run/run_sim.sh
# 等价于： cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_typhoon_h480
```

**终端 2 — MAVROS**

```bash
~/VR-teleoperated-aerial-manipulator/run/run_mavros.sh
# 等价于： roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580
```

**终端 3 — 用键盘控制机械臂**

```bash
python3 ~/VR-teleoperated-aerial-manipulator/run/arm_teleop.py --takeoff
#   a/d = 大臂回转   w/s = 小臂开合   q/e = 夹爪俯仰
#   空格/h = 回中     x = 退出
```

`arm_teleop.py` 含关节限幅与平滑限速，只依赖 MAVROS，控制链与实机完全一致。

> ⚠️ **必须先起飞**。停机在地面时，机械臂会垂到地面 (z≈0) 被卡住，三个关节几乎都不动。
> `--takeoff` 会先解锁并 `AUTO.TAKEOFF` 到 ~2.5 m，然后开始键盘遥控。
>
> 想手动飞：用 `~/QGroundControl.AppImage` 解锁起飞即可。

也可以直接用 `rostopic` 测试（起飞后）：

```bash
rostopic pub -r 10 /mavros/mount_control/command mavros_msgs/MountControl \
  "{header: {frame_id: ''}, mode: 2, pitch: 40.0, roll: -30.0, yaw: 50.0, altitude: 0.0, latitude: 0.0, longitude: 0.0}"
```

## 文档 / 实机对接

- `docs/mavlink_gimbal_device_v2.md`：如何把仿真里的机械臂控制搬到真机
  （链路、PX4 参数、协议时序、关节映射、安全清单）。
- `docs/gimbal_device_v2_firmware.ino`：实机机械臂控制板的 **MAVLink 云台协议 v2 设备端**
  参考固件（Arduino/STM32）。它对应的就是仿真里的
  `libgazebo_gimbal_controller_plugin.so`。

## 已知问题 / 注意事项

1. 原仓库 `Firmware/mc_rate_control` 里的几何控制器**有 bug**（姿态订阅 fd 恒为 -1、
   `orb_copy` 话题写错），本方案**未使用**它，飞行用的是 v1.17 原版控制器。
2. 原仓库把机械臂「roll」关节轴设成 `0 -1 0`（Y 轴），而 PX4 云台插件按
   `LocalAxis[0]`（X 分量）取方向，导致 roll 力矩恒为 0 —— 这是原仓库自身的 bug
   （v1.11.3 同样如此）。本移植把该关节轴改回 `-1 0 0`，roll 才可用。
3. 机械臂网格 `base.dae`/`link2.dae`/`pinza.dae` 较大（约 110 MB），Gazebo 首次加载会慢几秒。
4. 四轴改造后机身是 Iris，没有起落架；机械臂仍垂到地面，停机时会被卡住，演示前必须先起飞。
5. 夹爪 collision 的摩擦系数已从原仓库的 `mu=100` 降到 `1`（原值会让机械臂在地面上"粘住"，
   同时也不适合演示运动）。如果后面要做抓取实验，可以再调回较大的摩擦。
6. **键盘控制没反应？** `arm_teleop.py` 是直接读终端的原始按键的，而**键盘事件只会发给
   当前有焦点的窗口**。运行后请**先用鼠标点一下那个终端窗口**，再去按 `a/d/w/s/q/e`；
   如果长时间按了没反应，节点会在 10 秒后打印一条警告提醒。
7. 机械臂关节的云台 PID 与关节阻尼已针对「垂挂臂 + 重力」重新调过
   （关节 `<damping>` `0.1 → 2.0`；云台 PID 降 P、大幅增 D、开小积分 ±0.3），
   目的是消除末端夹爪的自激振荡（修复前会在 1.0→2.2 rad 之间自己来回摆）。
   若仍需调整，改 `typhoon_h480.sdf.jinja` 里 `gimbal_controller` 的 PID 后重新生成模型即可。
