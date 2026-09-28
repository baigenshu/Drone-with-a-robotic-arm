# Drone with a Robotic Arm

> 一架带 3 自由度机械臂 + 夹爪的四轴无人机，在 **PX4 SITL + Gazebo Classic + MAVROS**
> 下可完整仿真，并可通过 **VR / 键盘**遥操作机械臂。
>
> 本项目在原作基础上做了**四轴化移植 + 本机可复现验证**，实机对接方案见 [`docs/`](./docs/)。

---

## 来源与致谢

本工程源自 **Rodolfo Isaac Verdín**（GitHub: [@Rodolfo9706](https://github.com/Rodolfo9706)）的开源项目
[`VR-teleoperated-aerial-manipulator`](https://github.com/Rodolfo9706/VR-teleoperated-aerial-manipulator)，
对应论文：

- *Teleoperated aerial manipulator and its avatar*，ICUAS 2021
- DOI: [10.1109/ICUAS51884.2021.947688](https://doi.org/10.1109/ICUAS51884.2021.947688)

本仓库在原作基础上做了以下工作，**核心模型、机械臂网格、控制思路均来自原作者**：

1. 把原 **Typhoon H480 六轴**机身原地改为 **Iris 四轴**（机械臂链、云台插件、启动命令保持不变）。
2. 修复原作中导致 **roll 关节力矩恒为 0** 的关节轴向 bug，并重新整定云台 PID / 关节阻尼。
3. 修正与实机不符的推力/摩擦参数，使其能稳定悬停、可演示。
4. 补充 **键盘遥操作脚本**、**运行脚本**与 **实机对接文档 + 设备端参考固件**。
5. 剔除上游仓库中的构建产物与重复网格，仓库体积大幅精简（约 190 MB 重复文件被移除）。

> 上游 `Firmware/` 补丁针对 PX4 **v1.11.3**，无法直接覆盖到新版 PX4；
> 本项目采用「复用已编译的新版 PX4，只移植机械臂模型」的方案，详见 [`RUN_DEMO.md`](./RUN_DEMO.md)。

---

## 仓库结构

```
.
├── RUN_DEMO.md                     # ⭐ 本机已验证可用的完整运行说明（先看这个）
├── Firmware/
│   ├── typhoon_h480/               # 机体 + 机械臂 Gazebo 模型（SDF / mesh）
│   │   ├── typhoon_h480.sdf        #   主模型：Iris 四轴机身 + 3-DoF 机械臂 + 夹爪
│   │   ├── meshes/                 #   机械臂与机体网格
│   │   ├── carpeta_typhoon/        #   模型早期版本 + joyarm.py（手柄控制示例）
│   │   └── orange/                 #   橙色调色板
│   ├── vmount/                     # PX4 云台(机械臂)模块补丁
│   ├── mc_rate_control/            # PX4 角速率控制器补丁（原作几何控制器，见「已知问题」）
│   └── logger/                     # PX4 日志模块补丁
├── run/
│   ├── run_sim.sh                  # 终端 1：启动 PX4 SITL + Gazebo
│   ├── run_mavros.sh               # 终端 2：启动 MAVROS
│   └── arm_teleop.py               # 终端 3：键盘遥操作机械臂（含限幅与平滑）
└── docs/
    ├── mavlink_gimbal_device_v2.md # 仿真 → 实机的迁移方案（链路/PX4 参数/时序/安全清单）
    └── gimbal_device_v2_firmware.ino # 实机机械臂控制板的 MAVLink 云台协议 v2 参考固件
```

---

## 快速开始

**依赖**：Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11 + PX4（SITL 可编译）+ MAVROS。

完整步骤、参数说明与踩坑记录见 **[`RUN_DEMO.md`](./RUN_DEMO.md)**。三条命令概览：

```bash
# 终端 1 —— 仿真（带 GUI）
run/run_sim.sh

# 终端 2 —— MAVROS
run/run_mavros.sh

# 终端 3 —— 键盘操控机械臂（会先自动起飞到 ~2.5 m）
python3 run/arm_teleop.py --takeoff
#   a/d = 大臂回转   w/s = 小臂开合   q/e = 夹爪俯仰
#   空格/h = 回中     x = 退出
```

> ⚠️ **必须先起飞**：停机在地面时机械臂会垂到地面被卡住，三个关节几乎都不动。

---

## 控制原理

机械臂**没有独立控制器**，而是复用 PX4 的**云台(gimbal)通道**——云台与 3-DoF 机械臂本质是同一类问题
（让负载指向给定姿态）：

```
MAVROS /mavros/mount_control/command   (mavros_msgs/MountControl, mode=2)
   └─▶ PX4 gimbal 模块                  (MNT_MODE_IN=3, InputMavlinkCmdMount)
        └─▶ MAVLink gimbal protocol v2  (GIMBAL_DEVICE_SET_ATTITUDE, UDP 13030)
             └─▶ Gazebo libgazebo_gimbal_controller_plugin.so
                  └─▶ cgo3_vertical_arm_joint / cgo3_horizontal_arm_joint / cgo3_camera_joint
```

| 指令轴 | Gazebo 关节 | 机械臂动作 |
|---|---|---|
| `yaw`   | `cgo3_vertical_arm_joint`   | 大臂回转 |
| `roll`  | `cgo3_horizontal_arm_joint` | 小臂开合 |
| `pitch` | `cgo3_camera_joint`         | 夹爪俯仰 |

**迁移到实机时只需把 Gazebo 插件换成一块跑 MAVLink 云台协议 v2 的单片机**，其余（MAVROS 话题、PX4 参数、
上位机代码）完全不变 —— 方案与设备端参考固件见 [`docs/`](./docs/)。

---

## 与原工程的主要差异

| 项目 | 原作 | 本仓库 |
|---|---|---|
| 机身 | Typhoon H480 六轴 | **Iris 四轴**（机械臂/云台接口不变） |
| `roll` 关节轴 | `0 -1 0`，导致力矩恒为 0 | 修正为 `-1 0 0`，roll 可用 |
| 电机推力 | 推重比 ≈ 1.0，起不来 | 推重比 ≈ 3.3，可稳定悬停 |
| 夹爪摩擦 | `mu = 100`（粘地） | `mu = 1` |
| 云台 PID / 关节阻尼 | 末端夹爪自激振荡 | 重新整定，振荡消除 |
| 遥操作 | Unity/VR + `joyarm.py` | 新增 `run/arm_teleop.py`（键盘，含限幅与平滑） |
| 仓库体积 | 含构建产物与重复网格 | 已清理，约 190 MB 重复文件被移除 |

---

## 已知问题

1. 原作 `Firmware/mc_rate_control` 的几何控制器**有 bug**（姿态订阅 fd 恒为 -1、`orb_copy` 话题写错），
   本项目**未使用**它，飞行使用 PX4 原版控制器。
2. 四轴化后机身是 Iris，**没有起落架**，机械臂仍垂到地面，停机时会被卡住；演示前必须先起飞。
3. 机械臂网格较大（约 100 MB），Gazebo 首次加载需要几秒。
4. `Firmware/typhoon_h480/typhoon_h480 (copia).sdf`、`carpeta_typhoon/` 为原作遗留的历史版本文件，保留仅为可追溯。

---

## 引用

若本项目对你的工作有帮助，请引用原作论文：

```bibtex
@inproceedings{verdin2021teleoperated,
  title     = {Teleoperated aerial manipulator and its avatar},
  booktitle = {2021 International Conference on Unmanned Aircraft Systems (ICUAS)},
  year      = {2021},
  doi       = {10.1109/ICUAS51884.2021.947688}
}
```
