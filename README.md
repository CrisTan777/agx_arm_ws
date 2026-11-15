# AgileX PiPER X 工作空间

松灵 PiPER X 六轴机械臂的 ROS 2 Jazzy 驱动与二次开发工作空间。

## 硬件 / 软件版本

| 项目 | 值 |
|---|---|
| 机械臂 | AgileX PiPER X（6-DoF，CAN 1 Mbps） |
| 固件 | `S-V1.9-0`（硬件 `H-V1.2-1`，生产日期 2026-08-07） |
| CAN 适配器 | gs_usb / candleLight，接口 `can0` |
| 末端执行器 | AgxGripper 两指夹爪 |
| 腕部相机 | 松灵双目相机 DaBai DC1（59.6 × 17.4 × 11.1 mm，基线 40 mm） |
| 系统 | Ubuntu 24.04 + ROS 2 Jazzy |
| 驱动 | `agx_arm_ros@ros2` (`60b8fafc`) |
| SDK | `pyAgxArm` 1.0.0（`d90e234b`，editable 安装） |
| URDF | `agx_arm_urdf@main` (`680fa925`) |

## 目录结构

```
agx_arm_ws/
├── src/
│   ├── agx_arm_ros/        # 官方 ROS 2 驱动（ctrl/description/msgs/moveit）
│   └── piper_x_app/        # 二次开发：MoveIt 规划/执行示例
├── third_party/
│   └── pyAgxArm/           # 官方 Python SDK（已加 COLCON_IGNORE）
└── tools/
    ├── env.sh                    # source 后进入工作空间环境（ROS_DOMAIN_ID=42）
    ├── setup_host.sh             # 首次：装依赖 + 启用 CAN + 安装 SDK
    ├── can_up.sh                 # 每次开机后启用 can0 @1Mbps
    ├── check_arm.py              # 只读检测（固件/关节角）
    ├── gripper_check.py          # AgxGripper 状态检测 / 开合测试
    ├── gripper_calibrate.py      # AgxGripper 零点/行程标定
    ├── switch_to_can_control.py  # 示教模式 -> CAN 控制切换
    ├── preflight_can_mode.py     # 启动前 CAN/控制模式预检
    ├── bringup_moveit.sh         # 启动 真机 + MoveIt + RViz
    ├── start_tmux.sh             # 一键 tmux 工作台
    ├── monitor_arm.py            # 实时状态监控
    └── CHEATSHEET.txt            # 常用命令速查
```

## 首次配置

```bash
cd ~/jiangqi/learning_part/agx_arm_ws
bash tools/setup_host.sh                 # 需要 sudo 密码
source tools/env.sh
colcon build --symlink-install --base-paths src
```

## tmux 一键工作台（推荐）

```bash
bash tools/start_tmux.sh            # 创建并进入工作台
bash tools/start_tmux.sh --restart  # 重启整个工作台
tmux attach -t agx_arm              # 已退出后重新进入
tmux kill-session -t agx_arm        # 关闭工作台
```

工作台包含三个窗口：

| 窗口 | 内容 |
|---|---|
| `moveit` | 驱动 + MoveIt + RViz，日志同时写入 `/tmp/agx_arm_moveit.log` |
| `shell`  | 已 `source tools/env.sh` 的命令窗口，启动时显示常用命令速查 |
| `monitor`| 关节角、`ctrl_mode`/`arm_status`、TCP 位姿实时刷新 |

启动流程会自动：启用 `can0`（需要时提示 sudo 密码）→ 预检 CAN/控制模式 →
启动 MoveIt → 若机械臂正常则自动使能（默认速度 10%、执行时才开控制门）。

常用选项：`--no-enable`（不自动使能）、`--detach`（只创建不进入）、
`--gripper`（装夹爪后使用，默认同时加载腕部 DaBai DC1 相机模型）、
`--no-camera`（不加载相机模型）。

> 如果预检检测到拖动示教模式，会拒绝自动使能并在 `moveit`/`shell` 窗口提示。
> 处理顺序：
>
> ```bash
> # 1. 单击 J5/J6 之间的示教按键，确认 LED 熄灭
> # 2. 退出示教、切 CAN、自动归零并使能（10% 速度）
> python3 tools/switch_to_can_control.py --robot piper_x --can can0 --zero
> # 3. 重启工作台，让驱动以 auto_enable=true 接管
> bash tools/start_tmux.sh --restart
> ```

## 日常启动

```bash
# 1. 开机后启用 CAN
bash tools/can_up.sh

# 2. 新终端进入环境（每个终端都要做）
source tools/env.sh

# 3. 只读检查
python3 tools/check_arm.py --robot piper_x

# 4. 启动 真机 + MoveIt + RViz（安全门控 + 10% 速度）
bash tools/bringup_moveit.sh            # 无末端
bash tools/bringup_moveit.sh gripper    # 已安装 AgxGripper（自动设置 TCP 0.1425m）

# 等价于：
# ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
#   can_port:=can0 arm_type:=piper_x effector_type:=none \
#   auto_enable:=false speed_percent:=10 auto_control_gate:=true

# 5. 断电重启后首次使用，需要手动使能电机
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```

> 如果机械臂处于拖动示教模式（`ctrl_mode=2`），先按下面的流程退出示教，再使能。

`auto_control_gate:=true` 时，只有 MoveIt 执行轨迹期间控制门才会打开，空闲时驱动会忽略
`/control/*` 指令。RViz 中选中 `arm` 组，拖动交互标记 → **Plan** → **Execute** 即可。

### 命令行规划 / 执行

`piper_x_app` 默认只规划，**必须加 `--execute` 才会动**：

```bash
# 回零（只规划）
ros2 run piper_x_app moveit_demo home

# 回零并执行
ros2 run piper_x_app moveit_demo home --execute

# 关节空间目标（rad）
ros2 run piper_x_app moveit_demo joint --values 0.2 0.3 -0.2 0.0 0.2 0.3 --execute

# 笛卡尔位姿目标（米 / 度，参考 tcp_link）
ros2 run piper_x_app moveit_demo pose --xyz 0.2 0.0 0.35 --rpy 0 90 0 --execute

# 降低速度
ros2 run piper_x_app moveit_demo joint --values 0.1 0 0 0 0 0 --vel 0.05 --acc 0.05 --execute
```

## 安装 AgxGripper 夹爪

> 参考 PiPER X 用户手册 §2.4.2。**安装前必须断电**，并把机械臂摆到方便操作、
> 断电后稳定的姿态。

1. **装限位螺栓**：在 J6 电机左上位置装 `M3×4` 限位螺栓（头径 4.5 mm）。
2. **装转接法兰**：把法兰限位槽对准限位螺栓，用 **6 颗 `M3×8` 内六角圆柱头螺钉**固定。
3. **装夹爪**：夹爪背面有防呆结构，插入后用 **`M3×14` 内六角圆柱头螺钉**固定。
4. **接线**：把附件包里的电源/通信线接到 J6 末端和夹爪。末端是 **XT30 2+2**
   接口（24 V，最大 2 A），末端 CAN 只兼容松灵设备，不要接第三方 CAN 设备。
   线缆要留足 J6 旋转余量，避免夹线。
5. **首次上电**：底座绿灯慢闪后，按手册把夹爪转到限位姿态，执行
   “Set J6 Joint Zero”，再使能并让 J6 回零，确认 J6 与夹爪的零位标记对齐。
   Linux 下可用 pyAgxArm 的 `robot.calibrate_joint(6)`，但务必先确认姿态正确。
6. **软件验证**：

```bash
source tools/env.sh
python3 tools/gripper_check.py          # 只读：宽度、夹持力、homing/driver 状态
python3 tools/gripper_check.py --open   # 张开到 0.1 m
python3 tools/gripper_check.py --close  # 闭合
```

7. **启动带夹爪的 MoveIt**：

```bash
bash tools/bringup_moveit.sh gripper              # 默认加载腕部相机模型
bash tools/bringup_moveit.sh gripper --no-camera  # 不加载相机模型
```

   - MoveIt 规划组 `gripper` 提供 `gripper_open` / `gripper_half` / `gripper_close`
   - TCP 自动设为法兰前方 `0.1425 m`（夹爪指尖）
   - 命令行控制夹爪（默认只规划，加 `--execute` 才动）：

```bash
ros2 run piper_x_app moveit_demo gripper --width 0.05 --execute
ros2 run piper_x_app moveit_demo gripper --width 0.0 --execute
```

   - `/feedback/gripper_status` 可查看夹爪状态；`homing_status=false` 时需要先做夹爪标定
     （`calibrate_gripper()`，步骤见 pyAgxArm 文档）
   - 夹爪约 0.5 kg，PiPER X 额定负载 1.5 kg，剩余可用负载约 1 kg，抓取规划时注意

> **重要：** 夹爪电机只在 **CAN 控制模式** 下工作。拖动示教模式下夹爪反馈正常、使能位也会响应，
> 但主控不会给电机使能，表现为“能读状态但夹爪不动”。使用夹爪前必须先退出示教、切到 CAN 控制。
>
> 另外，本机这版固件的 `homing_status` 即使标定 ACK 成功也一直是 false，但实际宽度控制和
> 力反馈正常（已实测 0–0.0996 m 全行程）。不要因为该位为 false 就认为夹爪不可用。

## 腕部相机（DaBai DC1）

本工作空间已加入 **松灵双目相机 DaBai DC1 + 腕部相机支架** 的模型，用于 RViz /
MoveIt 可视化。当前只做 URDF 模型，暂未启动相机驱动。

| 项目 | 值 |
|---|---|
| 相机 | 奥比中光 DaBai DC1，59.6 × 17.4 × 11.1 mm，基线 40 mm，17 g |
| 安装 | 官方 `realsense_mid_stand` 支架，固定在 `gripper_base` 侧面 |
| 模型 | `piper_x_with_camera_description.xacro` |
| 支架网格 | `piper_x/meshes/dae/camera_stand.dae`（来自 agx_arm_sim） |
| 主要 frame | `camera_stand_link` → `dabai_camera_mount_link` → `camera_link` |

### 在 RViz 中显示

```bash
# 只看模型（不启动真机/MoveIt）
ros2 launch agx_arm_description display.launch.py \
  arm_type:=piper_x effector_type:=agx_gripper with_camera:=true

# 真机 + MoveIt + 相机模型（推荐的 tmux 工作台）
bash tools/start_tmux.sh --gripper              # 默认带相机模型
bash tools/start_tmux.sh --gripper --no-camera  # 不加载相机模型
```

在 RViz 左侧 `RobotModel` 中可以看到相机支架和相机；`camera_link` 的 **+X 指向镜头前方**。
若只关心相机，可把 Fixed Frame 设为 `camera_link` 并放大。

### 说明

- 相机本体按实物照片建模：**圆角梯形银色外壳 + 深色内嵌前面板**，前面从左到右为
  **左 IR、RGB、中央大镜头、LDM/LDP 双窗模块、右 IR**，双 IR 基线 40 mm。
  外壳和前面板使用生成的 STL（`dabai_housing.stl` / `dabai_front_panel.stl`），
  镜头用圆柱几何体近似；拿到官方 STEP/STL 后替换对应 `<visual>` 即可。
- 支架使用官方 `realsense_mid_stand.dae`，安装位沿用官方 PiPER X 参数
  `xyz="-0.034 -0.001 0.022"`（相对 `gripper_base`）。实际支架孔位不同的话改这个
  `camera_stand_joint` 的 origin。
- 相机约 17 g，支架约 0.2 kg，连同夹爪约 0.7 kg，仍在 PiPER X 1.5 kg 额定负载内。
- 目前只做 URDF 可视化，没有启动相机驱动。后续接入 Orbbec SDK / `astra_camera` 时，
  以 `camera_link` 为父坐标系发布深度/RGB 光学 frame 即可。
- `with_camera:=true` 只在 `arm_type:=piper_x` + `effector_type:=agx_gripper` 时生效。

## 拖动示教 ↔ CAN 控制（重要）

**示教模式下固件会忽略所有 CAN 指令。** 示教按键位于 J5/J6 之间：

- LED **常亮**：正在记录轨迹
- LED **闪烁**：正在复现轨迹
- LED **熄灭**：示教已停止

切回 CAN 控制的正确流程（官方手册）：

1. 在示教模式下把机械臂摆到**零位**（六个关节角≈0，折叠收拢姿态）；
2. 单击示教按键，确认 LED 熄灭；
3. 运行：

```bash
source tools/env.sh
# 已手动归零：待机 -> CAN，使能并保持当前姿态
python3 tools/switch_to_can_control.py --robot piper_x --can can0

# 未手动归零：让程序以 10% 速度自动归零
python3 tools/switch_to_can_control.py --robot piper_x --can can0 --zero
```

脚本会依次执行：待机模式 → CAN 控制模式 → 使能 → 保持/归零，并打印 `ctrl_mode`、
`arm_status` 和各关节使能状态。

## 安全相关服务 / 话题

```bash
# 急停（阻尼停止）
ros2 service call /emergency_stop std_srvs/srv/Empty

# 使能 / 失能（SetBool）
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"

# 关节反馈 / 末端位姿
ros2 topic echo /feedback/joint_states --once
ros2 topic echo /feedback/tcp_pose --once
```

> 注意：`/exit_teach_mode` 会把机械臂**先开到零位**再使能；现场空间不足时不要直接调用，
> 优先使用上面的 `switch_to_can_control.py`。

## 与 nexArm_ws 的隔离

本机默认 DDS 域上还运行着 `nexArm_ws`（`/dev/rrc` 串口板，不占用 can0）。
`tools/env.sh` 将本工作空间固定到 **`ROS_DOMAIN_ID=42`**，避免话题互相干扰。
所有终端操作前都要 `source tools/env.sh`，否则看不到这台机械臂的话题。

## 二次开发入口

- 驱动话题：`/feedback/joint_states`、`/feedback/tcp_pose`、`/control/joint_states`、
  `/control/move_j`、`/control/move_p`、`/control/move_l`、`/control/move_c`
- MoveIt action：`/move_action`；控制器：`/arm_controller/follow_joint_trajectory`
- 规划组：`arm`（`base_link → tcp_link`）
- 新功能建议放在 `src/` 下新建包，然后：

```bash
colcon build --symlink-install --base-paths src --packages-select <pkg>
```

## 故障排查

```bash
# CAN 是否有反馈帧（正常约 1600 帧/秒）
timeout 3 candump -n 20 can0

# 接口状态与错误计数
ip -details -statistics link show can0

# 固件 / 控制模式
python3 tools/check_arm.py
```

- `ctrl_mode=2`：拖动示教，CAN 指令无效，按上面的流程退出
- `ctrl_mode=1`：CAN 控制
- 无反馈：检查 24V 电源、CAN 线、`can0` 是否为 1 Mbps
- MoveIt 执行无动作：确认 `control_enabled` 门控是否打开、`speed_percent` 是否为 0
- MoveIt 报 `Joint 'joint4' from the starting state is outside bounds`：
  控制器在关节限位处会轻微过冲（实测 J4 到 1.5537，URDF 限位 1.553343），
  MoveIt 的 `CheckStartStateBounds` 是严格检查，会把之后所有规划都拒掉。
  本工作空间已在 `third_party/pyAgxArm/pyAgxArm/api/constants.py` 里把 `piper_x`
  的限位收紧 0.005 rad 作为安全余量。若仍出现，先用 `moveit_demo joint`
  把该关节移回限位内再规划。
- **J5 超限保护**：产品手册 J5 限位为 ±75°。本工作空间将操作限位设为
  **SDK ±74.0°**（`constants.py`，驱动中已调用 `set_joint_limits_enabled(True)`）
  和 **URDF/MoveIt ±74.2°**。请求 80° 会被夹到约 74.1°，
  距离手册限位仍有约 0.9° 安全余量。
- **J2/J3 零位过冲**：URDF 在 J2 下界和 J3 上界各放宽 0.005 rad，
  SDK 命令仍钳在 0，避免零位附近轻微过冲导致 MoveIt 起始状态误报。
- **RViz 看不到腕部相机**：确认启动参数包含 `with_camera:=true`，且
  `effector_type:=agx_gripper`、`arm_type:=piper_x`。可用下面的命令确认 URDF 和 TF：

```bash
ros2 topic echo /robot_description --once | grep -c camera_link   # >0 表示模型已加载
ros2 run tf2_ros tf2_echo base_link camera_link                   # 应输出平移/旋转
```

  如果仍看不到，在 RViz 的 `RobotModel` 里展开 `Links`，确认 `camera_link` /
  `camera_stand_link` 的 `Value` 是勾选状态。
