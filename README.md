<div align="center">

# Tomato-Picker-ROS2

ROS 2 Humble + MoveIt 2 + SerialArm-Core tomato picking application stack

面向自研机械臂的末端执行器、RGB-D 感知、运动能力与采摘任务编排

[![License](https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Humble-00A896?style=flat-square)](https://moveit.picknik.ai/humble/)
[![SerialArm-Core](https://img.shields.io/badge/Backend-SerialArm--Core-00599C?style=flat-square)](https://github.com/Kaede-Rei/SerialArm-Core)

</div>

## 项目简介

Tomato-Picker-ROS2 是构建在 [SerialArm-Core](https://github.com/Kaede-Rei/SerialArm-Core) 之上的番茄采摘应用层

SerialArm-Core 负责机械臂模型、控制、安全、Hardware Backend、Robot Profile、ros2_control 与 MoveIt 2；Tomato-Picker 不重复实现机械臂底层

Tomato-Picker 负责：

- EEF 末端执行器
- MoveIt 2 运动能力封装
- RGB-D / Planning Scene 感知
- 单目标采摘阶段机
- 可选腕部 RGB-D 人工框选 GUI
- 整机部署入口

## 架构

```mermaid
flowchart TB
    GUI["Wrist ROI GUI"] --> Target["TargetObject / PickTarget"]
    Perception["RGB-D / Planning Scene"] --> MoveIt["MoveIt 2"]
    Target --> Task["Pick Task"]
    Task --> Motion["Motion Capability"]
    Task --> EEF["EEF Capability"]
    Motion --> MoveIt
    MoveIt --> ROS2["SerialArm ros2_control Adapter"]
    ROS2 --> Core["SerialArm-Core"]
    EEF --> SharedCAN["Shared CAN"]
    Core --> SharedCAN
```

```text
SerialArm-Core
    Robot / Control / Safety / Hardware / Profile / ros2_control / MoveIt

Tomato-Picker
    interfaces / motion / eef / perception / task / gui / bringup
```

## 当前能力

| 能力 | 入口 |
| --- | --- |
| 机械臂整机 + MoveIt 2 | SerialArm-Core `moveit.launch.py` |
| HOME / JOINT / POSE / LINE | `/tomato_picker/motion/move_arm` |
| EEF OPEN / CLOSE / STOP / SET_POSITION | `/tomato_picker/eef/command` |
| EEF 就绪状态 | `/tomato_picker/eef/ready` |
| 单目标采摘 | `/tomato_picker/task/pick` |
| Planning Scene 开关 | `/tomato_picker/perception/set_scene_enabled` |
| GUI 框选目标 | `/tomato_picker/gui/selected_target` |

## 仓库结构

```text
Tomato-Picker-ROS2/
├── repos/
│   └── serial_arm.repos
├── src/tomato_picker/
│   ├── interfaces/    # ROS contracts
│   ├── motion/        # MoveIt capability
│   ├── eef/           # Picking tool capability
│   ├── perception/    # RGB-D / Planning Scene
│   ├── task/          # Pick behavior
│   ├── gui/           # Optional wrist ROI GUI
│   └── bringup/       # Deployment assembly
├── API.md
└── README.md
```

## Quick Start

### 1. 获取 SerialArm-Core

当前工作区跟踪 SerialArm-Core 的 `main` 分支；Tomato-Picker 通过外部 `profile_file/resource_paths` 接口持有自己的 Robot Profile。首次获取依赖，在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
vcs import src < repos/serial_arm.repos
```

如果已存在 SerialArm-Core，保留原仓库并切换分支，不必删除重克隆：

```bash
git -C src/SerialArm-Core fetch origin main
git -C src/SerialArm-Core switch main || git -C src/SerialArm-Core switch --track origin/main
git -C src/SerialArm-Core pull --ff-only origin main
```

SerialArm-Core 的 `main` 自带一个 `tomato_picker_description` 示例包，与本工作区的正式同名包冲突。导入或更新 Core 后，应从工作区根目录执行：

```bash
touch src/SerialArm-Core/src/robot_supports/robots/tomato_picker/tomato_picker_description/COLCON_IGNORE
colcon list | grep '^tomato_picker_description[[:space:]]'
```

检查输出应只包含 `src/tomato_picker/description` 对应的一个包。`COLCON_IGNORE` 不改动 Core 跟踪的源码，后续拉取更新通常也无需重复执行。如需彻底移除 Core 的内置描述包，可仅删除 `src/SerialArm-Core/src/robot_supports/robots/tomato_picker/tomato_picker_description`，但这会产生 Git 已删除文件状态，并可能在后续更新中恢复

当前 Robot Profile 所有权：

```text
tomato_picker_bringup/config/robot_profiles.yaml
  ├── core       -> tomato_picker_description
  ├── hardware   -> SerialArm Damiao backend + DM-Arm hardware mapping
  ├── description-> tomato_picker_description
  ├── controllers-> tomato_picker_bringup/config/ros2_controllers.yaml
  └── moveit     -> tomato_picker_moveit_config
```

`arm.launch.py` 只负责把这些参数传给 SerialArm-Core 的通用 `moveit.launch.py`，不再重复实现 robot_state_publisher、ros2_control 和 controller spawner

### 2. 安装依赖并编译

从已有 workspace 更新接口后，建议先清理生成物：

```bash
rm -rf build install log
```

然后：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --rosdistro humble

colcon build --symlink-install
source install/setup.bash
```

### 3. 一键启动整机

```bash
ros2 launch tomato_picker_bringup bringup.launch.py
```

启动顺序：

```text
SerialArm hardware / controllers / MoveIt
    ↓ READY + stable
EEF homing
    ↓ READY
Motion
    ↓ Action READY
Task / Perception / optional GUI
```

ARM 的 Robot Profile、阻抗模式、停车逻辑与 ros2_control 生命周期全部由 SerialArm-Core 管理

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_profile` | `dm_arm_gray` | SerialArm Robot Profile |
| `serial_port` | `/dev/ttyACM0` | 可选运行时串口覆盖，例如 `/dev/ttyACM1` |
| `baudrate` | `921600` | 可选运行时波特率覆盖 |
| `bus` | `main_can` | 可选共享 bus name 覆盖 |
| `eef_mode` | `damiao` | `damiao` / `mock` / `off` |
| `start_perception` | `true` | RGB-D Planning Scene |
| `start_wrist_camera` | `false` | 启动 Orbbec Gemini 330 系列驱动 |
| `start_gui` | `false` | 启动腕部相机框选 GUI |
| `use_sim_time` | `false` | ROS simulation time |

> serial_port / baudrate / bus 默认值为 SerialArm-Core 的 Robot Profile 配置

SerialArm runtime hardware override 会同时作用到 EEF 的共享 CAN 配置，避免 ARM 与 EEF 使用不同串口或 bus

例如机械臂枚举到 `/dev/ttyACM1`：

```bash
ros2 launch tomato_picker_bringup bringup.launch.py \
  serial_port:=/dev/ttyACM1 \
  start_perception:=false
```

### 4. 检查系统

```bash
ros2 control list_controllers
ros2 action list | grep tomato_picker
ros2 service list | grep tomato_picker
```

真机模式应看到：

```text
joint_state_broadcaster       active
joint_trajectory_controller   active
eef_controller                active
```

EEF：

```bash
ros2 service call \
  /tomato_picker/eef/ready \
  std_srvs/srv/Trigger \
  "{}"
```

应返回：

```text
success: true
message: READY
```

## 手眼标定专用 Bringup

手眼标定不要启动整机 `bringup.launch.py`；专用入口只启动 `RobotSession` bridge，由它独占机械臂 Hardware Backend，并直接发布 SerialArm-Core 已计算的末端位姿：

```bash
ros2 launch tomato_picker_bringup handeye.launch.py
```

默认：

```text
robot_profile = tomato_picker
pose_topic    = /arm/pose
joint_state_topic = /arm/joint_states
publish_rate  = 30 Hz
start mode    = COMPLIANT_DRAG
```

可覆盖硬件连接：

```bash
ros2 launch tomato_picker_bringup handeye.launch.py \
  robot_profile:=dm_arm_gray \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  bus:=main_can
```

此入口不会启动 MoveIt、ros2_control、EEF、Task、Perception、GUI 或相机驱动，避免多个节点同时占用机械臂硬件

Handeye-Calibration-App 使用：

```text
ROS2 input type = Pose
input topic     = /arm/pose
message         = geometry_msgs/msg/PoseStamped
```

发布规则：

```text
RobotState == ACTIVE
AND snapshot.valid == true
AND snapshot.last_error is empty
        ↓
发布 /arm/pose、/arm/tool_transform、/arm/joint_states（同一快照、同一发布时刻）
```

因此 FAULT 期间以及 `clear_fault()` 后第一个新有效控制周期到来之前都不会发布旧位姿

用于手眼标定诊断的关节反馈来自同一 `RobotSession.snapshot.cycle.joint_state`，标准话题为 `/arm/joint_states`，消息类型为 `sensor_msgs/msg/JointState`，`name` 按 Profile 中的 `joint_names` 顺序，`position` 是弧度、`velocity` 是弧度每秒、`effort` 是关节反馈力矩（Nm）。这三个话题携带相同的 **ROS 发布时刻** 时间戳，便于记录和对应；它不是电机反馈的原始采样时间戳。拖拽标定时不要同时启动 ros2_control 以获取另一个 `/joint_states`。

```bash
ros2 topic echo /arm/joint_states
ros2 bag record -o handeye_diagnosis /arm/pose /arm/tool_transform /arm/joint_states
```

### FAULT 在线恢复

SerialArm-Core 在可恢复 FAULT 情况下提供在线恢复：在可恢复 `FAULT + fault_holding` 下 worker 会保持存活并持续维护 fault hold；bridge 提供：

```text
/handeye/fault/clear               std_srvs/srv/Trigger
/handeye/fault/compliant_recovery  std_srvs/srv/Trigger
/handeye/fault/rigid_hold           std_srvs/srv/Trigger
/handeye/drag/enable                std_srvs/srv/Trigger
```

普通瞬态可恢复 FAULT，例如偶发 `INVALID_DT`，推荐流程：

```text
COMPLIANT_DRAG
    ↓
FAULT + RIGID_HOLD
    ↓
等待 fault hold 稳定
    ↓
/handeye/fault/clear
    ↓
ACTIVE + RIGID_HOLD
    ↓
等待新的 valid snapshot
    ↓
/handeye/drag/enable
    ↓
COMPLIANT_DRAG
```

命令：

```bash
ros2 service call /handeye/fault/clear std_srvs/srv/Trigger "{}"
ros2 service call /handeye/drag/enable std_srvs/srv/Trigger "{}"
```

`clear_fault()` **不会自动重新进入拖拽模式**；这是安全设计：清故障后先保持 `ACTIVE + RIGID_HOLD`，必须由操作员在确认机械臂和新位姿正常后显式调用 `/handeye/drag/enable`

只有在需要人工把机械臂拖离 Core 允许的危险构型时才使用 FAULT compliant recovery：

```bash
ros2 service call /handeye/fault/compliant_recovery std_srvs/srv/Trigger "{}"
ros2 service call /handeye/fault/rigid_hold std_srvs/srv/Trigger "{}"
ros2 service call /handeye/fault/clear std_srvs/srv/Trigger "{}"
```

是否允许进入 compliant recovery 仍完全由 SerialArm-Core Safety 判定，Tomato-Picker 不绕过 Safety

如果 Core 无法建立/维持 fault hold，worker 会停止，此时不要继续标定，应检查通信、电机和硬件状态后重新启动

## RGB-D 手眼闭环定位验证

此入口仅用于 Handeye-Calibration-App 的眼在手上 RGB-D 分步视觉伺服验证，不执行采摘任务、不进入拖拽模式

### 1. 从标定模式切换到运动模式

先用 `handeye.launch.py` 完成标定、求解和冻结矩阵，保存手眼结果，再正常停止该进程并确认它已释放机械臂硬件

**不能同时运行 `handeye.launch.py`、`handeye_servo.launch.py` 或其他会创建独立 `RobotSession` 的程序**，同一硬件只允许 ros2_control 或 RobotSession 其中一方持有

机械臂位于安全工作范围、远离障碍物，现场具备物理急停后启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tomato_picker_bringup handeye_servo.launch.py
```

可覆盖当前 Robot Profile 的连接参数：

```bash
ros2 launch tomato_picker_bringup handeye_servo.launch.py \
  serial_port:=/dev/ttyACM0 \
  baudrate:=921600 \
  bus:=main_can
```

设备名、波特率和 CAN bus 按实际硬件配置修改，空值由 Robot Profile 使用默认值

该入口只启动 SerialArm-Core ros2_control、`joint_state_broadcaster`、`joint_trajectory_controller`、MoveIt2 `move_group`、一个只读机器人位姿 TF 桥接节点以及可选的 Orbbec 腕部相机；不启动原 `handeye_bridge`、EEF、采摘任务、普通运动能力服务器、Planning Scene 点云预处理或 RViz

相机默认由本入口启动，若已由其他系统启动同一设备，则加 `start_wrist_camera:=false`，不要同时启动两个 Orbbec 驱动；如果关闭则仍必须提供与标定一致的 RGB、已对齐 Depth 和 CameraInfo，且两种图像的 `frame_id` / 分辨率 / 时间戳要满足 App 的校验要求

如果不需要 GUI 相机，仍可以只检查机械臂规划：

```bash
ros2 launch tomato_picker_bringup handeye_servo.launch.py start_wrist_camera:=false
```

### 2. 运动 Action、位姿和 TCP

| 用途 | 接口或参数 | 含义 |
| --- | --- | --- |
| MoveIt2 Action | `/move_action` | `moveit_msgs/action/MoveGroup`，不是自定义 MoveArm Action |
| 运动规划组 | `handeye_tip` | 6 轴 `base_link -> tip` 的独立运动学组，不影响正常 `arm` 组 |
| 伺服 TCP | `tip` | URDF 中 `tool0 -> tip` 的固定偏移，需按真实针尖实测 |
| 机器人末端姿态 | `/arm/pose` | `geometry_msgs/msg/PoseStamped`，Frame 为 `base_link`，姿态对象为 `tool0` |
| 机器人末端变换 | `/arm/tool_transform` | `geometry_msgs/msg/TransformStamped`，`base_link -> tool0` |
| 机器人关节 | `/joint_states` | ros2_control 的真实反馈，供 robot_state_publisher 与 MoveIt2 使用 |
| 彩色相机 | `/camera/wrist/color/image_raw` | 以实际 ROS2 topic list 为准 |
| 深度相机 | `/camera/wrist/depth/image_raw` | 须确认实际消息已对齐到彩色光学 Frame |
| 相机内参 | `/camera/wrist/color/camera_info` | 必须与彩色帧内参和尺寸匹配 |

当前 URDF `tool0 -> tip` 固定变换为 `xyz=0 0 0.101`，它只是模型配置，**不能代替对真实标定指针尖端的物理测量**；未确认针尖坐标之前，不得执行向标定板接近的运动

普通采摘 `arm` 组仍止于 `tool0`；独立 `handeye_tip` 组止于 `tip`，且在 `kinematics.yaml` 中具有自己的 KDL IK 配置，从而让 App 可以向实际工具 Link 发送位置和姿态约束，不需要修改 App 算法

只读姿态桥接从 robot_state_publisher 的 TF 读取 `base_link -> tool0`，保留关节状态派生的时间戳；超过 250 ms 的姿态不发布，避免将旧关节位姿作为新采样；相机时间戳和机器人时间戳需使用可比较的 ROS 时间基准，停稳采样不等于运动中的硬件曝光同步

### 3. 先做无运动检查

启动后另开终端并 `source install/setup.bash`：

```bash
ros2 control list_controllers
ros2 action list -t | grep /move_action
ros2 topic hz /arm/pose
ros2 run tf2_ros tf2_echo base_link tip
```

确认关节控制器均为 `active`、MoveGroup Action 存在、真实 TF 持续刷新后进行 **只规划、不执行** 的验证：

```bash
ros2 run tomato_picker_bringup check_handeye_servo
```

默认规划当前 `tip` 位姿，不会执行；通过后可测试附近 5mm 偏移的 IK 和规划（仍不执行运动）：

```bash
ros2 run tomato_picker_bringup check_handeye_servo --offset-x 0.005
```

检查失败时不要直接在 GUI 点击开始运动，应先确认控制器、TF、`handeye_tip` IK 求解器、MoveGroup Action 和碰撞设置

### 4. 在 Handeye-Calibration-App 配置

在 App 中使用已经完成的 **同一份手眼求解结果**，冻结矩阵并导入包含 `tip` 的真实 Description：

```text
ROS input type: PoseStamped
Input topic:    /arm/pose
Robot base:     base_link
Robot end:      tool0
Motion Action:  /move_action
Planning group: handeye_tip
Target link:    tip
```

手眼矩阵的末端 Frame 必须是 `tool0`，相机 Frame 必须与 RGB、对齐深度图一致；在 GUI 中搜索当前正在发布的三路相机话题，不要以预设名称替代真实 `frame_id` 校验

先用计划检查确认配置，再选择稳定、无遮挡的目标点；第一次移动选择安全空间中的静态目标，使用较小步长和高于最低要求的安全距离；机器人周围需要无障碍物或已有可信 Planning Scene，App 的 TCP 安全距离 **不能替代整个机械臂的碰撞检测**

App 运行的是“观察 → 规划 → 小步运动 → 停稳 → 重新观察”的离散闭环；不是持续速度伺服；未验证真机 TCP、跟踪稳定性、控制器取消及急停之前不得用它接触或撞击标定板

此模式仅解决手眼定位验证，**不依赖 Tomato-Perception**，也不改变正常的 `bringup.launch.py` 和采摘接口

## 采摘阶段

默认阶段机：

```text
PRE_PICK
↓
APPROACH
↓
PICK
↓
EEF_CLOSE
↓
RETREAT
↓
PLACE       optional
↓
EEF_OPEN    optional
↓
HOME        optional
```

`pre_pick_pose`、`approach_pose` 与 `retreat_pose` 都可以显式指定

未显式指定时，Task 会沿最终采摘位姿的 **局部 TCP Z 轴** 自动生成：

```text
pre_pick_distance = 0.15 m
approach_distance = 0.08 m
retreat_distance  = 0.10 m
```

详细字段与阶段反馈见 [API.md](API.md)

## 常用请求

### MoveArm HOME

首次真机测试建议先只规划：

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

### EEF

```bash
# OPEN
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 0, value: 0.0}"

# SET_POSITION, normalized [0, 1] -> TODO:新夹爪应修正为 [0, 0.91]
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 3, value: 0.25}"
```

### PickTarget

目标位姿必须替换为已经验证的安全值：

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.40}, orientation: {w: 1.0}}}, pre_pick_distance: 0.15, approach_distance: 0.08, retreat_distance: 0.10, use_eef: true, go_home_after_finish: false}" \
  --feedback
```

完整 Service / Action Reference 见 [API.md](API.md)

## 腕部相机与框选 GUI

GUI 是暂未接入视觉模型时的人工目标入口，仅保留一台腕部 RGB-D 相机

### Gemini 335L

可选安装官方 Orbbec ROS 2 driver：

```bash
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
```

使用一个 bringup 同时启动相机与 GUI：

```bash
python3 -m pip install PySide6

ros2 launch tomato_picker_bringup bringup.launch.py \
  start_wrist_camera:=true \
  start_gui:=true \
  start_perception:=false
```

默认相机 topics：

```text
/camera/wrist/color/image_raw
/camera/wrist/depth/image_raw
/camera/wrist/depth/camera_info
```

GUI 要求 depth 已对齐到 color；`start_wrist_camera:=true` 会以 `depth_registration:=true` 启动 Gemini 330 系列 driver

如果当前 Orbbec TF tree 的 `camera_link` 尚未连接到机械臂 URDF 中的 `camera` frame，并且两者已经确认使用同一物理原点，可额外启用：

```bash
publish_wrist_camera_mount_tf:=true
```

已有手眼标定 TF 时不要重复发布该默认连接

### GUI 操作

1. 左键在腕部 RGB 图像上依次添加多边形 ROI 顶点
2. 右键或 `清除 ROI` 清空当前选择
3. `确认框选目标`：ROI → 稳健深度 → 3D 反投影 → TF 到 `base_link`
4. 目标发布到 `/tomato_picker/gui/selected_target`
5. GUI 可直接发送 `/tomato_picker/task/pick`

当前 GUI 只从 ROI 获取目标 **位置**；目标姿态沿用框选时 `tool0` 的当前姿态，不做视觉姿态识别

后续接入检测 / 分割模型时，只需要替换目标产生方式，不需要修改 Motion / EEF / PickTask 接口

## 配置

| 模块 | 配置文件 |
| --- | --- |
| EEF | `src/tomato_picker/eef/config/damiao_eef.yaml` |
| Motion | `src/tomato_picker/motion/config/motion.yaml` |
| Perception | `src/tomato_picker/perception/config/perception.yaml` |
| Pick Task | `src/tomato_picker/task/config/task.yaml` |
| Wrist GUI | `src/tomato_picker/gui/config/gui.yaml` |

机械臂 Core、Hardware、URDF、Controllers、MoveIt 与阻抗模式由 SerialArm Robot Profile 管理，不在 Tomato-Picker 重复配置

## 文档

- [API.md](API.md)：Service / Action / Message API Reference
- [SerialArm-Core](https://github.com/Kaede-Rei/SerialArm-Core)：机械臂平台层
- [Tomato-Picker-PiPER](https://github.com/Kaede-Rei/Tomato-Picker-PiPER)：旧 ROS 1 实验系统，仅作为历史设计参考

## License

以仓库当前 [LICENSE](LICENSE) 为准
