# Tomato-Picker-ROS2 API Reference

本文档面向通过 ROS 2 调用 Tomato-Picker 能力与任务接口的开发者

公共接口按“用途 → 字段 → 返回值 / 反馈 → 示例 → 注意”组织

## 1. 接口总览

| 能力 | 接口 | 类型 |
| --- | --- | --- |
| 机械臂运动 | `/tomato_picker/motion/move_arm` | `tomato_picker_interfaces/action/MoveArm` |
| EEF 命令 | `/tomato_picker/eef/command` | `tomato_picker_interfaces/srv/CommandEef` |
| EEF READY | `/tomato_picker/eef/ready` | `std_srvs/srv/Trigger` |
| 单目标采摘 | `/tomato_picker/task/pick` | `tomato_picker_interfaces/action/PickTarget` |
| Planning Scene | `/tomato_picker/perception/set_scene_enabled` | `tomato_picker_interfaces/srv/SetSceneEnabled` |
| GUI 目标 | `/tomato_picker/gui/selected_target` | `tomato_picker_interfaces/msg/TargetObject` |
| 手眼标定位姿 | `/arm/pose` | `geometry_msgs/msg/PoseStamped` |
| 手眼 FAULT 清除 | `/handeye/fault/clear` | `std_srvs/srv/Trigger` |
| 手眼重新拖拽 | `/handeye/drag/enable` | `std_srvs/srv/Trigger` |

`bringup.launch.py` 按以下顺序提供这些接口

```text
SerialArm READY
→ EEF READY
→ MoveArm READY
→ PickTarget / Perception / optional GUI
```

---

## 2. `MoveArm` Action

```text
/tomato_picker/motion/move_arm
tomato_picker_interfaces/action/MoveArm
```

将 Tomato-Picker 上层运动请求封装到已运行的 MoveIt 2 `move_group`

### 2.1. 命令

| `command_type` | 名称 | 作用 |
| --- | --- | --- |
| `0` | `HOME` | MoveIt named target `home` |
| `1` | `JOINT` | 关节空间目标 |
| `2` | `POSE` | 末端位姿目标 |
| `3` | `LINE` | 笛卡尔直线目标 |

### 2.2. Goal

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command_type` | `uint8` | `HOME / JOINT / POSE / LINE` |
| `target_pose` | `geometry_msgs/PoseStamped` | `POSE / LINE` 使用 |
| `joint_names` | `string[]` | `JOINT` 可选关节名 |
| `joints` | `float64[]` | 关节目标，rad |
| `velocity_scale` | `float64` | MoveIt 速度缩放 |
| `acceleration_scale` | `float64` | MoveIt 加速度缩放 |
| `execute` | `bool` | `false` 只规划，`true` 规划并执行 |

`velocity_scale <= 0` 或 `acceleration_scale <= 0` 时使用 `motion.yaml` 默认值；有效请求限制到 `[0.01, 1.0]`

### 2.3. Result

| 字段 | 说明 |
| --- | --- |
| `success` | 是否成功 |
| `error_code` | 错误码 |
| `message` | 结果文本 |
| `final_pose` | 结束时末端位姿 |
| `final_joints` | 结束时关节位置 |

错误码

| 值 | 说明 |
| --- | --- |
| `0` | 成功 |
| `1` | Goal 非法 |
| `2` | 规划失败 |
| `3` | 执行失败 |
| `4` | 已取消 |

### 2.4. Feedback

```text
string stage
float32 progress
geometry_msgs/PoseStamped current_pose
```

常见阶段

```text
SET_TARGET
CARTESIAN_PLAN
PLANNING
EXECUTING
DONE
```

### 2.5. HOME

只规划

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

执行

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: true}" \
  --feedback
```

### 2.6. JOINT

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 1, joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

`joint_names` 为空时，`joints` 数量必须等于规划组全部关节数量；提供 `joint_names` 时可只修改部分关节

### 2.7. POSE

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 2, target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

### 2.8. LINE

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 3, target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

`LINE` 的 `target_pose.header.frame_id` 必须与 MoveGroup 当前 pose reference frame 一致

---

## 3. `CommandEef` Service

```text
/tomato_picker/eef/command
tomato_picker_interfaces/srv/CommandEef
```

### 3.1. 命令

| `command` | 名称 | `value` | 说明 |
| --- | --- | --- | --- |
| `0` | `OPEN` | 忽略 | 打开 EEF |
| `1` | `CLOSE` | 忽略 | 关闭 EEF |
| `2` | `STOP` | 忽略 | 停止当前 EEF 动作 |
| `3` | `SET_POSITION` | `[0, 1]` | 归一化位置 |

`SET_POSITION`

```text
0.0 = open_position
1.0 = closed_position
```

### 3.2. Response

```text
bool success
int32 error_code
string message
```

| 错误码 | 说明 |
| --- | --- |
| `0` | 命令已接受 |
| `1` | 不支持的命令 |
| `2` | 位置超出 `[0, 1]` |
| `3` | EEF 未激活 |
| `4` | Worker busy |
| `5` | EEF 正在 homing |

`success=true` 表示 Worker 已接受命令，不表示机械动作已在 Service 返回前完成

### 3.3. 示例

```bash
# OPEN
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 0, value: 0.0}"

# CLOSE
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 1, value: 0.0}"

# STOP
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 2, value: 0.0}"

# SET_POSITION
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 3, value: 0.25}"
```

---

## 4. EEF READY Service

```text
/tomato_picker/eef/ready
std_srvs/srv/Trigger
```

```bash
ros2 service call \
  /tomato_picker/eef/ready \
  std_srvs/srv/Trigger \
  "{}"
```

| `success` | `message` | 说明 |
| --- | --- | --- |
| `true` | `READY` | homing 完成，可接受位置命令 |
| `false` | `HOMING` | 正在 homing |
| `false` | `UNCONFIGURED` | 未配置 |
| `false` | `INACTIVE` | 未激活 |
| `false` | `ERROR` | EEF 错误 |
| `false` | `SHUTDOWN` | 已关闭 |

`eef_mode:=mock` 提供相同接口

---

## 5. `PickTarget` Action

```text
/tomato_picker/task/pick
tomato_picker_interfaces/action/PickTarget
```

单目标采摘阶段机只依赖 `MoveArm` 与 `CommandEef`，不直接访问 SerialArm-Core、controller_manager 或 CAN

### 5.1. 阶段机

```text
PRE_PICK
↓
APPROACH
↓
PICK
↓
EEF_CLOSE       use_eef=true
↓
RETREAT
↓
PLACE           use_place_pose=true
↓
EEF_OPEN        use_eef=true && use_place_pose=true
↓
HOME            go_home_after_finish=true
```

### 5.2. Goal

| 字段 | 说明 |
| --- | --- |
| `target_pose` | 最终采摘位姿，`frame_id` 必填 |
| `use_pre_pick_pose` | 是否使用显式 `pre_pick_pose` |
| `pre_pick_pose` | 预采摘位姿 |
| `use_approach_pose` | 是否使用显式 `approach_pose` |
| `approach_pose` | 接近位姿 |
| `use_retreat_pose` | 是否使用显式 `retreat_pose` |
| `retreat_pose` | 退出位姿 |
| `use_place_pose` | 是否执行放置阶段 |
| `place_pose` | 放置位姿 |
| `use_eef` | 是否执行 EEF CLOSE / OPEN |
| `go_home_after_finish` | 完成后是否 HOME |
| `pre_pick_distance` | 自动预采摘距离 m |
| `approach_distance` | 自动接近距离 m |
| `retreat_distance` | 自动退出距离 m |
| `retry_times` | 单个 Pose 步骤额外重试次数 |

未提供显式位姿时，三个位姿都从 `target_pose` 沿其 **局部 Z 轴** 自动生成

默认值来自 `task.yaml`

```text
pre_pick_distance = 0.15 m
approach_distance = 0.08 m
retreat_distance  = 0.10 m
motion_velocity_scale = 0.15
motion_acceleration_scale = 0.15
```

### 5.3. Feedback

| `current_stage` | 常量 | 文本 |
| --- | --- | --- |
| `1` | `STAGE_PRE_PICK` | `PRE_PICK` |
| `2` | `STAGE_APPROACH` | `APPROACH` |
| `3` | `STAGE_PICK` | `PICK` |
| `4` | `STAGE_EEF_CLOSE` | `EEF_CLOSE` |
| `5` | `STAGE_RETREAT` | `RETREAT` |
| `6` | `STAGE_PLACE` | `PLACE` |
| `7` | `STAGE_EEF_OPEN` | `EEF_OPEN` |
| `8` | `STAGE_HOME` | `HOME` |

其余反馈字段

```text
uint32 completed_steps
uint32 total_steps
string stage_text
```

### 5.4. Result

```text
bool success
int32 error_code
string message
uint32 completed_steps
bool canceled
geometry_msgs/PoseStamped final_pose
```

| 错误码 | 说明 |
| --- | --- |
| `0` | 成功 |
| `1` | Motion / EEF dependency 不可用 |
| `2` | Motion 步骤失败 |
| `3` | EEF 步骤失败 |
| `4` | 已取消 |

### 5.5. 最小请求

自动生成 PRE_PICK / APPROACH / RETREAT

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.40}, orientation: {w: 1.0}}}, pre_pick_distance: 0.15, approach_distance: 0.08, retreat_distance: 0.10, use_eef: true, go_home_after_finish: false}" \
  --feedback
```

### 5.6. 显式 PRE_PICK / APPROACH / RETREAT

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}, use_pre_pick_pose: true, pre_pick_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}, use_approach_pose: true, approach_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.33, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}, use_retreat_pose: true, retreat_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}, use_eef: true, go_home_after_finish: false}" \
  --feedback
```

### 5.7. PLACE + HOME

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.30}, orientation: {w: 1.0}}}, use_eef: true, use_place_pose: true, place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.25, z: 0.30}, orientation: {w: 1.0}}}, go_home_after_finish: true}" \
  --feedback
```

---

## 6. Planning Scene Service

```text
/tomato_picker/perception/set_scene_enabled
tomato_picker_interfaces/srv/SetSceneEnabled
```

Request

```text
bool enabled
bool clear_octomap
```

Response

```text
bool success
string message
```

关闭并清空

```bash
ros2 service call \
  /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: false, clear_octomap: true}"
```

重新开启

```bash
ros2 service call \
  /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: true, clear_octomap: false}"
```

---

## 7. `TargetObject` Message

```text
tomato_picker_interfaces/msg/TargetObject
```

```text
string id
string class_name
float32 confidence
geometry_msgs/PoseStamped pose
```

当前腕部 GUI 发布

```text
/tomato_picker/gui/selected_target
```

GUI 默认值

```text
id         = gui_target
class_name = manual_roi
confidence = 1.0
pose       = ROI 深度反投影并 TF 到 base_link 后的目标
```

查看

```bash
ros2 topic echo /tomato_picker/gui/selected_target
```

当前 GUI 的目标姿态使用框选时 `tool0` 的实际姿态；GUI 只通过 ROI 与深度估计目标位置，不做视觉姿态识别

---

## 8. Wrist GUI

启动

```bash
ros2 launch tomato_picker_bringup bringup.launch.py \
  start_gui:=true
```

或单独启动

```bash
ros2 run tomato_picker_gui wrist_target_gui
```

默认订阅

```text
/camera/wrist/color/image_raw
/camera/wrist/depth/image_raw
/camera/wrist/depth/camera_info
```

处理链

```text
polygon ROI
→ mask morphology
→ skeleton / centroid target pixel
→ robust ROI depth
→ camera intrinsics back-projection
→ TF to base_link
→ TargetObject
→ optional PickTarget
```

GUI 要求 depth 已对齐到 color 且 TF 能从图像 `frame_id` 转换到 `base_link`

---

## 9. Bringup Runtime Hardware Override

最新 SerialArm-Core 支持从 ROS 2 launch 覆盖

```text
serial_port
baudrate
bus
```

Tomato 顶层入口原样透传，并将同一组覆盖值传给 EEF 共享 CAN runtime

示例

```bash
ros2 launch tomato_picker_bringup bringup.launch.py \
  robot_profile:=dm_arm_gray \
  serial_port:=/dev/ttyACM1 \
  baudrate:=921600 \
  bus:=main_can
```

未提供覆盖值时，ARM 与 EEF 分别使用各自配置文件中的默认连接参数


---

## 10. Hand-eye Calibration Bringup

专用入口

```bash
ros2 launch tomato_picker_bringup handeye.launch.py
```

该入口由 `handeye_bridge` 直接持有 SerialArm-Core `RobotSession`，不会同时启动 ros2_control / MoveIt / EEF / perception / task

### 10.1 Pose Topic

```text
/arm/pose
geometry_msgs/msg/PoseStamped
```

`header.frame_id` 使用当前 Robot Profile 的 `dynamics.base_frame`，`pose` 为 SerialArm-Core `snapshot.dynamics.tool_pose` 对应的 `base_frame -> tool_frame` 位姿

只有以下条件同时成立才发布

```text
RobotState.ACTIVE
snapshot.valid == true
snapshot.last_error == empty
```

关节反馈 `/arm/joint_states` 使用 `sensor_msgs/msg/JointState`，包含 Profile 定义的 `joint1`～`joint6` 的 `position`（rad）、`velocity`（rad/s）和 `effort`（Nm）；和 `/arm/pose`、`/arm/tool_transform` 由同一 RobotSession 快照生成，三者 Header 使用相同的 ROS 发布时间戳（非电机原始采样时刻）；可通过 `joint_state_topic` 启动参数修改话题名称

### 10.2 Online FAULT Recovery

```text
/handeye/fault/clear
/handeye/fault/compliant_recovery
/handeye/fault/rigid_hold
/handeye/drag/enable
```

全部使用

```text
std_srvs/srv/Trigger
```

普通可恢复 FAULT

```bash
ros2 service call /handeye/fault/clear std_srvs/srv/Trigger "{}"
```

成功后 Robot 保持 `ACTIVE + RIGID_HOLD`；等 `/arm/pose` 恢复为新的有效样本后，由操作员显式恢复拖拽

```bash
ros2 service call /handeye/drag/enable std_srvs/srv/Trigger "{}"
```

如果 `clear_fault` 返回失败，bridge 不自动重试，Core fault hold 继续保持；应根据返回的 SerialArm-Core 错误判断是否等待后重试或停止标定

`/handeye/fault/compliant_recovery` 仅包装 Core 的受限 FAULT 柔性恢复能力；不允许的 fault 类型或不满足 Safety 条件时会直接返回失败


## 手眼 RGB-D 闭环接口

整机运行由 AT-System-Integration 读取 `../../Bringup/src/robot-profile` 并准备各子系统节点
机械臂系统不依赖视觉包或融合启动包

| 能力 | 名称 | 类型 / 说明 |
| --- | --- | --- |
| 标准运动规划 | `/move_action` | `moveit_msgs/action/MoveGroup` |
| 工作流使用的运动接口 | `/tomato_picker/servo/move_arm` | `tomato_picker_interfaces/action/MoveArm`，由工程启动参数指定名称 |
| 规划组 | `handeye_tip` | `base_link` 到 `tip` 的六轴组 |
| 工具位姿 | `/tf`、`/tf_static` | 按观测时间查询 `base_link -> tool0` |
| 关节反馈 | `/joint_states` | 用于新鲜度与静止判定 |

手眼矩阵保存在机器人工程的 `assets/handeye.yaml`
`/arm/pose` 与 `/arm/tool_transform` 仍由独立手眼标定入口提供，整机流程直接使用 TF
`tool0 -> tip` 固定偏移以真实 TCP 测量值为准
