# MuJoCo 物理采摘

此包用 `mujoco_ros2_control/MujocoSystemInterface` 驱动六轴机械臂与双指夹爪，MoveIt 和原有采摘任务接口保持一致

- 关节反馈来自 MuJoCo 积分状态，夹爪由一个直线关节驱动，另一指与齿轮通过约束联动
- MuJoCo 专用 TCP 位于夹持长条内，`tool0` 下坐标为 `[-0.0039, 0.0083, 0.066] m`，相机固定 TF 与渲染外参使用同一标定文件
- 果实带质量和自由关节，初始通过果梗约束悬挂；双指持续接触后解除果梗，持握靠接触摩擦
- 张开后果实受重力下落，进入篮内、接触篮底并停稳才增加采摘数量
- MoveIt 同步所有物理果实的碰撞球，夹持后同步为附着碰撞物；RESET / SELECT / GRASP / RELEASE 等待对应场景版本应用成功后才返回
- 底座固定部件、机械臂、夹指、篮底与四壁均参与碰撞，CAD 视觉网格与凸分解碰撞网格分开
- 固定底座碰撞网格使用 0.21 mm 几何容差，处理 CAD 微小重叠；超过容差仍产生碰撞力，摩擦保留
- `mujoco_target_server` 将物理果实位置转换为相机坐标，供现有闭环任务使用；此入口使用 ground truth 观测
- 相机插件另发 RGB、深度和内参，可供后续图像检测接入；本入口不运行 YOLO

## 编译

本机已使用 MuJoCo 3.12.0 验证；负 margin 的接触模型需 MuJoCo 3.12 或更新版本，语义见 [官方接触文档](https://mujoco.readthedocs.io/en/3.12.0/computation/index.html#contact)

需要 ROS 2 Humble、`mujoco_ros2_control`、`mujoco_ros2_control_plugins`、`mujoco_vendor` 以及原项目的 MoveIt / ros2_control 依赖

本机原生窗口退出还需要 `mujoco_ros2_control 0.1.2` 的线程与启动取消修复；从源码重装运行时时，先按 [本地 MuJoCo 运行时修复](../../../patches/README.md) 安装补丁，再编译本包

在 Tomato-Manipulation 中执行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select tomato_picker_interfaces tomato_picker_motion tomato_picker_bringup tomato_picker_mujoco \
  --cmake-args -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

在 Tomato-Perception 中执行

```bash
source /opt/ros/humble/setup.bash
source ../Tomato-Manipulation/install/setup.bash
source install/setup.bash
colcon build --packages-select vision_target_server
```

## 启动

先停止同一 ROS 域中原来的仿真节点，避免两个控制器同时发布反馈

通过 Studio 使用 `Bringup/src/robot-profile-mujoco`，运行其中的连续采摘流程；Studio 会按依赖启动所有节点，无需同时执行下列 launch

单独调试物理模型时，在 Tomato-Manipulation 中执行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source ../Tomato-Perception/install/setup.bash
ros2 launch tomato_picker_mujoco harvest_mujoco.launch.py headless:=false rviz:=false
```

该命令打开 MuJoCo 原生窗口并关闭 RViz；点击三维画面后，按 `Tab` 显示左侧菜单，按 `Shift+Tab` 显示右侧菜单，按 `F1` 查看操作帮助

所有消费者使用 `use_sim_time=true`，时间来源为 `/clock`；新工程已启用 Studio 的通用仿真时钟选项

关节运动先尝试同步各轴的 Pilz PTP 直接轨迹，使用完整场景快照按关节空间最大范围的 0.001 细分各段，核对机器人、附着果实与世界碰撞；直接轨迹不可用时回到 OMPL 规划，保留 15 秒搜索上限与 4 路并行搜索，找到首条有效路径后结束搜索

OMPL 关闭多路径混合与耗尽剩余搜索预算的路径简化；运动节点对后备关节路径额外尝试最多 0.25 秒的几何捷径，每个新增段仍按相同 0.001 分辨率核对完整场景，保留原有不可直连段，用 TOTG 按原速度与加速度限制平滑并每 0.02 秒重采样，再对最终轨迹密集核对碰撞与关节边界，仅在安全且至少节省 0.02 秒时替换；直线接近和放置沿用原笛卡尔路径

运动节点日志分别记录规划耗时、轨迹时长、轨迹点数与实际执行耗时，用于区分搜索等待、轨迹运动和任务调度的耗时

本 MuJoCo 工程的空载动作速度与加速度缩放均为 0.75，六轴加速度上限为 4 rad/s²，因此有效加速度为 3 rad/s²；携果撤离、转向和放置采用速度缩放 0.5、加速度缩放 0.25，有效加速度为 1 rad/s²；这些设置只用于 MuJoCo，不修改实际机械臂或其他仿真入口的参数

撤离目标可用 `min_retreat_height_m` 声明基座坐标系 Z 轴的最低通行高度，本 MuJoCo 工程为 0.27 m；已经高于该高度的目标保持不变，低位果实在原撤离 LINE 中同时抬升，保留 XY 和夹爪方向；这样第三果先离开底座附近的低位区域，再转向侧篮，不增加新的等待状态，整条路径继续检查机器人、附着果实和环境碰撞

物理时钟保持 1×，夹持接触、入篮停稳和末段视觉收敛条件保持原配置

本机本次在子系统已就绪、MuJoCo 原生窗口和 1× 物理时间下运行两轮，完整全零出发 → 三果实际入篮 → 全零回收流程为 45.1–45.9 秒；从开始运行请求到结束为 46.9–47.6 秒，冷启动子系统另约 37 秒

MuJoCo 运动节点通过 `harvest_state_topic` 监测实际掉果，发现掉果即停止当前轨迹并返回失败；`harvest_recovery_joints` 声明允许的空载回零和观察位，使故障后重新运行仍可到达观察位并重置场景，其他运动在重置前保持阻止

每次运动后用新鲜关节反馈连续 0.12 秒低速确认停止；已完成停止核对后不重复等待；末段位置收敛仍需三帧时间戳递增的观测，最后一果真实入篮并抬升后，经观察位路点回全零收纳位，跳过无剩果的选择与阶段准备，再确认停稳

## 状态与接口

`/simulation/physics/state` 发布实际果实位姿、速度、双指接触力、持握对象和入篮数量

`/tomato_picker/eef/command` 接受 OPEN / CLOSE / STOP；STOP 在保持实测位置后等待速度降到阈值才确认

`/simulation/harvest` 保持原 ManageHarvest 接口；其 RESET / SELECT_NEXT / GRASP / RELEASE 转交物理插件，释放请求等待真实入篮结果

`/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` 为 MuJoCo 相机输出

## 重新生成模型

运行时直接使用 `mjcf/tomato_picker.xml` 和已生成的网格，不需要安装网格处理工具

离线修改 CAD 或场景后，可在独立 Python 环境安装脚本列出的生成依赖，再执行 `scripts/generate_mjcf.py --help` 查看参数；生成后重新编译安装本包

当前物理参数用于本项目仿真，接触摩擦、执行器增益与果实质量可在生成脚本中调整
