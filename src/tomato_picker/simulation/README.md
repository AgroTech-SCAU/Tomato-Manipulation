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

连续路径通过 Robot 自有 `MoveArmSequence` Action 先完成整段规划与密集碰撞核对，再执行融合后的轨迹；原 `MoveArm` 接口保持兼容，序列能力仅在 MuJoCo 配置中启用

Pilz PTP 按请求缩放关节速度与加速度上限；当前携果转向加速度缩放 0.75，对应 3 rad/s²，连续空返 PTP 缩放 0.65，对应 2.6 rad/s²

Pilz LIN 按请求缩放笛卡尔上限；当前平移加速度上限为 2.25 m/s²，携果撤离缩放 0.15、下降缩放 0.25、空返抬升缩放 0.35，分别对应 0.3375、0.5625 与 0.7875 m/s²；IK 后仍检查各轴 4 rad/s² 的绝对上限，LIN 的 factor 不能直接解释成各轴 `4 × factor` 的加速度

各段动态限制可在 Robot 任务 `mujoco_sim_harvest_continuous_goal` 的 `carry_limits` 和 `return_limits` 内分别配置，不修改硬件默认参数

撤离目标可用 `min_retreat_height_m` 声明基座坐标系 Z 轴的最低通行高度，本 MuJoCo 工程为 0.27 m；已经高于该高度的目标保持不变，低位果实在原撤离 LINE 中同时抬升，保留 XY 和夹爪方向；这样第三果先离开底座附近的低位区域，再转向侧篮，不增加新的等待状态，整条路径继续检查机器人、附着果实和环境碰撞

MuJoCo 按中果 → 上果 → 下果采摘，物理果实编号为 2 → 1 → 3，按采摘顺序放入第 1 → 2 → 3 个篮槽；仅改变选择顺序与果实到篮槽的映射，三果世界位置、篮位置与物理参数保持原配置

主流程以果实中心为目标连续视觉接近，单次直线最多 150 mm、累计最多 300 mm；保留目标关联与观测新鲜度检查，每次运动后实际停稳，夹取前需三个时间戳递增的新帧确认误差不超过 0.5 mm

物理时钟保持 1×，夹持接触与入篮停稳条件保持原配置

本机子系统已就绪、MuJoCo 原生窗口和 1× 物理时间下完整运行两轮，全零出发 → 三果实际入篮 → 全零回收的流程计时为 26.4–28.5 秒；从开始运行请求到结束为 26.9–29.1 秒，本轮冷启动子系统另约 44 秒

MuJoCo 运动节点通过 `harvest_state_topic` 监测实际掉果，发现掉果即停止当前轨迹并返回失败；`harvest_recovery_joints` 声明允许的空载回零和观察位，使故障后重新运行仍可到达观察位并重置场景，其他运动在重置前保持阻止

融合路径只在 Action 终点用新鲜关节反馈连续 0.12 秒低速确认停止；已完成停止核对后不重复等待；末段位置收敛仍需三帧时间戳递增的观测，最后一果真实入篮后，将抬升、观察位路点与全零收纳位合成同一条完整预规划轨迹，仅在最终全零位确认实际停稳后完成，跳过中间停止与无剩果的选择准备

## 状态与接口

`/simulation/physics/state` 发布实际果实位姿、速度、双指接触力、持握对象和入篮数量

`/tomato_picker/eef/command` 接受 OPEN / CLOSE / STOP；已成功完成 CLOSE、持果且夹指静止时，STOP 保留原闭合目标的夹持预紧；运动中、未持果或最近执行其他指令时保持实测位置，两种情况均等待实测速度持续低于阈值才确认停止

`/simulation/harvest` 保持原 ManageHarvest 接口；其 RESET / SELECT_NEXT / GRASP / RELEASE 转交物理插件，释放请求等待真实入篮结果

`/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` 为 MuJoCo 相机输出

## 重新生成模型

运行时直接使用 `mjcf/tomato_picker.xml` 和已生成的网格，不需要安装网格处理工具

离线修改 CAD 或场景后，可在独立 Python 环境安装脚本列出的生成依赖，再执行 `scripts/generate_mjcf.py --help` 查看参数；生成后重新编译安装本包

当前物理参数用于本项目仿真，接触摩擦、执行器增益与果实质量可在生成脚本中调整
