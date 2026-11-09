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

在 Tomato-Manipulation 中执行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select tomato_picker_interfaces tomato_picker_mujoco \
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
ros2 launch tomato_picker_mujoco harvest_mujoco.launch.py
```

默认隐藏 MuJoCo 原生窗口并显示 RViz；添加 `headless:=false` 可同时打开物理引擎窗口，添加 `rviz:=false` 可关闭 RViz

所有消费者使用 `use_sim_time=true`，时间来源为 `/clock`；新工程已启用 Studio 的通用仿真时钟选项

关节规划时间预算为 15 秒；规划阶段机械臂保持静止，可在运行台查看规划与执行反馈

## 状态与接口

`/simulation/physics/state` 发布实际果实位姿、速度、双指接触力、持握对象和入篮数量

`/tomato_picker/eef/command` 接受 OPEN / CLOSE / STOP；STOP 在保持实测位置后等待速度降到阈值才确认

`/simulation/harvest` 保持原 ManageHarvest 接口；其 RESET / SELECT_NEXT / GRASP / RELEASE 转交物理插件，释放请求等待真实入篮结果

`/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` 为 MuJoCo 相机输出

## 重新生成模型

运行时直接使用 `mjcf/tomato_picker.xml` 和已生成的网格，不需要安装网格处理工具

离线修改 CAD 或场景后，可在独立 Python 环境安装脚本列出的生成依赖，再执行 `scripts/generate_mjcf.py --help` 查看参数；生成后重新编译安装本包

当前物理参数用于本项目仿真，接触摩擦、执行器增益与果实质量可在生成脚本中调整
