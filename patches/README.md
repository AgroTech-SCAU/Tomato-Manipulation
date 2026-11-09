# MuJoCo 原生窗口关闭修复

`mujoco_ros2_control-0.1.2-ui-thread-cleanup.patch` 对应 [上游 0.1.2](https://github.com/ros-controls/mujoco_ros2_control/tree/0.1.2)，在渲染线程退出前释放窗口适配器，使 `mjr_freeContext`、OpenGL context 解绑和 GLFW 窗口销毁都发生在创建窗口的线程

原版把适配器留到 controller manager 线程析构，在本机 Linux / MuJoCo 3.12.0 上，关闭原生窗口会在 `GlfwAdapter::~GlfwAdapter` → `mjr_freeContext` → `libGLdispatch` 崩溃；移除相机插件仍可复现

`mujoco_ros2_control-0.1.2-shutdown-clock.patch` 处理刚启动即停止的另一处退出竞态：首次 `wait_until_started` 被 shutdown 唤醒后仍调用 `sleep_for`，会因 ROS context 已失效而抛出异常并退出 `-6`；此补丁只在 context 确认失效时结束线程，context 仍有效的异常继续抛出

`mujoco_ros2_control-0.1.2-startup-load-cancel.patch` 处理初次模型加载时停止的死锁：UI 已退出后，物理线程仍等待加载确认，主线程则等待物理线程退出；补丁保留加载请求、确认及模型所有权，仅在退出请求到来时取消等待，并在 UI 退出后唤醒加载线程

## 安装到当前工作区

先停止该工作区的仿真进程，在 Tomato-Manipulation 根目录执行以下命令；只重建 `mujoco_ros2_control`，使用现有 isolated `install/`，保留同级已安装包

```bash
overlay_install="$PWD/install"
overlay_patch_dir="$PWD/patches"
overlay_tmp="$(mktemp -d /tmp/tomato-mujoco-overlay-XXXXXX)"
trap 'rm -rf "$overlay_tmp"' EXIT

git clone --depth 1 --branch 0.1.2 \
  https://github.com/ros-controls/mujoco_ros2_control.git "$overlay_tmp/source"
git -C "$overlay_tmp/source" apply \
  "$overlay_patch_dir/mujoco_ros2_control-0.1.2-ui-thread-cleanup.patch" \
  "$overlay_patch_dir/mujoco_ros2_control-0.1.2-shutdown-clock.patch" \
  "$overlay_patch_dir/mujoco_ros2_control-0.1.2-startup-load-cancel.patch"

# 复用 mujoco_vendor 已安装的 lodepng archive
# 上游 CMake 使用 lodepng，本机 vendor 导出 lodepng::lodepng
mkdir "$overlay_tmp/lodepng"
cat > "$overlay_tmp/lodepng/lodepngConfig.cmake" <<'CMAKE'
if(NOT TARGET lodepng)
  add_library(lodepng INTERFACE IMPORTED)
  set_property(TARGET lodepng PROPERTY INTERFACE_LINK_LIBRARIES lodepng::lodepng)
endif()
set(lodepng_FOUND TRUE)
CMAKE

env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u LD_LIBRARY_PATH -u PYTHONPATH \
  COLCON_DEFAULTS_FILE=/dev/null \
  bash --noprofile --norc -s -- "$overlay_tmp" "$overlay_install" <<'BUILD'
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "$1"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon --log-base "$1/log" build \
  --base-paths "$1/source/mujoco_ros2_control" \
  --build-base "$1/build" --install-base "$2" \
  --packages-select mujoco_ros2_control --allow-overriding mujoco_ros2_control \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
  -DCMAKE_BUILD_TYPE=Release "-Dlodepng_DIR=$1/lodepng"
BUILD

source install/setup.bash
ros2 pkg prefix mujoco_ros2_control
rm -rf "$overlay_tmp"
trap - EXIT
```

最后输出应为当前工作区的 `install/mujoco_ros2_control`；原有 `install/setup.bash` 会自动发现新包，Studio 工程现有环境入口无需调整

构建产物使用复制安装；临时源码、构建目录和日志可在命令结束后删除；`/opt/ros/humble` 保持原安装

## 验证

启动 MuJoCo 原生窗口，关闭该窗口后检查 driver 退出状态；预期正常退出，无 `libGLdispatch` 段错误；重新启动后再停止工作流与子系统，确认主动停止也正常退出

本机已用同一已安装 `libmujoco_simulate.a` 验证原生生命周期：原版关闭窗口后退出 `-11`，渲染线程释放适配器后退出 `0`；本地 overlay 的实际 driver 已确认加载新库，并在相机初始化后正常关窗退出 `0`

三个补丁已在本机安装；隔离 ROS 域 225 的实际 driver 在控制线程刚启动即 SIGINT、相机输出后正常运行再 SIGINT、正常关窗三条路径均退出 `0`；另用真实 `MujocoSimulation` 公共接口确认 UI 先退出、随后才启动物理线程和关闭时，也能取消初次加载并退出 `0`
