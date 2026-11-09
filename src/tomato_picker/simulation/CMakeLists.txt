cmake_minimum_required(VERSION 3.16)
project(tomato_picker_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(tomato_picker_description REQUIRED)
get_filename_component(description_share "${tomato_picker_description_DIR}" DIRECTORY)
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_physics_urdf.py
  ${description_share}/urdf/tomato_picker.sim.urdf
  ${CMAKE_CURRENT_BINARY_DIR}/urdf/tomato_picker.physics.urdf
  ${CMAKE_CURRENT_SOURCE_DIR}/config/handeye.yaml
  COMMAND_ERROR_IS_FATAL ANY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/urdf/tomato_picker.physics.urdf
  DESTINATION share/${PROJECT_NAME}/urdf)
find_package(mujoco_vendor REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tomato_picker_interfaces REQUIRED)

add_library(tomato_picker_harvest_scene SHARED
  src/harvest_scene_plugin.cpp
  src/scoped_camera_plugin.cpp)
ament_target_dependencies(tomato_picker_harvest_scene
  pluginlib rclcpp std_msgs std_srvs tomato_picker_interfaces)
target_link_libraries(tomato_picker_harvest_scene
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins harvest_scene_plugin.xml)
install(TARGETS tomato_picker_harvest_scene
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# Resolve cross-package CAD paths for both isolated and merged colcon installs
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/mjcf/tomato_picker.xml)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/mjcf/tomato_picker.xml scene_xml)
string(REPLACE "../../tomato_picker_description/" "${description_share}/" scene_xml "${scene_xml}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tomato_picker.xml "${scene_xml}")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tomato_picker.xml
  DESTINATION share/${PROJECT_NAME}/mjcf)
install(DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY mjcf/assets DESTINATION share/${PROJECT_NAME}/mjcf)
install(PROGRAMS scripts/generate_mjcf.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)
ament_package()
