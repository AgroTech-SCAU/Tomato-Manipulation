cmake_minimum_required(VERSION 3.8)
project(tomato_picker_motion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tomato_picker_interfaces REQUIRED)

add_executable(arm_motion_node src/arm_motion_server.cpp)
ament_target_dependencies(arm_motion_node
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  tomato_picker_interfaces
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS arm_motion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
