cmake_minimum_required(VERSION 3.8)
project(tomato_picker_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetObject.msg"
  "msg/HarvestPhysicsState.msg"
  "srv/CommandEef.srv"
  "srv/SetSceneEnabled.srv"
  "action/MoveArm.action"
  "action/PickTarget.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
