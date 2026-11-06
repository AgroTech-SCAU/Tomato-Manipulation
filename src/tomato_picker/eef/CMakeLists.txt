cmake_minimum_required(VERSION 3.8)
project(tomato_picker_eef)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_arm_core CONFIG REQUIRED)
find_package(serial_arm_hardware_damiao REQUIRED)
find_package(serial_arm_protocol_damiao_usb2can CONFIG REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tomato_picker_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(TOMATO_PICKER_EEF_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(TOMATO_PICKER_EEF_YAML_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp was found, but no usable target exists")
endif()

add_library(tomato_picker_eef_controller SHARED
  src/damiao_eef_driver.cpp
  src/eef_controller.cpp
  src/eef_worker.cpp
)
target_include_directories(tomato_picker_eef_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tomato_picker_eef_controller
  ament_index_cpp
  controller_interface
  pluginlib
  rclcpp
  serial_arm_core
  serial_arm_hardware_damiao
  serial_arm_protocol_damiao_usb2can
  std_srvs
  tomato_picker_interfaces
)
target_link_libraries(tomato_picker_eef_controller
  serial_arm::core
  serial_arm_protocol_damiao_usb2can
  ${TOMATO_PICKER_EEF_YAML_TARGET}
)

add_executable(eef_mock_node src/eef_mock_node.cpp)
ament_target_dependencies(eef_mock_node rclcpp sensor_msgs std_srvs tomato_picker_interfaces)

pluginlib_export_plugin_description_file(controller_interface eef_controller_plugin.xml)

install(
  TARGETS tomato_picker_eef_controller eef_mock_node
  EXPORT export_tomato_picker_eef_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_tomato_picker_eef_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  rclcpp
  serial_arm_core
  serial_arm_hardware_damiao
  serial_arm_protocol_damiao_usb2can
  std_srvs
  tomato_picker_interfaces
)

ament_package()
