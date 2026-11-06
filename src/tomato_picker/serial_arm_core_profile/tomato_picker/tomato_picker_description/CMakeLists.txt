cmake_minimum_required(VERSION 3.8)
project(tomato_picker_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(simulation_urdf "${CMAKE_CURRENT_BINARY_DIR}/tomato_picker.sim.urdf")
add_custom_command(
  OUTPUT "${simulation_urdf}"
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_simulation_urdf.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/tomato_picker.urdf"
    "${simulation_urdf}"
  DEPENDS
    urdf/tomato_picker.urdf
    scripts/generate_simulation_urdf.py
    meshes/simulation/base_link_collision.stl
  VERBATIM
)
add_custom_target(simulation_description ALL DEPENDS "${simulation_urdf}")

install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES "${simulation_urdf}" DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
