cmake_minimum_required(VERSION 3.8)
project(tomato_picker_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wait_for_arm_ready.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wait_for_arm_ready
)

install(PROGRAMS
  scripts/wait_for_eef_ready.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wait_for_eef_ready
)

install(PROGRAMS
  scripts/wait_for_motion_ready.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wait_for_motion_ready
)

install(PROGRAMS
  scripts/handeye_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME handeye_bridge
)

install(PROGRAMS
  scripts/handeye_operator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME handeye_operator
)

install(PROGRAMS
  scripts/handeye_servo_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME handeye_servo_pose
)

install(PROGRAMS
  scripts/check_handeye_servo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_handeye_servo
)

ament_package()
