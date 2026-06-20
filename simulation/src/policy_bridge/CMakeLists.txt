cmake_minimum_required(VERSION 3.8)
project(policy_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  policy_bridge/policy_bridge_node.py
  RENAME policy_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# FlightRL-v1 непрерывный velocity-облёт (rl-lab Часть B / Aleks 2026-06-10)
install(PROGRAMS
  policy_bridge/manual_fly_node.py
  RENAME manual_fly_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  policy_bridge/teleop_keyboard.py
  RENAME teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  policy_bridge/m1c_bridge_node.py
  RENAME m1c_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
