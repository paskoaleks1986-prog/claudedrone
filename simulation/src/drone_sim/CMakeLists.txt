cmake_minimum_required(VERSION 3.8)
project(drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ зависимости
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python поддержка
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Регистрируем ноды
install(PROGRAMS
  drone_sim/sensor_monitor.py
  RENAME sensor_monitor
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  drone_sim/takeoff_node.py
  RENAME takeoff
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  drone_sim/stop_scan_node.py
  RENAME stop_scan
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  drone_sim/servo_cmd_node.py
  RENAME servo_cmd
  DESTINATION lib/${PROJECT_NAME}
)
# Ресурсы — launch, models, worlds
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Всегда последняя строка!
ament_package()