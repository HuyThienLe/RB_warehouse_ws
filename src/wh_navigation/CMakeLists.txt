cmake_minimum_required(VERSION 3.8)
project(wh_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/auto_initial_pose.py
  scripts/go_to_waypoint.py
  scripts/estop_safety_node.py
  scripts/mode_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
