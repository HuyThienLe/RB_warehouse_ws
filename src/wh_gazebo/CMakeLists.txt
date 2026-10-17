cmake_minimum_required(VERSION 3.8)
project(wh_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(wh_gripper_attacher SHARED
  src/gazebo_gripper_attacher.cpp
)
target_include_directories(wh_gripper_attacher PUBLIC
  include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(wh_gripper_attacher
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(wh_gripper_attacher
  rclcpp
  gazebo_dev
  gazebo_ros
  std_srvs
  std_msgs
  geometry_msgs
)

install(TARGETS wh_gripper_attacher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config worlds models scripts
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/box_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gen_world.py
  scripts/gen_tags.py
  scripts/dynamic_obstacle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
