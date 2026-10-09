cmake_minimum_required(VERSION 3.8)
project(wh_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds models scripts
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/box_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gen_world.py
  scripts/gen_tags.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
