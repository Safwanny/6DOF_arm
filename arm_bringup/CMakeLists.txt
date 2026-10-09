cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/camera_view.py scripts/spawn_cubes.py scripts/cube_detector.py DESTINATION lib/${PROJECT_NAME})

ament_package()
