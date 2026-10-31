cmake_minimum_required(VERSION 3.8)
project(rokea_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  urdf
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/step_to_torso_stl.py
  scripts/zero_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
