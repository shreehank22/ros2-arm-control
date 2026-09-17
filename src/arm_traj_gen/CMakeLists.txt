cmake_minimum_required(VERSION 3.8)
project(arm_traj_gen)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(
  include
)

add_executable(traj_gen_node
  src/traj_gen_node.cpp
  src/dls_ik.cpp
)

target_link_libraries(traj_gen_node
  pinocchio::pinocchio
)

ament_target_dependencies(traj_gen_node
  rclcpp
)

install(TARGETS
  traj_gen_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()