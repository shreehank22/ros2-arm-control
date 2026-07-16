cmake_minimum_required(VERSION 3.8)
project(arm_traj_gen)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(traj_gen_node src/traj_gen_node.cpp)
ament_target_dependencies(traj_gen_node rclcpp trajectory_msgs)

install(TARGETS traj_gen_node DESTINATION lib/${PROJECT_NAME})

ament_package()
