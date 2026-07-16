cmake_minimum_required(VERSION 3.8)
project(arm_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(sim_node src/sim_node.cpp src/dynamics.cpp)
ament_target_dependencies(sim_node rclcpp sensor_msgs Eigen3)

install(TARGETS sim_node DESTINATION lib/${PROJECT_NAME})

ament_package()
