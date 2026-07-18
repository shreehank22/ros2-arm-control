cmake_minimum_required(VERSION 3.8)
project(arm_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node pinocchio::pinocchio)
ament_target_dependencies(sim_node rclcpp sensor_msgs std_msgs)

install(TARGETS sim_node DESTINATION lib/${PROJECT_NAME})

ament_package()