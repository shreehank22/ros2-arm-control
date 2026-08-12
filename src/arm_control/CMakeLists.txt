cmake_minimum_required(VERSION 3.8)
project(arm_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node pinocchio::pinocchio)
ament_target_dependencies(controller_node rclcpp sensor_msgs std_msgs)

install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

ament_package()