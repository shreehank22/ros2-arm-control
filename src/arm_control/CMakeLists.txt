
cmake_minimum_required(VERSION 3.8)
project(arm_control)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

# Main controller
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node pinocchio::pinocchio)
ament_target_dependencies(controller_node ament_index_cpp rclcpp sensor_msgs std_msgs)

# Read-only gravity calculation test
add_executable(gravity_test src/gravity_test.cpp)
target_link_libraries(gravity_test pinocchio::pinocchio)

install(TARGETS
  controller_node
  gravity_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
