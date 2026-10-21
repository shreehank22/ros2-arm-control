
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
target_include_directories(controller_node PRIVATE include)
target_link_libraries(controller_node pinocchio::pinocchio)
ament_target_dependencies(controller_node ament_index_cpp rclcpp sensor_msgs std_msgs)

# Read-only gravity calculation test
add_executable(gravity_test tests/gravity_test.cpp)
target_link_libraries(gravity_test pinocchio::pinocchio)

install(TARGETS
  controller_node
  gravity_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Unit tests
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_control_state tests/test_control_state.cpp)
target_include_directories(test_control_state PRIVATE include)

ament_package()
