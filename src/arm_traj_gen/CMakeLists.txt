
cmake_minimum_required(VERSION 3.8)
project(arm_traj_gen)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(arm_kinodynamics REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Quintic trajectory library
add_library(${PROJECT_NAME}
  src/trajectory.cpp
  src/joint_limits.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
)

# Trajectory generator ROS 2 node
add_executable(traj_gen_node
  src/traj_gen_node.cpp
)

target_link_libraries(traj_gen_node
  ${PROJECT_NAME}
  pinocchio::pinocchio
  arm_kinodynamics::arm_kinodynamics
)

ament_target_dependencies(traj_gen_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)

# FollowJointTrajectory action server: streams MoveIt trajectories to /desired_joint_states
add_executable(trajectory_action_server
  src/trajectory_action_server.cpp
)

target_link_libraries(trajectory_action_server
  Eigen3::Eigen
  pinocchio::pinocchio
  arm_kinodynamics::arm_kinodynamics
)

ament_target_dependencies(trajectory_action_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)

# Unit tests
ament_add_gtest(test_trajectory
  tests/test_trajectory_node.cpp
)

target_link_libraries(test_trajectory
  ${PROJECT_NAME}
)

ament_add_gtest(test_joint_limits
  tests/test_joint_limits.cpp
)

target_link_libraries(test_joint_limits
  ${PROJECT_NAME}
)

target_compile_definitions(test_joint_limits PRIVATE
  FR3_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../arm_sim/urdf/fr3.urdf"
)

# Integration test: fake /joint_states -> traj_gen_node -> /desired_joint_states (isolated domain)
ament_add_pytest_test(test_traj_gen_node_integration
  tests/test_traj_gen_node_integration.py
  ENV TRAJ_GEN_NODE=$<TARGET_FILE:traj_gen_node>
  TIMEOUT 120
)

# Install library and export it
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable where ros2 run expects it
install(
  TARGETS traj_gen_node trajectory_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
