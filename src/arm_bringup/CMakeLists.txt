cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Adds the platform and the ground to MoveIt's planning scene
add_executable(planning_scene_setup
  src/planning_scene_setup.cpp
)

ament_target_dependencies(planning_scene_setup
  rclcpp
  moveit_msgs
  shape_msgs
  geometry_msgs
)

install(TARGETS planning_scene_setup
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
