cmake_minimum_required(VERSION 3.8)
project(arm_traj_gen)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(${PROJECT_NAME}
  src/trajectory.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_add_gtest(test_trajectory
  tests/test_trajectory_node.cpp
)

target_link_libraries(test_trajectory
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()