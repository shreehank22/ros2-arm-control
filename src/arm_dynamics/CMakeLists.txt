cmake_minimum_required(VERSION 3.8)
project(arm_dynamics)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# ============================================================
# Library
# ============================================================

add_library(${PROJECT_NAME}
  src/dynamics.cpp
  src/kinematics.cpp
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

# ============================================================
# Tests
# ============================================================

ament_add_gtest(test_kinodynamics
  tests/test_kinodynamics.cpp
)

target_link_libraries(test_kinodynamics
  ${PROJECT_NAME}
)

# ============================================================
# Install
# ============================================================

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

# ============================================================
# Export
# ============================================================

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio)

ament_package()