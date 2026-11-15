cmake_minimum_required(VERSION 3.8)
project(motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(core_types REQUIRED)

# ------------------------------------------------------------------------------
# core/: ROS-free. The planning ports (Planner, IkPort, CollisionPort, ScenePort), the POD
# request/result types, MotionPlanner (segment planning + bridge-resume orchestration) and the
# pure geometry helpers. No rclcpp, MoveIt, tf2 or message headers anywhere in this package.
# Backends (MoveIt/OMPL today; cuRobo, VAMP later) are separate adapter packages that
# implement these ports, e.g. motion_planner_moveit.
# ------------------------------------------------------------------------------
add_library(motion_planner_core
  src/core/motion_planner.cpp
  src/core/planners/reachability_tilt.cpp
  src/core/planners/seed_preferring_ik.cpp
)
target_include_directories(motion_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motion_planner_core PUBLIC core_types::core_types Eigen3::Eigen)
target_compile_features(motion_planner_core PUBLIC cxx_std_17)

# Optional .pcd/.ply point-cloud file loading (the only part of the core that needs PCL).
find_package(PCL QUIET COMPONENTS common io)
set(CORE_TARGETS motion_planner_core)
if(PCL_FOUND)
  add_library(motion_planner_pcd src/core/point_cloud_file_loader.cpp)
  target_link_libraries(motion_planner_pcd PUBLIC motion_planner_core ${PCL_LIBRARIES})
  target_include_directories(motion_planner_pcd PUBLIC ${PCL_INCLUDE_DIRS})
  list(APPEND CORE_TARGETS motion_planner_pcd)
else()
  message(WARNING "PCL not found: motion_planner builds without point-cloud file loading.")
endif()

install(TARGETS ${CORE_TARGETS}
  EXPORT export_motion_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_add_gtest(motion_planner_test test/core/motion_planner_test.cpp)
  target_link_libraries(motion_planner_test motion_planner_core)

  ament_add_gtest(reachability_tilt_test test/core/reachability_tilt_test.cpp)
  target_link_libraries(reachability_tilt_test motion_planner_core)

  ament_add_gtest(seed_preferring_ik_test test/core/seed_preferring_ik_test.cpp)
  target_link_libraries(seed_preferring_ik_test motion_planner_core)

  ament_add_gtest(wall_penetration_check_test test/core/wall_penetration_check_test.cpp)
  target_link_libraries(wall_penetration_check_test motion_planner_core)

  if(PCL_FOUND)
    ament_add_gtest(point_cloud_file_loader_test test/core/point_cloud_file_loader_test.cpp)
    target_link_libraries(point_cloud_file_loader_test motion_planner_pcd)
    target_compile_definitions(point_cloud_file_loader_test PRIVATE
      MOTION_PLANNER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
endif()

ament_export_targets(export_motion_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(core_types Eigen3)
ament_package()
