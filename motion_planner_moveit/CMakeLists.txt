cmake_minimum_required(VERSION 3.8)
project(motion_planner_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# PCL (and its Findlibusb.cmake module) must be resolved before motion_planner: the exported
# motion_planner_pcd link interface references the libusb::libusb imported target that only
# PCL's own find_package() call defines.
find_package(PCL REQUIRED COMPONENTS common io)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
find_package(motion_planner REQUIRED)

# ------------------------------------------------------------------------------
# MoveIt adapter: implements the motion_planner ports on top of MoveIt / OMPL.
# ------------------------------------------------------------------------------
add_library(motion_planner_moveit_lib
  src/moveit_planning_backend.cpp
  src/moveit_scene_adapter.cpp
  src/cartesian_planner.cpp
  src/ompl_planner.cpp
)
target_include_directories(motion_planner_moveit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_compile_features(motion_planner_moveit_lib PUBLIC cxx_std_17)
# core_types/ros_adapters/octomap export modern (namespaced) targets: link them directly rather
# than via ament_target_dependencies, which expects the legacy export style.
target_link_libraries(motion_planner_moveit_lib PUBLIC
  motion_planner::motion_planner_core
  motion_planner::motion_planner_pcd
  core_types::core_types
  ros_adapters::ros_adapters
  octomap
  ${PCL_LIBRARIES})
ament_target_dependencies(motion_planner_moveit_lib PUBLIC
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  moveit_ros_planning_interface moveit_ros_planning moveit_msgs std_srvs octomap_msgs)

add_executable(world_model_loader_node src/world_model_loader_node.cpp)
target_link_libraries(world_model_loader_node motion_planner_moveit_lib)
ament_target_dependencies(world_model_loader_node rclcpp)

add_executable(coverage_pass_demo_node src/coverage_pass_demo_node.cpp)
target_link_libraries(coverage_pass_demo_node motion_planner_moveit_lib nlohmann_json::nlohmann_json)
ament_target_dependencies(coverage_pass_demo_node
  rclcpp rclcpp_action control_msgs std_msgs geometry_msgs)

install(TARGETS motion_planner_moveit_lib
  EXPORT export_motion_planner_moveit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS world_model_loader_node coverage_pass_demo_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_add_gtest(planners_test test/planners_test.cpp)
  target_link_libraries(planners_test motion_planner_moveit_lib)

  ament_add_gtest(moveit_planning_backend_test test/moveit_planning_backend_test.cpp)
  target_link_libraries(moveit_planning_backend_test motion_planner_moveit_lib)
endif()

ament_export_targets(export_motion_planner_moveit HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs moveit_ros_planning_interface
  moveit_ros_planning moveit_msgs core_types ros_adapters octomap octomap_msgs std_srvs
  motion_planner)
ament_package()
