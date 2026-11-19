cmake_minimum_required(VERSION 3.8)
project(reachability_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Same octomap pin as motion_planner_moveit (MoveIt and ROS ship two octomap copies).
if(NOT octomap_DIR AND EXISTS /usr/share/octomap/octomap-config.cmake)
  set(octomap_DIR /usr/share/octomap CACHE PATH "octomap package directory (same copy as MoveIt)")
endif()
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
find_package(reachability REQUIRED)
find_package(motion_planner REQUIRED)

# ROS-free: the verifier over the motion_planner ports.
add_library(port_stand_verifier src/port_stand_verifier.cpp)
target_include_directories(port_stand_verifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(port_stand_verifier PUBLIC reachability::reachability_core motion_planner::motion_planner_core Eigen3::Eigen)
target_compile_features(port_stand_verifier PUBLIC cxx_std_17)
set_target_properties(port_stand_verifier PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The probe needs MoveIt (motion_planner_moveit); the verifier and its tests do not.
find_package(motion_planner_moveit QUIET)
find_package(rclcpp QUIET)
find_package(tf2_ros QUIET)
if(motion_planner_moveit_FOUND AND rclcpp_FOUND AND tf2_ros_FOUND)
  find_package(octomap REQUIRED)
  find_package(moveit_ros_planning_interface REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  add_executable(stand_verify_probe apps/stand_verify_probe.cpp)
  target_link_libraries(stand_verify_probe port_stand_verifier motion_planner_moveit::motion_planner_moveit_lib)
  ament_target_dependencies(stand_verify_probe rclcpp tf2_ros)
  install(TARGETS stand_verify_probe DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
else()
  message(WARNING "reachability_moveit: motion_planner_moveit not found, building without stand_verify_probe")
endif()

# The library is static and linked into the probe; it is not installed as a target (colcon's symlink install
# refuses a target whose link interface holds imported targets). Include it with add_subdirectory or copy the
# two files if another package needs it.
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(port_stand_verifier_test test/port_stand_verifier_test.cpp)
  target_link_libraries(port_stand_verifier_test port_stand_verifier)
endif()

ament_export_include_directories(include)
ament_export_dependencies(reachability motion_planner Eigen3)
ament_package()
