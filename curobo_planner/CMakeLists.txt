cmake_minimum_required(VERSION 3.8)
project(curobo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(core_types REQUIRED)
find_package(motion_planner REQUIRED)
find_package(curobo_bridge_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(curobo_planner src/curobo_planner.cpp)
target_include_directories(curobo_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(curobo_planner PUBLIC motion_planner::motion_planner_core core_types::core_types Eigen3::Eigen)
ament_target_dependencies(curobo_planner PUBLIC rclcpp curobo_bridge_msgs geometry_msgs sensor_msgs)

install(TARGETS curobo_planner EXPORT export_curobo_planner
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(curobo_planner_test test/curobo_planner_test.cpp)
  target_link_libraries(curobo_planner_test curobo_planner)
endif()

ament_export_targets(export_curobo_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp core_types motion_planner curobo_bridge_msgs geometry_msgs sensor_msgs Eigen3)
ament_package()
