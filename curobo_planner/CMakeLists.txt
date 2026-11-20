cmake_minimum_required(VERSION 3.8)
project(curobo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
# PCL first: the exported motion_planner_pcd link interface needs the libusb target PCL defines.
find_package(PCL QUIET COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(core_types REQUIRED)
find_package(motion_planner REQUIRED)
find_package(scene_model REQUIRED)
find_package(curobo_bridge_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(curobo_planner src/curobo_planner.cpp src/joint_limit_guard.cpp)
target_include_directories(curobo_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(curobo_planner PUBLIC motion_planner::motion_planner_core scene_model::scene_model_core core_types::core_types Eigen3::Eigen yaml-cpp)
ament_target_dependencies(curobo_planner PUBLIC rclcpp curobo_bridge_msgs geometry_msgs sensor_msgs)

# pluginlib plugins (backend selection, docs/backend_selection.md); the planner library is linked
# in, so it must be position independent.
set_target_properties(curobo_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(curobo_planner_plugins SHARED src/plugins.cpp)
target_link_libraries(curobo_planner_plugins PUBLIC curobo_planner pluginlib::pluginlib)
ament_target_dependencies(curobo_planner_plugins PUBLIC ament_index_cpp)
pluginlib_export_plugin_description_file(motion_planner plugins.xml)
install(TARGETS curobo_planner_plugins ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# Wall-clock benchmark of the node + client path: scripts/planner_benchmark.py in motion_planner_moveit,
# docs/planner_benchmark.md.
add_executable(curobo_planner_benchmark src/planner_benchmark.cpp)
target_link_libraries(curobo_planner_benchmark curobo_planner)
install(TARGETS curobo_planner_benchmark RUNTIME DESTINATION lib/${PROJECT_NAME})

# One-shot loader: a point-cloud file into the cuRobo node's world (sim passes, next to world_model_loader).
if(PCL_FOUND AND TARGET motion_planner::motion_planner_pcd)
  add_executable(curobo_world_loader src/curobo_world_loader_node.cpp)
  target_link_libraries(curobo_world_loader curobo_planner motion_planner::motion_planner_pcd ${PCL_LIBRARIES})
  ament_target_dependencies(curobo_world_loader tf2_ros)
  install(TARGETS curobo_world_loader RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS curobo_planner EXPORT export_curobo_planner
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(curobo_planner_test test/curobo_planner_test.cpp)
  target_link_libraries(curobo_planner_test curobo_planner)
  ament_add_gtest(joint_limit_guard_test test/joint_limit_guard_test.cpp)
  target_link_libraries(joint_limit_guard_test curobo_planner)
  target_compile_definitions(joint_limit_guard_test PRIVATE
    SHIPPED_JOINT_LIMITS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../../description/mobile_manipulator_moveit_config/config/joint_limits.yaml")
  ament_add_gtest(curobo_plugins_test test/curobo_plugins_test.cpp)
  target_link_libraries(curobo_plugins_test curobo_planner motion_planner::motion_planner_ros)
endif()

ament_export_targets(export_curobo_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp pluginlib yaml-cpp ament_index_cpp core_types motion_planner scene_model curobo_bridge_msgs geometry_msgs sensor_msgs Eigen3)
ament_package()
