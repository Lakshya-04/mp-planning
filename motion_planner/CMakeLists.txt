cmake_minimum_required(VERSION 3.8)
project(motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(core_types REQUIRED)
find_package(ros_adapters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# motion_planner's scope is planning ALGORITHMS only (CartesianPlanner,
# OmplPlanner, the Planner interface + POD types) -- ManipulationManager,
# CoverageSource/CoverageFeasibilityAnalyzer, MotionValidator, the
# ExecuteMotionSegments action server, and (as of the ManipulationManager
# consolidation) PlannerBenchmark itself all moved to the separate
# manipulation_manager package, since motion_planner must never
# depend on it (see MotionPlanner's own doc comment for the one-way
# dependency direction).
add_library(${PROJECT_NAME}_core
  src/planners/cartesian_planner.cpp
  src/planners/ompl_planner.cpp
  src/planners/reachability_tilt.cpp
  src/moveit_planning_backend.cpp
  src/moveit_scene_adapter.cpp
  src/point_cloud_file_loader.cpp
  src/motion_planner.cpp
)

# core_types/ros_adapters only export modern (namespaced) CMake
# targets (no legacy <pkg>_LIBRARIES var), so they're linked directly rather
# than via ament_target_dependencies, which expects the legacy export style.
# octomap similarly exports a plain (non-namespaced) IMPORTED target, linked
# directly for the same reason. octomap_msgs, unlike those, doesn't export
# one umbrella target (only per-typesupport ones, e.g.
# octomap_msgs::octomap_msgs__rosidl_generator_cpp) -- it goes through
# ament_target_dependencies() below instead, like moveit_msgs.
target_link_libraries(${PROJECT_NAME}_core
  core_types::core_types
  ros_adapters::ros_adapters
  octomap
  ${PCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME}_core PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  std_srvs
  octomap_msgs
)

add_executable(world_model_loader_node src/world_model_loader_node.cpp)
target_compile_features(world_model_loader_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(world_model_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(world_model_loader_node
  rclcpp
)
target_link_libraries(world_model_loader_node ${PROJECT_NAME}_core)

add_executable(coverage_pass_demo_node src/coverage_pass_demo_node.cpp)
target_compile_features(coverage_pass_demo_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(coverage_pass_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(coverage_pass_demo_node
  rclcpp
  rclcpp_action
  control_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(coverage_pass_demo_node ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  world_model_loader_node
  coverage_pass_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(planners_test
    test/planners_test.cpp
  )
  if(TARGET planners_test)
    target_link_libraries(planners_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(point_cloud_file_loader_test
    test/point_cloud_file_loader_test.cpp
  )
  if(TARGET point_cloud_file_loader_test)
    target_link_libraries(point_cloud_file_loader_test ${PROJECT_NAME}_core)
    target_compile_definitions(point_cloud_file_loader_test PRIVATE
      MOTION_PLANNER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    )
  endif()

  ament_add_gtest(motion_planner_test
    test/motion_planner_test.cpp
  )
  if(TARGET motion_planner_test)
    target_link_libraries(motion_planner_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(reachability_tilt_test
    test/reachability_tilt_test.cpp
  )
  if(TARGET reachability_tilt_test)
    target_link_libraries(reachability_tilt_test ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(wall_penetration_check_test
    test/wall_penetration_check_test.cpp
  )
  if(TARGET wall_penetration_check_test)
    target_link_libraries(wall_penetration_check_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  ament_add_gtest(moveit_planning_backend_test
    test/moveit_planning_backend_test.cpp
  )
  if(TARGET moveit_planning_backend_test)
    target_link_libraries(moveit_planning_backend_test ${PROJECT_NAME}_core core_types::core_types)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  core_types
  ros_adapters
  octomap
  octomap_msgs
  std_srvs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
# Consumers outside this package (e.g. manipulation_manager) linking against
# ${PROJECT_NAME}_core -- a static archive -- need its own transitive link requirements
# (moveit/tf2/octomap/PCL/...) carried automatically, which only a real namespaced imported
# target (not ament_export_libraries()'s legacy variable alone) provides.
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()
