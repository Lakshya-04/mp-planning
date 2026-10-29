cmake_minimum_required(VERSION 3.8)
project(reachability)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# core/: ROS-free. RM4D map, arm model (joint chain FK), parallel builder, lift-band planner, I/O.
add_library(reachability_core
  src/core/map4d.cpp
  src/core/arm_model.cpp
  src/core/builder.cpp
  src/core/lift_bands.cpp
  src/core/station.cpp
  src/core/io.cpp
)
target_include_directories(reachability_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(reachability_core PUBLIC Eigen3::Eigen yaml-cpp Threads::Threads)
target_compile_features(reachability_core PUBLIC cxx_std_17)

add_executable(rm4d_tool apps/rm4d_tool.cpp)
target_link_libraries(rm4d_tool reachability_core)

# ros/: services over a loaded map. Optional: the core builds without rclcpp.
find_package(rclcpp QUIET)
find_package(reachability_msgs QUIET)
set(REACHABILITY_TARGETS reachability_core)
if(rclcpp_FOUND AND reachability_msgs_FOUND)
  add_library(reachability_ros src/ros/reachability_node.cpp)
  target_link_libraries(reachability_ros PUBLIC reachability_core)
  ament_target_dependencies(reachability_ros PUBLIC rclcpp reachability_msgs)
  add_executable(reachability_node src/ros/reachability_main.cpp)
  target_link_libraries(reachability_node reachability_ros)
  install(TARGETS reachability_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
  list(APPEND REACHABILITY_TARGETS reachability_ros)
else()
  message(WARNING "rclcpp/reachability_msgs not found: building reachability without the ROS node")
endif()

install(TARGETS ${REACHABILITY_TARGETS}
  EXPORT export_reachability
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS rm4d_tool DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/urdf_to_chain.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t map4d builder lift_bands io station)
    ament_add_gtest(${t}_test test/core/${t}_test.cpp)
    target_link_libraries(${t}_test reachability_core)
    target_include_directories(${t}_test PRIVATE test/core)
  endforeach()
  if(TARGET reachability_ros)
    ament_add_gtest(reachability_node_test test/ros/reachability_node_test.cpp)
    target_link_libraries(reachability_node_test reachability_ros)
    target_include_directories(reachability_node_test PRIVATE test/core)
  endif()
endif()

ament_export_targets(export_reachability HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
