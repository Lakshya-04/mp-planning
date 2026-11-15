cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/smoke_plan.py scripts/sample_collisions.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(rclcpp REQUIRED)
  # FK/IK round trip through MoveIt, driven by test/test_moveit_swap.py.
  add_executable(fk_ik_check test/fk_ik_check.cpp)
  target_link_libraries(fk_ik_check moveit_ros_planning::moveit_robot_model_loader rclcpp::rclcpp)
  install(TARGETS fk_ik_check DESTINATION lib/${PROJECT_NAME})
  ament_add_pytest_test(test_moveit_swap test/test_moveit_swap.py TIMEOUT 600)
endif()

ament_package()
