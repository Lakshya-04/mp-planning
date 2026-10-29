cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/smoke_plan.py scripts/sample_collisions.py DESTINATION lib/${PROJECT_NAME})

ament_package()
