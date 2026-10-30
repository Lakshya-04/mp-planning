cmake_minimum_required(VERSION 3.14.4)
project(curobo_bridge_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Box.msg"
  "srv/PlanJoint.srv"
  "srv/PlanPose.srv"
  "srv/SetWorld.srv"
  "srv/SolveIk.srv"
  "srv/CheckMotion.srv"
  DEPENDENCIES geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs sensor_msgs trajectory_msgs)
ament_package()
