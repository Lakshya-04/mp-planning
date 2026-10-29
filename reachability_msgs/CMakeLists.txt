cmake_minimum_required(VERSION 3.14.4)
project(reachability_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseCandidate.msg"
  "msg/LiftBand.msg"
  "srv/IsReachable.srv"
  "srv/GetBaseCandidates.srv"
  "srv/PlanLiftBands.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()
