cmake_minimum_required(VERSION 3.8)
project(agv_localization)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoInfo.msg"
)

ament_package()