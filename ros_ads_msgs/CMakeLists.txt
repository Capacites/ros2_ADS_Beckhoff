cmake_minimum_required(VERSION 3.5)
project(ros_ads_msgs)


set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_STANDARD_REQUIRED ON )

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ADS.msg" "msg/State.msg" DEPENDENCIES std_msgs)

ament_package()



