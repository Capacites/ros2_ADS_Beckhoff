cmake_minimum_required(VERSION 3.5)
project(ros_ads_msgs)


set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_STANDARD_REQUIRED ON )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ADS.msg" 
  "msg/State.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES std_msgs)

add_library(${PROJECT_NAME}_decode src/ADSDecode.cpp)
target_include_directories(${PROJECT_NAME}_decode PRIVATE include)
target_link_libraries(${PROJECT_NAME}_decode ${rclcpp_LIBRARIES}) 
rosidl_target_interfaces(${PROJECT_NAME}_decode  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(${PROJECT_NAME}_decode rclcpp)
ament_export_targets(${PROJECT_NAME}_decode HAS_LIBRARY_TARGET)

add_dependencies(${PROJECT_NAME}_decode ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_decode
  EXPORT ${PROJECT_NAME}_decode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()