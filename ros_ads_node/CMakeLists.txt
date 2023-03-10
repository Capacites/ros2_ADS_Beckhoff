cmake_minimum_required(VERSION 3.5)
project(ros_ads_node)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_STANDARD_REQUIRED ON )

add_subdirectory(lib/ADS/AdsLib)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_ads_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
set(req_deps
  "rclcpp"
  
)

add_executable(${PROJECT_NAME} src/Ads_Interface.cpp src/Ads_node.cpp )

target_include_directories(${PROJECT_NAME}  PRIVATE include)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} yaml-cpp ads -lpthread )
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs ros_ads_msgs)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
  
ament_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
 

