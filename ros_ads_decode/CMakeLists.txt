cmake_minimum_required(VERSION 3.5)
project(ros_ads_decode)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_STANDARD_REQUIRED ON )

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_ads_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)




add_library(${PROJECT_NAME} SHARED src/ADSDecode.cpp)

target_include_directories(${PROJECT_NAME}  PRIVATE include)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs ros_ads_msgs )
ament_export_include_directories(include)     # <-- superfluous?
ament_export_libraries(${PROJECT_NAME})  

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)   # <-- documentation prepends export_
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}     # <-- documentation prepends export_
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
 

