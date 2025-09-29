cmake_minimum_required(VERSION 3.5)
project(pcc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcc src/pcc.cpp)
ament_target_dependencies(pcc
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(pcc ${PCL_LIBRARIES})

install(TARGETS
  pcc
  DESTINATION lib/${PROJECT_NAME})

ament_package()
