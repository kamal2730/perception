cmake_minimum_required(VERSION 3.5)
project(pcc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloud2Array.msg"
  DEPENDENCIES sensor_msgs
)

# Include your headers
include_directories(include)

# Create executable
add_executable(pcc_node src/pcc.cpp)

# Link ROS2 dependencies
ament_target_dependencies(pcc_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
)

# Link generated messages
rosidl_target_interfaces(pcc_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install
install(TARGETS
  pcc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
