cmake_minimum_required(VERSION 3.5)
project(perception)

# Add policy to handle PCL_ROOT
cmake_policy(SET CMP0074 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloud2Array.msg"
  DEPENDENCIES sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(depth2pose_node src/depth2pose.cpp)

ament_target_dependencies(depth2pose_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  custom_interfaces
  geometry_msgs
)

target_link_libraries(depth2pose_node ${PCL_LIBRARIES})

rosidl_target_interfaces(depth2pose_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(cluster_merger_node src/merge_clusters.cpp)

ament_target_dependencies(cluster_merger_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros 
)

target_link_libraries(cluster_merger_node ${PCL_LIBRARIES})

rosidl_target_interfaces(cluster_merger_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(detection_visualizer_node src/detection_visualizer.cpp)

ament_target_dependencies(detection_visualizer_node
  rclcpp
  sensor_msgs
  custom_interfaces
  cv_bridge
  OpenCV
)

rosidl_target_interfaces(detection_visualizer_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)



install(TARGETS
  depth2pose_node
  cluster_merger_node
  detection_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
