cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Find Dependencies
# ========================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ========================
# Generate Messages
# ========================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloud2Array.msg"
  DEPENDENCIES sensor_msgs
)

# ========================
# Executables
# ========================

add_executable(perception_node src/perception.cpp)
add_executable(depth2pose_node src/depth2pose.cpp)
add_executable(cluster_merger_node src/merge_clusters.cpp)

# ========================
# Include Directories
# ========================

target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(depth2pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(cluster_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# ========================
# Link Libraries
# ========================

target_link_libraries(perception_node ${PCL_LIBRARIES})
target_link_libraries(depth2pose_node ${PCL_LIBRARIES})
target_link_libraries(cluster_merger_node ${PCL_LIBRARIES})

# ========================
# Compile Features
# ========================

target_compile_features(perception_node PUBLIC c_std_99 cxx_std_17)

# ========================
# Ament Dependencies
# ========================

ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  custom_interfaces
)

ament_target_dependencies(depth2pose_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  geometry_msgs
  custom_interfaces
)

ament_target_dependencies(cluster_merger_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

# ========================
# ROSIDL Target Interfaces
# ========================

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(depth2pose_node ${cpp_typesupport_target})
target_link_libraries(cluster_merger_node ${cpp_typesupport_target})

# ========================
# Install
# ========================

install(TARGETS
  perception_node
  depth2pose_node
  cluster_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ========================
# Testing
# ========================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()