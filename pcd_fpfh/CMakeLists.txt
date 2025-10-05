cmake_minimum_required(VERSION 3.5)
project(pcd_fpfh)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io features filters)
find_package(visualization_msgs REQUIRED)
find_package(pcc REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_fpfh src/pcd_fpfh.cpp)

ament_target_dependencies(pcd_fpfh
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  pcc
  custom_interfaces
)

target_link_libraries(pcd_fpfh ${PCL_LIBRARIES})

target_include_directories(pcd_fpfh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS pcd_fpfh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
