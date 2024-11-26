cmake_minimum_required(VERSION 3.8)
project(dynamic_lidar_interpolation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


# Add executable
add_executable(pointcloud_interpolation_node 
  src/pointcloud_interpolation_node.cpp 
)

ament_target_dependencies(pointcloud_interpolation_node
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  Eigen3
)

target_link_libraries(pointcloud_interpolation_node
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  pointcloud_interpolation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(DIRECTORY
  config
  launch
  rviz
  foxglove
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
