cmake_minimum_required(VERSION 3.5)
project(adaptive_clustering)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
#find_package(adaptive_clustering REQUIRED)  
# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Declare message files
set(msg_files
  "msg/ClusterArray.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)
# Add executable
add_executable(adaptive_clustering_node src/adaptive_clustering.cpp)

# Link libraries
ament_target_dependencies(adaptive_clustering_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  PCL
)

target_link_libraries(adaptive_clustering_node ${PCL_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(adaptive_clustering_node "${cpp_typesupport_target}") 
#target_link_libraries(adaptive_clustering_node ${PROJECT_NAME})

# Ensure the message headers are included
#add_dependencies(adaptive_clustering_node ${PROJECT_NAME}_cpp)

# Install targets
install(TARGETS adaptive_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
