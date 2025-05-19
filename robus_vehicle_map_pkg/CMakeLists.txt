# Ensure we define minimum version and project
cmake_minimum_required(VERSION 3.5)
project(robus_vehicle_map_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Export dependencies - must be called before ament_package
ament_export_dependencies(rclcpp geometry_msgs nav_msgs visualization_msgs)

# Include directories
include_directories(include)

# Declare your executables
add_executable(random_ego_publisher src/random_ego_node.cpp)

# Link libraries
ament_target_dependencies(random_ego_publisher rclcpp geometry_msgs nav_msgs)

# Install the executable
install(TARGETS random_ego_publisher DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Finally call ament_package() - this must be the last line
ament_package()


