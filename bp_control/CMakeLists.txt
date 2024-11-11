cmake_minimum_required(VERSION 3.5)
project(bp_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
install(
    DIRECTORY src launch 
    DESTINATION lib/${PROJECT_NAME}
)

# Mark the package as ament
ament_package()

