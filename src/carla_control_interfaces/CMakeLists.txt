cmake_minimum_required(VERSION 3.5)
project(carla_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateTrajectory.srv"
)

# Explicitly install the srv directory (IMPORTANT)
install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
