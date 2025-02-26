cmake_minimum_required(VERSION 3.8)
project(optitrack_multiplexer_ros2)

add_compile_options(-g)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(optitrack_multiplexer_ros2_msgs REQUIRED)

include_directories($ENV{G1_EIGEN_DIR})

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  optitrack_multiplexer_ros2_msgs
)

# Include directories
include_directories(
  include
  include/optitrack_multiplexer_ros2
)

# Library for memory
add_library(memory_class src/memory.cpp)
target_include_directories(memory_class PUBLIC include)
ament_target_dependencies(memory_class ${DEPENDENCIES})

# Multiplexer class
add_library(multiplexer_class src/optitrack_multiplexer.cpp)
ament_target_dependencies(multiplexer_class ${DEPENDENCIES})

# Multiplexer node
add_executable(multiplexer_node src/multiplexer_node.cpp)
target_link_libraries(multiplexer_node multiplexer_class)
ament_target_dependencies(multiplexer_node ${DEPENDENCIES})

# Listener node
add_executable(listener_node src/listener_node.cpp)
target_link_libraries(listener_node memory_class)
ament_target_dependencies(listener_node ${DEPENDENCIES})

# Install configurations
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  multiplexer_node
  listener_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
