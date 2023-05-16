cmake_minimum_required(VERSION 3.8)
project(optitrack_multiplexer_ros2)

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(optitrack_multiplexer_ros2_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  optitrack_multiplexer_ros2_msgs
)

include_directories(include/optitrack_multiplexer_ros2)

add_library(multiplexer_class src/optitrack_multiplexer.cpp)
ament_target_dependencies(multiplexer_class ${DEPENDENCIES})

add_executable(multiplexer_node src/multiplexer_node.cpp)
target_link_libraries(multiplexer_node multiplexer_class)

#Add the configuration file to the binaries
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  multiplexer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
