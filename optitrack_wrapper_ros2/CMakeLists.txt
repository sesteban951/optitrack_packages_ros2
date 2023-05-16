cmake_minimum_required(VERSION 3.8)
project(optitrack_wrapper_ros2)

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(optitrack_wrapper_ros2_msgs REQUIRED)

set(LIBNATNET_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/lib/libNatNet.so)
set(NATNET_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/include
)
set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  optitrack_wrapper_ros2_msgs
)

include_directories(include/optitrack_wrapper_ros2 ${NATNET_INCLUDE_DIR})

add_library(description_message_functions src/description_message_functions.cpp)
ament_target_dependencies(description_message_functions ${DEPENDENCIES})

add_library(data_message_functions src/data_message_functions.cpp)
ament_target_dependencies(data_message_functions ${DEPENDENCIES})

add_library(wrapper_class src/optitrack_wrapper.cpp)
ament_target_dependencies(wrapper_class ${DEPENDENCIES})
target_link_libraries(wrapper_class 
                      description_message_functions 
                      data_message_functions 
                      ${LIBNATNET_SDK_LIBRARY})

add_executable(wrapper_node src/wrapper_node.cpp)
target_link_libraries(wrapper_node wrapper_class)

#Add the configuration file to the binaries
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  ${LIBNATNET_SDK_LIBRARY}
  DESTINATION lib
)

ament_package()
