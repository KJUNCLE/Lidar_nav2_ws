cmake_minimum_required(VERSION 3.5)
project(remote_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_path(MICROHTTPD_INCLUDE_DIR microhttpd.h)
find_library(MICROHTTPD_LIBRARY microhttpd)

add_executable(remote_bridge_node src/remote_bridge_node.cpp)
target_include_directories(remote_bridge_node PRIVATE
  ${MICROHTTPD_INCLUDE_DIR}
)
ament_target_dependencies(remote_bridge_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(remote_bridge_node
  ${MICROHTTPD_LIBRARY}
)

install(TARGETS
  remote_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
