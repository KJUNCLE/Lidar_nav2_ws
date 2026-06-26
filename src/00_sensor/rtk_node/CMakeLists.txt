cmake_minimum_required(VERSION 3.8)
project(rtk_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(rtk_node_core
  src/message_conversions.cpp
  src/rtk_parser.cpp
  src/serial_port.cpp
)
target_include_directories(rtk_node_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtk_node_core
  builtin_interfaces
  rclcpp
  rtk_msgs
  sensor_msgs
)

add_executable(rtk_node
  src/rtk_node.cpp
)
target_link_libraries(rtk_node
  rtk_node_core
)
ament_target_dependencies(rtk_node
  rclcpp
  rtk_msgs
  sensor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    rtk_node
    rtk_node_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(rtk_parser_test
    test/rtk_parser_test.cpp
  )
  target_include_directories(rtk_parser_test PRIVATE include)
  target_link_libraries(rtk_parser_test
    rtk_node_core
  )
  ament_target_dependencies(rtk_parser_test
    builtin_interfaces
    rtk_msgs
  )

  ament_add_gtest(message_conversions_test
    test/message_conversions_test.cpp
  )
  target_include_directories(message_conversions_test PRIVATE include)
  target_link_libraries(message_conversions_test
    rtk_node_core
  )
  ament_target_dependencies(message_conversions_test
    rtk_msgs
    sensor_msgs
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(rtk_node_core)
ament_export_dependencies(builtin_interfaces rclcpp rtk_msgs sensor_msgs)

ament_package()
