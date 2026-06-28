cmake_minimum_required(VERSION 3.8)
project(autodrive_d02_can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autodrive_d02_can_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(autodrive_d02_can_encoder
  src/can_encoder.cpp
)
target_include_directories(autodrive_d02_can_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(autodrive_d02_can_decoder
  src/can_decoder.cpp
)
target_include_directories(autodrive_d02_can_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(autodrive_d02_cmd_vel_converter
  src/cmd_vel_converter.cpp
)
target_include_directories(autodrive_d02_cmd_vel_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(autodrive_d02_cmd_vel_converter PUBLIC
  autodrive_d02_can_encoder
)

add_executable(d02_vehicle_control_sender_node
  src/d02_vehicle_control_sender_node.cpp
)
target_link_libraries(d02_vehicle_control_sender_node
  autodrive_d02_can_encoder
)
ament_target_dependencies(d02_vehicle_control_sender_node
  autodrive_d02_can_msgs
  can_msgs
  rclcpp
)

add_executable(d02_cmd_vel_to_vehicle_control_node
  src/d02_cmd_vel_to_vehicle_control_node.cpp
)
target_link_libraries(d02_cmd_vel_to_vehicle_control_node
  autodrive_d02_cmd_vel_converter
)
ament_target_dependencies(d02_cmd_vel_to_vehicle_control_node
  autodrive_d02_can_msgs
  geometry_msgs
  rclcpp
)

add_executable(d02_chassis_control_test_node
  src/d02_chassis_control_test_node.cpp
)
ament_target_dependencies(d02_chassis_control_test_node
  can_msgs
  rclcpp
)

add_executable(d02_vehicle_feedback_decoder_node
  src/d02_vehicle_feedback_decoder_node.cpp
)
target_link_libraries(d02_vehicle_feedback_decoder_node
  autodrive_d02_can_decoder
)
ament_target_dependencies(d02_vehicle_feedback_decoder_node
  autodrive_d02_can_msgs
  can_msgs
  rclcpp
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
    autodrive_d02_can_decoder
    autodrive_d02_can_encoder
    autodrive_d02_cmd_vel_converter
    d02_chassis_control_test_node
    d02_cmd_vel_to_vehicle_control_node
    d02_vehicle_feedback_decoder_node
    d02_vehicle_control_sender_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(d02_vehicle_control_encoder_test
    test/d02_vehicle_control_encoder_test.cpp
  )
  target_link_libraries(d02_vehicle_control_encoder_test
    autodrive_d02_can_encoder
  )

  ament_add_gtest(d02_cmd_vel_converter_test
    test/d02_cmd_vel_converter_test.cpp
  )
  target_link_libraries(d02_cmd_vel_converter_test
    autodrive_d02_cmd_vel_converter
  )

  ament_add_gtest(d02_target_speed_feedback_decoder_test
    test/d02_target_speed_feedback_decoder_test.cpp
  )
  target_link_libraries(d02_target_speed_feedback_decoder_test
    autodrive_d02_can_decoder
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  autodrive_d02_can_decoder
  autodrive_d02_can_encoder
  autodrive_d02_cmd_vel_converter
)
ament_export_dependencies(autodrive_d02_can_msgs can_msgs geometry_msgs rclcpp)

ament_package()
