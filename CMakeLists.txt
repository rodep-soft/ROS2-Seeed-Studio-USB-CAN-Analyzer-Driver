cmake_minimum_required(VERSION 3.14)
project(seeed_usb_can_analyzer_driver)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
)

add_library(usb_can_serial_protocol
  src/serial_protocol.cpp
)
target_include_directories(usb_can_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(usb_can_serial_protocol Boost::headers)


add_executable(usb_can_analyzer_node
  src/usb_can_analyzer_node.cpp
)

target_include_directories(usb_can_analyzer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(usb_can_analyzer_node
  PUBLIC
    rclcpp::rclcpp
  PRIVATE
    Boost::headers
    usb_can_serial_protocol
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(usb_can_analyzer_node PRIVATE "${cpp_typesupport_target}")
add_dependencies(usb_can_analyzer_node ${PROJECT_NAME}__rosidl_generator_cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  usb_can_serial_protocol
  usb_can_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
