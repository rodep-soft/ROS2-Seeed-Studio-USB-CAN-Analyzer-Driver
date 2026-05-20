cmake_minimum_required(VERSION 3.14)
project(seeed_usb_can_analyzer_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(usb_can_analyzer_node
  src/usb_can_analyzer_node.cpp
)

target_link_libraries(usb_can_analyzer_node Boost::system)
ament_target_dependencies(usb_can_analyzer_node rclcpp)

install(TARGETS usb_can_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
