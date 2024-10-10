cmake_minimum_required(VERSION 3.8)
project(roboteq_bridge)

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

add_executable(base_controller_node src/base_controller_node.cpp)
ament_target_dependencies(base_controller_node serial rclcpp geometry_msgs nav_msgs)

install(
 TARGETS base_controller_node
 DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
