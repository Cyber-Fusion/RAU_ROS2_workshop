cmake_minimum_required(VERSION 3.8)
project(head_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(common_protocol REQUIRED)


add_library(head_node SHARED
  src/head_node.cpp
  include/head_package/head_node.hpp)

ament_target_dependencies(head_node
  "common_protocol"
  "rclcpp"
  "rclcpp_action"
  "std_msgs")

target_include_directories(head_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(head_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
  TARGETS head_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
