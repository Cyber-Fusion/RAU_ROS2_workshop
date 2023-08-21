cmake_minimum_required(VERSION 3.8)
project(leg_package)

set(LIBNAME ${PROJECT_NAME}_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_protocol REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${LIBNAME} SHARED
  "src/leg_node.cpp"
  "src/exec.cpp"
  "include/leg_node/leg_node.hpp")

ament_target_dependencies(${LIBNAME}
  "rclcpp"
  "common_protocol"
  "rclcpp_action"
  "rclcpp_components")

target_include_directories(${LIBNAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${LIBNAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rclcpp_components_register_node(${LIBNAME} PLUGIN "LegNode" EXECUTABLE leg_node)

install(TARGETS ${LIBNAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS leg_node
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
