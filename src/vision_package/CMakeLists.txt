cmake_minimum_required(VERSION 3.8)
project(vision_package)

set(LIBNAME ${PROJECT_NAME}_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${LIBNAME} SHARED
  "src/vision_node.cpp"
  "src/exec.cpp"
  "include/vision_package/vision_node.hpp")

ament_target_dependencies(${LIBNAME}
  "rclcpp"
  "std_msgs"
  "rclcpp_components")

target_include_directories(${LIBNAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${LIBNAME} PLUGIN "VisionNode" EXECUTABLE vision_node)

install(TARGETS ${LIBNAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS vision_node
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
