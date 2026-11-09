cmake_minimum_required(VERSION 3.8)
project(stonefish_tutorial VERSION 1.6.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)

# define libraries
add_library(sensor_plugin SHARED
  src/${PROJECT_NAME}/SensorPlugin.cpp
)
target_link_libraries(sensor_plugin
  stonefish_ros2::stonefish_ros2_interface
)
target_include_directories(sensor_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sensor_plugin
 rclcpp 
 ament_index_cpp 
 stonefish_ros2
 geometry_msgs
)
ament_export_libraries(sensor_plugin)
ament_export_include_directories(include)

# installation
install(
  TARGETS sensor_plugin
  EXPORT sensor_plugin_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
