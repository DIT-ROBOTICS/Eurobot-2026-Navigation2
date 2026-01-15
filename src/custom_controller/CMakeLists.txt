cmake_minimum_required(VERSION 3.8)
project(custom_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dep_pkgs
    rclcpp
    rclcpp_lifecycle
    nav2_core
    nav2_costmap_2d
    pluginlib
    nav_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(custom_controller SHARED
  src/custom_controller.cpp
)

add_library(teb_controller SHARED
  src/teb_controller.cpp
)

target_link_libraries(custom_controller yaml-cpp)
target_link_libraries(teb_controller yaml-cpp)

ament_target_dependencies(custom_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)
ament_target_dependencies(teb_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

target_include_directories(custom_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(teb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS custom_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS teb_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core resource/custom.xml)
pluginlib_export_plugin_description_file(nav2_core resource/teb.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
