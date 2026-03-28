cmake_minimum_required(VERSION 3.8)
project(sima_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(lib_dep_pkgs
rclcpp
nav2_costmap_2d
pluginlib
geometry_msgs
std_srvs)

set(sim_dep_pkgs
rclcpp
nav_msgs
std_msgs)

set(bridge_dep_pkgs
rclcpp
geometry_msgs
nav_msgs
std_msgs)

add_executable(SimaSim
  src/SimaSim.cpp)

add_library(sima_layer SHARED
  plugin/sima_layer.cpp)

target_link_libraries(sima_layer yaml-cpp)

include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d sima_layer.xml)
ament_target_dependencies(sima_layer ${lib_dep_pkgs})
ament_target_dependencies(SimaSim ${sim_dep_pkgs})

install(TARGETS
  sima_layer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  SimaSim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
