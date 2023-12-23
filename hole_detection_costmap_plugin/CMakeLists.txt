cmake_minimum_required(VERSION 3.5)
project(hole_detection_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
  rclcpp
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  sensor_msgs
  laser_geometry
  message_filters
  nav2_voxel_grid
  pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/voxel_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs voxel_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d voxel_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
