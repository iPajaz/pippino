cmake_minimum_required(VERSION 3.8)
project(is_aruco_visible_bt_condition_plugin)

set(lib_name nav2_is_aruco_visible_condition_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(aruco_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_interface
  std_srvs
  nav2_util
  pluginlib
)

add_library(${lib_name} SHARED
  src/is_aruco_visible_condition.cpp)

ament_target_dependencies(${lib_name} ${dependencies})
target_compile_definitions(${lib_name} PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(FILES aruco_visible_condition_node.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${lib_name}
)

ament_export_dependencies(${dependencies})

ament_package()
