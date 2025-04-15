cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(planner_msgs REQUIRED)

# Define the planner plugin library
add_library(vegmap_planner_plugin SHARED
  src/vegmap_planner.cpp
)

# Define the vegetation costmap layer library
add_library(veg_costmap_layer_plugin SHARED
  src/veg_costmap_layer.cpp
)

# Define the straight line planner library
add_library(straight_line_planner_plugin SHARED
  src/straight_line_planner.cpp
)



# Define dependencies
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  rosidl_default_runtime
  nav2_common
  tf2
  tf2_msgs
  planner_msgs
)


# Set dependencies for the planner plugin
ament_target_dependencies(vegmap_planner_plugin ${dependencies})

ament_target_dependencies(straight_line_planner_plugin ${dependencies})

# Link plugins with the planner_msgs library
target_link_libraries(vegmap_planner_plugin
  ${planner_msgs_TARGETS}
)
target_link_libraries(veg_costmap_layer_plugin
  ${planner_msgs_TARGETS}
)
target_link_libraries(straight_line_planner_plugin
  ${planner_msgs_TARGETS}
)
# Set proper include directories for plugins
target_include_directories(vegmap_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(straight_line_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Set dependencies for the vegetation costmap layer
ament_target_dependencies(veg_costmap_layer_plugin
  rclcpp
  nav2_common
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

target_include_directories(veg_costmap_layer_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Python executables
install(PROGRAMS
  src/pose_info_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
install(TARGETS 
  vegmap_planner_plugin
  veg_costmap_layer_plugin
  straight_line_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install plugin definitions
pluginlib_export_plugin_description_file(nav2_core vegmap_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d veg_costmap_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_plugin.xml)

# Install directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Ensure config and launch directories exist before installing
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/maps)
  install(DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install XML files
install(FILES 
  vegmap_planner_plugin.xml
  veg_costmap_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(vegmap_planner_plugin veg_costmap_layer_plugin)
ament_export_dependencies(${dependencies})
ament_package()