cmake_minimum_required(VERSION 3.5)
project(gazebo_vegetation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install project model files
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Make sure the models are visible to Gazebo
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_vegetation.dsv.in)

if(BUILD_TESTING)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()