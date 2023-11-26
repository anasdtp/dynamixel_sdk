cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(lib_dynamixel_DIR "/home/anas/ros2_humble/install/lib_dynamixel/share/lib_dynamixel/cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(lib_dynamixel REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

# Build
add_library(read_write_node SHARED
    include/read_write_node.h
    src/read_write_node.cpp
)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  lib_dynamixel
  rclcpp
)

add_executable(dynamixel_node 
  main.cpp
)
ament_target_dependencies(dynamixel_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  lib_dynamixel
  rclcpp
)

# Ajout de l'exécutable et de la bibliothèque dans le même répertoire
target_include_directories(dynamixel_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Ajout de la bibliothèque à l'exécutable
target_link_libraries(dynamixel_node read_write_node)

# Install
install(TARGETS
  dynamixel_node
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
