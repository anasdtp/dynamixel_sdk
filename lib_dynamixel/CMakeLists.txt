cmake_minimum_required(VERSION 3.5)
project(lib_dynamixel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable)
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)



################################################################################
# Build
################################################################################
include_directories(
  include/${lib_dynamixel}
)

set(SOURCES
  src/lib_dynamixel.cpp
)


add_library(lib_dynamixel SHARED
    include/lib_dynamixel.h
    ${SOURCES}
)

ament_target_dependencies(lib_dynamixel

  dynamixel_sdk
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
