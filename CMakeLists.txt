# FIXED

cmake_minimum_required(VERSION 3.5)
project(rflex)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Set the build type. Options are:
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Add source files
set(SOURCES
  src/rflex_driver.cc
  src/atrvjr_driver.cc
  src/atrvjr_node.cc
  src/keyboard_control.cpp
)

# Add executable and link libraries
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

add_executable(atrvjr src/atrvjr_node.cc)
target_link_libraries(atrvjr ${PROJECT_NAME})

add_executable(keyboardControl src/keyboard_control.cpp)
target_link_libraries(keyboardControl ${PROJECT_NAME})

# Install
install(
  TARGETS ${PROJECT_NAME} atrvjr keyboardControl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
