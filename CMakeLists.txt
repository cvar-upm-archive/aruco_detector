cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME aruco_detector)
project(${PROJECT_NAME} VERSION 0.1.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
ament_cmake
rclcpp
as2_core
as2_msgs
std_msgs
std_srvs
sensor_msgs
geometry_msgs
nav_msgs
cv_bridge
tf2
tf2_ros
tf2_geometry_msgs
yaml-cpp
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


include_directories(
  include
  include/${PROJECT_NAME}
)

set(HEADER_HPP_FILES 
  include/${PROJECT_NAME}/aruco_detector.hpp  
)

set(SOURCE_CPP_FILES 
  src/aruco_detector.cpp
)

add_executable(${PROJECT_NAME}_node src/aruco_detector_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

include(tests/profiling_cmake.cmake)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
