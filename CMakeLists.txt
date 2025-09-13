cmake_minimum_required(VERSION 3.8)
project(drone_apriltag_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Create executable - updated name to match new approach
add_executable(apriltag_detector_node src/apriltag_node.cpp)

# Include directories
target_include_directories(apriltag_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(apriltag_detector_node
  ${OpenCV_LIBRARIES}
)

# Specify dependencies
ament_target_dependencies(apriltag_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Specify C++ standard
target_compile_features(apriltag_detector_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS apriltag_detector_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()