cmake_minimum_required(VERSION 3.8)
project(rs1_perception)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# START ---- From Apriltag CMakeLists.txt
# Dependencies from apriltag_detector
find_package(apriltag REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib)
if(${rclcpp_VERSION} VERSION_GREATER_EQUAL "28.0.0")
add_definitions(-DUSE_MATCHED_EVENTS)
endif()
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.3.0")
add_definitions(-DIMAGE_TRANSPORT_USE_QOS)
endif()
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.4.0")
add_definitions(-DIMAGE_TRANSPORT_USE_NODEINTERFACE)
endif()
include_directories(
include
${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Adjusted for rs1_perception
add_library(${PROJECT_NAME} SHARED src/perception_node.cpp)
target_link_libraries(${PROJECT_NAME} ${sensor_msgs_TARGETS}
${apriltag_msgs_TARGETS}
image_transport::image_transport
cv_bridge::cv_bridge
pluginlib::pluginlib
rclcpp_components::component)

# Adjusted for rs1_perception
target_include_directories(
${PROJECT_NAME}
PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

# Adjusted for rs1_perception
rclcpp_components_register_nodes(${PROJECT_NAME}
"${PROJECT_NAME}::DetectorComponent")

# Adjusted for rs1_perception
target_link_libraries(rs1_perception
${sensor_msgs_TARGETS}
${apriltag_msgs_TARGETS}
apriltag::apriltag
image_transport::image_transport
cv_bridge::cv_bridge
pluginlib::pluginlib
rclcpp_components::component)

# END ---- From Apriltag CMakeLists.txt
ament_target_dependencies(rs1_perception
"rclcpp"
"sensor_msgs"
"geometry_msgs"
"std_msgs"
"nav_msgs"
"tf2_geometry_msgs"
"apriltag"
"apriltag_msgs"
"cv_bridge"
"image_transport"
"pluginlib"
"tf2"
)

# NODE ---- Adjusted for rs1_perception, but is basically AprilTag_Detector
add_executable(perception_node src/main.cpp)
target_link_libraries(perception_node rs1_perception)
ament_target_dependencies(perception_node
"rclcpp"
"std_msgs"
"geometry_msgs"
"sensor_msgs"
"nav_msgs"
"tf2_geometry_msgs"
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
perception_node
DESTINATION lib/${PROJECT_NAME})

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()
ament_package()