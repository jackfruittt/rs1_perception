// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file perception_node.h
 * @brief AprilTag detection and scenario recognition node for autonomous drone operations
 * 
 * This file contains heavily modified code originally from apriltag_detector.
 * Modifications made to support custom drone swarm operations.
 * Original copyright retained above.
 * 
 * @author Ace Viray
 * @author Jackson Russell
 * ADD AUTHORS HERE AND BELOW
 * @date Sep-2025
 */

#ifndef RS1_PERCEPTION_PERCEPTION_NODE_H_
#define RS1_PERCEPTION_PERCEPTION_NODE_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "apriltag_detector/detector.hpp"

namespace rs1_perception
{

/**
 * @enum Scenario
 * @brief Emergency scenario enumeration for detected AprilTags
 * 
 * Defines various emergency scenarios that can be detected via AprilTag IDs.
 * Each scenario corresponds to a specific tag ID number.
 */
enum class Scenario
{
  STRANDED_HIKER = 0,    ///< Person requiring rescue assistance
  WILDFIRE = 1,          ///< Fire hazard requiring immediate response
  DEBRIS_OBSTRUCTION = 2 ///< Blocked path or debris field
};

/**
 * @class PerceptionNode
 * @brief ROS 2 node for AprilTag detection and emergency scenario recognition
 * 
 * Manages computer vision-based perception for drone operations including
 * AprilTag detection, scenario classification, depth estimation, and
 * multi-camera image processing for emergency response missions.
 */
class PerceptionNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor with configurable node options
   * @param options ROS 2 node options for composition and configuration
   */
  explicit PerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor - cleans up subscriptions and timers
   */
  ~PerceptionNode();

  // Getter methods for monitoring
  /**
   * @brief Get total number of processed images
   * @return Number of images processed since node startup
   */
  auto getNumMessages() const { return num_messages_; }
  
  /**
   * @brief Get total number of detected AprilTags
   * @return Number of tags detected since node startup
   */
  auto getNumTagsDetected() const { return num_tags_detected_; }
  
  /**
   * @brief Check if node is actively subscribed to image topics
   * @return True if subscribed to image streams
   */
  auto isSubscribed() const { return is_subscribed_; }

private:
  // Core perception callback methods
  /**
   * @brief Process incoming images from front camera for AprilTag detection
   * @param msg Image message from camera sensor
   */
  void frontImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  
  /**
   * @brief Process images from bottom camera for scenario documentation
   * @param msg Image message from bottom-facing camera
   */
  void bottomImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  
  /**
   * @brief Process odometry updates for position-based depth calculations
   * @param msg Odometry message with drone position and velocity
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  /**
   * @brief Process IMU data for orientation estimation
   * @param msg IMU sensor data for roll, pitch, yaw calculations
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  /**
   * @brief Process detected AprilTag arrays for scenario recognition
   * @param msg Array of detected AprilTags with pose information
   */
  void tagDetectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

  // Subscription management methods
  /**
   * @brief Subscribe to image topics when publishers are available
   */
  void subscribe();
  
  /**
   * @brief Unsubscribe from image topics to save resources
   */
  void unsubscribe();
  
  /**
   * @brief Timer callback to check for active subscribers
   */
  void subscriptionCheckTimerExpired();

  // Core perception processing methods
  /**
   * @brief Perform AprilTag detection on input image
   * @param cv_img OpenCV image for tag detection
   * @param header ROS message header for timestamp/frame info
   * @return Array of detected AprilTags
   */
  apriltag_msgs::msg::AprilTagDetectionArray::UniquePtr detectAprilTags(
      const cv::Mat& cv_img, const std_msgs::msg::Header& header);
  
  /**
   * @brief Estimate depth/distance to detected AprilTag using corner geometry
   * @param tag_corners Vector of tag corner points in image coordinates
   * @return Estimated depth in metres
   */
  double estimateTagDepth(const std::vector<cv::Point2f>& tag_corners) const;
  
  /**
   * @brief Calculate adjusted position coordinates based on drone orientation
   * @param drone_pos Current drone position
   * @param yaw_angle Drone heading angle in radians
   * @param depth_estimate Estimated distance to target
   * @return Adjusted target coordinates in world frame
   */
  geometry_msgs::msg::Point calculateAdjustedPosition(
      const geometry_msgs::msg::Point& drone_pos, 
      double yaw_angle, 
      double depth_estimate) const;
  
  /**
   * @brief Publish scenario detection information to mission planner
   * @param scenario_type Detected emergency scenario
   * @param target_position Calculated target position in world coordinates
   * @param drone_heading Current drone orientation
   */
  void publishScenarioDetection(
      Scenario scenario_type,
      const geometry_msgs::msg::Point& target_position,
      double drone_heading);

  // Utility methods
  /**
   * @brief Convert scenario enum to human-readable string
   * @param scenario Scenario enumeration value
   * @return Australian English scenario description
   */
  std::string scenarioToString(Scenario scenario) const;
  
  /**
   * @brief Load perception parameters from ROS parameter server
   */
  void loadPerceptionParams();
  
  /**
   * @brief Convert QoS profile string to RMW profile
   * @param profile_str String representation of QoS profile
   * @return RMW QoS profile configuration
   */
  rmw_qos_profile_t stringToQosProfile(const std::string& profile_str) const;

  // ROS 2 communication interfaces
  // Subscriber interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                    ///< Drone odometry subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                       ///< IMU data subscription
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_; ///< AprilTag detection subscription

  // Image transport subscriptions
  std::shared_ptr<image_transport::Subscriber> front_image_sub_;                         ///< Front camera subscription
  std::shared_ptr<image_transport::Subscriber> bottom_image_sub_;                        ///< Bottom camera subscription (dynamic)

  // Publisher interfaces
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detect_pub_;  ///< AprilTag detection publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scenario_detection_pub_;           ///< Scenario detection publisher
  std::shared_ptr<image_transport::Publisher> scenario_image_pub_;                       ///< Scenario image publisher

  // Timer for subscription management
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;                                ///< Subscription monitoring timer

  // AprilTag detection components
  pluginlib::ClassLoader<apriltag_detector::Detector> detector_loader_;                 ///< Plugin loader for detector
  std::shared_ptr<apriltag_detector::Detector> detector_;                                ///< AprilTag detector instance

  // Current state variables
  nav_msgs::msg::Odometry::SharedPtr current_odom_;                                      ///< Latest odometry data
  sensor_msgs::msg::Imu::SharedPtr current_imu_;                                         ///< Latest IMU data
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr current_tags_;                   ///< Latest tag detections
  
  // Detection statistics
  std::size_t num_messages_;                                                             ///< Total processed images
  std::size_t num_tags_detected_;                                                        ///< Total detected tags
  bool is_subscribed_;                                                                   ///< Subscription status

  // Configuration parameters
  std::string drone_namespace_;                                                          ///< ROS namespace for this drone
  std::string image_qos_profile_;                                                        ///< Image transport QoS profile
  std::string in_transport_;                                                             ///< Input image transport type
  double front_camera_focal_length_;                                                     ///< Front camera focal length (pixels)
  double april_tag_size_;                                                                ///< Physical AprilTag size (metres)
  double position_tolerance_;                                                            ///< Position calculation tolerance
  
  // Thread safety
  std::mutex state_mutex_;                                                               ///< Mutex for thread-safe state access
  std::chrono::steady_clock::time_point last_detection_time_;                            ///< Last detection timestamp
};

}  // namespace rs1_perception

#endif  // RS1_PERCEPTION_PERCEPTION_NODE_H_