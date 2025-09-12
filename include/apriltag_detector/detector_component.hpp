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

#ifndef APRILTAG_DETECTOR__DETECTOR_COMPONENT_HPP_
#define APRILTAG_DETECTOR__DETECTOR_COMPONENT_HPP_

#include <apriltag_detector/detector.hpp>
#include <image_transport/image_transport.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace apriltag_detector
{
using Image = sensor_msgs::msg::Image;
using svec = std::vector<std::string>;
using svecvec = std::vector<std::vector<std::string>>;
using Image = sensor_msgs::msg::Image;
using VecImagePtr = std::vector<Image::ConstSharedPtr>;
using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
using VecApriltagArrayPtr = std::vector<ApriltagArray::ConstSharedPtr>;

class DetectorComponent : public rclcpp::Node
{
public:
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  using Image = sensor_msgs::msg::Image;
  explicit DetectorComponent(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DetectorComponent();
  auto getNumMessages() const { return (num_messages_); }
  auto getNumTagsDetected() const { return (num_tags_detected_); }
  auto isSubscribed() const { return (is_subscribed_); }

private:
  void subscribe();
  void unsubscribe();
  void subscriptionCheckTimerExpired();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;
  rclcpp::Publisher<ApriltagArray>::SharedPtr detect_pub_;
  std::shared_ptr<image_transport::Subscriber> image_sub_;
  bool is_subscribed_{false};
  std::string image_qos_profile_{"default"};
  std::string in_transport_{"raw"};
  std::size_t num_messages_{0};
  std::size_t num_tags_detected_{0};
  pluginlib::ClassLoader<apriltag_detector::Detector> detector_loader_;
  std::shared_ptr<apriltag_detector::Detector> detector_;

  /* -------------------START Not from original Apriltag detector_component.hpp------------------------- */
  
  // Added Variables and Functions for drone swarm handling
  void setup_drone_subpub(int drone_id);
  void switch_drone(int drone_id);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void tag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
 
  int drone_count_; // Input parameter passed in to the constructor from the launch
  int current_drone_;
  int tag_id;
  nav_msgs::msg::Odometry::SharedPtr drone_odom_;
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr current_tag_;

  // ROS2 Subscribers 
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_; // Detect /tags message 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // For saving xyzrpy data
  
  // ROS2 Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scenario_detection_pub_; // Publish Scenario name, Severity Rating, Drone odom, Bool drone can respond? 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_; // Publish image topic of bottom camera when tag detected

  // msgs
  sensor_msgs::msg::Image::SharedPtr img_msg_;
  std_msgs::msg::String scenario_msg_;

  // To Switch topics
  std::string current_topic_img_;
  std::string current_topic_odom_;
  
  /* -------------------END Not from original Apriltag detector_component.hpp------------------------- */

};

}  // namespace apriltag_detector

#endif  // APRILTAG_DETECTOR__DETECTOR_COMPONENT_HPP_
