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

/*
    Customising Code to adapt for multiple image streams
*/

#include <apriltag_detector/detector_component.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp> 
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace apriltag_detector
{
// Detector Constructor
DetectorComponent::DetectorComponent(const rclcpp::NodeOptions & options)
: Node("detector", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)), detector_loader_("apriltag_detector", "apriltag_detector::Detector")
{
  // Adjust this function to handle multiple drones
  setup_drone_subpub(1);

  // Detector type parameter
  const std::string type = this->get_parameter_or("type", std::string("umich"));
  RCLCPP_INFO_STREAM(get_logger(), "tag detector type: " << type);

  // Store type
  detector_ = detector_loader_.createSharedInstance(
    "apriltag_detector_" + type + "::Detector");

  // Set Detector Tag Family = 36h11 default unless passed otherwise
  detector_->setFamily(get_parameter_or("tag_family", std::string("tf36h11")));

  // Set input transport = raw default unless passed otherwise
  in_transport_ = get_parameter_or("image_transport", std::string("raw"));

  // Quality of Service Profile
  get_parameter_or(
    "image_qos_profile", image_qos_profile_, std::string("default"));

  // only supported by the UMich detector
  detector_->setDecimateFactor(get_parameter_or("decimate_factor", 1.0));
  detector_->setQuadSigma(get_parameter_or("blur", 0.0));
  detector_->setNumberOfThreads(get_parameter_or("num_threads", 1));
  detector_->setMaxAllowedHammingDistance(
    get_parameter_or("max_allowed_hamming_distance", 0));
  // only supported by the MIT detector
  detector_->setBlackBorder(get_parameter_or("black_border_width", 1));

  // Adjust publisher options if needed
  rclcpp::PublisherOptions pub_options;
#ifdef USE_MATCHED_EVENTS
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo & s) {
      if (is_subscribed_) {
        if (s.current_count == 0) {
          unsubscribe();
        }
      } else {
        if (s.current_count != 0) {
          subscribe();
        }
      }
    };
#endif

  // tag publisher, custom message = AprilTagArray
  detect_pub_ = this->create_publisher<ApriltagArray>(
    "/rs1_drone_1/tags", rclcpp::QoS(100), pub_options);

#ifndef USE_MATCHED_EVENTS
  // Since early ROS2 does not call back when subscribers come and go
  // must check by polling
  subscription_check_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&DetectorComponent::subscriptionCheckTimerExpired, this));
#endif
}

// Deconstructor, destroys subscriptions
DetectorComponent::~DetectorComponent()
{
  if (subscription_check_timer_) {
    subscription_check_timer_->cancel();
  }
}

// Quality of Service Profile
rmw_qos_profile_t string_to_profile(const std::string & s)
{
  if (s == "sensor_data") {
    return (rmw_qos_profile_sensor_data);
  }
  return (rmw_qos_profile_default);
}


#ifdef IMAGE_TRANSPORT_USE_QOS
static rclcpp::QoS convert_profile(const rmw_qos_profile_t & p)
{
  return (rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(p), p));
}
#else
static const rmw_qos_profile_t & convert_profile(const rmw_qos_profile_t & p)
{
  return (p);
}
#endif

// Subscribe to image topic -- Original 
void DetectorComponent::subscribe()
{
  const auto profile = string_to_profile(image_qos_profile_);
    
  // Subscriber for 1 drone -- THIS WORKS, adjust this for multiple drones -- maybe for loop going through each drone id and have a delay?
    image_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
            this,"/rs1_drone_1/front/image",std::bind(&DetectorComponent::callback, this, std::placeholders::_1),
            in_transport_, convert_profile(profile)
        )
    );

    // image_sub_ = std::make_shared<image_transport::Subscriber>(
    //     image_transport::create_subscription(
    // #ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
    //     *this,
    // #else
    //     this,
    // #endif
    //     "image",
    //     std::bind(&DetectorComponent::callback, this, std::placeholders::_1),
    //     in_transport_, convert_profile(profile)));
  is_subscribed_ = true;
}

void DetectorComponent::unsubscribe()
{
  image_sub_->shutdown();
  is_subscribed_ = false;
}

void DetectorComponent::subscriptionCheckTimerExpired()
{
  if (detect_pub_->get_subscription_count()) {
    // -------------- subscribers ---------------------
    if (!is_subscribed_) {
      subscribe();
    }
  } else {
    // -------------- no subscribers -------------------
    if (is_subscribed_) {
      unsubscribe();
    }
  }
}

/*
  This image callback handles the apriltag detection using apriltag_detector
  It subscribes to the front camera of the drone -- Check subscribe function
*/
void DetectorComponent::callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  num_messages_++;
  if (detect_pub_->get_subscription_count() != 0) {
    cv_bridge::CvImageConstPtr cvImg;
    try {
      cvImg = cv_bridge::toCvShare(msg, "mono8");
    } catch (const cv_bridge::Exception & e) {
      if (msg->encoding == "8UC1") {
        // hack to muddle on when encoding is wrong
        std::shared_ptr<Image> img_copy(new Image(*msg));
        img_copy->encoding = "mono8";
        cvImg = cv_bridge::toCvShare(img_copy, "mono8");
      }
    }
    if (!cvImg) {
      RCLCPP_WARN(get_logger(), "cannot convert image to mono!");
      return;
    }
    // Create apriltag array message
    auto array_msg = std::make_unique<apriltag_msgs::msg::AprilTagDetectionArray>();

    // Detect tag
    detector_->detect(cvImg->image, array_msg.get());

    /* ------------------- Estimate Scenario Tag Depth START --------------------------------*/
    // Do "Scenario Location" Estimation Here using corners from array message
    // From AprilTagDetection.msg
    // Point[4] corners                # corners of tag ((x1,y1),(x2,y2),...)
    if(array_msg && !array_msg->detections.empty())
    {
      std::vector<cv::Point2f> corner_points;
      for (int i = 0; i < 4; i++)
      {
        cv::Point2f corner(
            array_msg->detections[0].corners[i].x,
            array_msg->detections[0].corners[i].y
        );
        corner_points.push_back(corner);
      }
      if(!corner_points.empty())
      {

        for (size_t i = 0; i < corner_points.size(); i++) 
        {
          /*
            focal_frontcam = 185.7px
            z_depth = f/y1 - y4, since cube is 1x1x1m H = 1m therefore its just z_depth=f/(y1-y4)
          */
          z_depth = 185.7/(corner_points[0].y - corner_points[3].y);
          
          // Debug z_depth
          // RCLCPP_INFO_STREAM(this->get_logger(), "z_depth: " << z_depth << '\n');
        }
      }
    }
    /* ------------------- Estimate Scenario Tag Depth END --------------------------------*/

    // Update message and publish
    array_msg->header = msg->header;
    num_tags_detected_ += array_msg->detections.size();
    detect_pub_->publish(std::move(array_msg));
    }
}
 
// Additional Multidrone functions
void DetectorComponent::setup_drone_subpub(int drone_id)
{
    current_topic_odom_ = "/rs1_drone_" + std::to_string(drone_id) + "/odom";
    current_topic_imu_ = "/rs1_drone_" + std::to_string(drone_id) + "/imu";
    
    // Create/recreate publishers and subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        current_topic_odom_, 10, std::bind(&DetectorComponent::odom_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        current_topic_imu_, 10, std::bind(&DetectorComponent::imu_callback, this, std::placeholders::_1));
    
    // TODO: Adjust for multidrone use
    detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/rs1_drone_1/tags",10,std::bind(&DetectorComponent::tag_callback, this, std::placeholders::_1));
  
    // TODO: Adjust for multidrone use
    scenario_detection_pub_ = this->create_publisher<std_msgs::msg::String>("/rs1_drone_1/scenario_detection",10);
    
    // Adjust for multi_drone spawn -- This is to send the image of the scenario captured
    scenario_image_pub_ = std::make_shared<image_transport::Publisher>(
        image_transport::create_publisher(this, "/rs1_drone_1/scenario_img"));
    
}

void DetectorComponent::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    drone_odom_ = msg;

    // Added Debug message for drone_odom_, orientation xyzw is all 0?
    // RCLCPP_INFO_STREAM(this->get_logger(), 
    // "Received odom - Position: (" << 
    // msg->pose.pose.position.x << ", " <<
    // msg->pose.pose.position.y << ", " <<
    // msg->pose.pose.position.z << ") Orientation: (" <<
    // msg->pose.pose.orientation.x << ", " <<
    // msg->pose.pose.orientation.y << ", " <<
    // msg->pose.pose.orientation.z << ", " <<
    // msg->pose.pose.orientation.w << ")");
}

void DetectorComponent::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    drone_imu_ = msg;
    // RCLCPP_INFO_STREAM(this->get_logger(), 
    // "Received imu - Orientation: (" <<
    // msg->orientation.x << ", " <<
    // msg->orientation.y << ", " <<
    // msg->orientation.z << ", " <<
    // msg->orientation.w << ")");
}

// Callback function to get tag id based on what is detected
void DetectorComponent::tag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    current_tag_ = msg;

    // rpy values for orientation
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;  

    double tolerance = 0.5;
    float adjusted_x = drone_odom_->pose.pose.position.x; // These gets adjusted below based on yaw
    float adjusted_y = drone_odom_->pose.pose.position.y;

    // Get drone yaw
    if (drone_imu_) {
        tf2::Quaternion q(
            drone_imu_->orientation.x,
            drone_imu_->orientation.y,
            drone_imu_->orientation.z,
            drone_imu_->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
    } else {
        RCLCPP_WARN(this->get_logger(), "No odometry data available");
    }

    // TO-DO: Tidy this up if possible, rn just hardcoded values 
    // Apply z_depth to either x or y position based on drone yaw
    if(yaw >= 0-tolerance && yaw <= 0+tolerance) adjusted_x = drone_odom_->pose.pose.position.x + z_depth;
    if(yaw >= 3.14-tolerance && yaw <= 3.14+tolerance) adjusted_x = drone_odom_->pose.pose.position.x - z_depth; 
    if(yaw >= 1.57-tolerance && yaw <= 1.57+tolerance) adjusted_y = drone_odom_->pose.pose.position.y + z_depth; 
    if(yaw >= -1.57-tolerance && yaw <= -1.57+tolerance) adjusted_y = drone_odom_->pose.pose.position.y - z_depth; 

    // Check if there are any detections
    if (!current_tag_->detections.empty()) {
        // Access the first detection's id
        tag_id = current_tag_->detections[0].id;
        Scenario scenario = static_cast<Scenario>(tag_id);
        const char* scenarioStr = ScenarioToString(scenario);

        scenario_msg_.data = std::string(scenarioStr) + "," + 
                             std::to_string(adjusted_x) + "," + 
                             std::to_string(adjusted_y) + "," +
                             std::to_string(drone_odom_->pose.pose.position.z) + "," +
                             std::to_string(yaw) + "," +
                             "respond:1/0"; // Can drone respond? true or false (1/0)

        // Show what bottom camera sees when "scenario" is detected -- Adjust for multidrone use
        scenario_image_sub_ = std::make_shared<image_transport::Subscriber>(
            image_transport::create_subscription(
                this, "/rs1_drone_1/bottom/image", 
                std::bind(&DetectorComponent::image_callback, this, std::placeholders::_1),
                "raw"));
        
        scenario_detection_pub_->publish(scenario_msg_); // Publish tag id detected
    }
}

// Bottom Image View Callback
void DetectorComponent::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // Publish last received image message from drone bottom camera
  scenario_image_pub_->publish(msg);

  // Unsubscribe from bottom drone camera to avoid sending unrelated image stream 
  scenario_image_sub_->shutdown();
}

// Add more Scenarios Here depending on environment and tag id
const char* DetectorComponent::ScenarioToString(Scenario scenario){
    switch (scenario) {
        case STRANDED_HIKER:
            return "STRANDED_HIKER";
        case WILDFIRE:
            return "WILDFIRE";
        case DEBRIS_OBSTRUCTION:
            return "DEBRIS_OBSTRUCTION";
        default:
            return "UNKNOWN";
    }
}

}// namespace apriltag_detector

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector::DetectorComponent)