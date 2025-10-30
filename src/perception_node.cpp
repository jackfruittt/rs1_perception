#include "perception_node.h"
#include <cmath>

namespace rs1_perception {

// Quality of Service Profile conversion utility
rmw_qos_profile_t string_to_profile(const std::string& profile_str) {
    if(profile_str == "sensor_data") {
        return rmw_qos_profile_sensor_data;
    }
    return rmw_qos_profile_default;
}

#ifdef IMAGE_TRANSPORT_USE_QOS
static rclcpp::QoS convert_profile(const rmw_qos_profile_t& profile) {
    return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
}
#else
static const rmw_qos_profile_t& convert_profile(const rmw_qos_profile_t& profile) {
    return profile;
}
#endif

// PerceptionNode Implementation
PerceptionNode::PerceptionNode(const rclcpp::NodeOptions& options)
    : Node("perception_node", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
    , detector_loader_("apriltag_detector", "apriltag_detector::Detector")
    , num_messages_(0)
    , num_tags_detected_(0)
    , is_subscribed_(false) {
    // Get drone namespace parameter (declared automatically from launch file)
    drone_namespace_ = this->get_parameter_or("drone_namespace", std::string("rs1_drone"));

    // Load perception and detector parameters
    loadPerceptionParams();

    // Initialise AprilTag detector
    const std::string detector_type = this->get_parameter_or("detector_type", std::string("umich"));
    RCLCPP_INFO(this->get_logger(), "Initialising AprilTag detector type: %s", detector_type.c_str());

    detector_ = detector_loader_.createSharedInstance("apriltag_detector_" + detector_type + "::Detector");

    // Configure detector parameters
    detector_->setFamily(get_parameter_or("tag_family", std::string("tf36h11")));
    detector_->setDecimateFactor(get_parameter_or("decimate_factor", 1.0));
    detector_->setQuadSigma(get_parameter_or("blur", 0.0));
    detector_->setNumberOfThreads(get_parameter_or("num_threads", 1));
    detector_->setMaxAllowedHammingDistance(get_parameter_or("max_allowed_hamming_distance", 0));
    detector_->setBlackBorder(get_parameter_or("black_border_width", 1));

    // Create ROS subscriptions using drone namespace pattern
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/" + drone_namespace_ + "/odom", 10, std::bind(&PerceptionNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                   "/" + drone_namespace_ + "/imu", 10, std::bind(&PerceptionNode::imuCallback, this, std::placeholders::_1));

    detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                         "/" + drone_namespace_ + "/tags", 10,
                         std::bind(&PerceptionNode::tagDetectionCallback, this, std::placeholders::_1));

    // Create publishers
    rclcpp::PublisherOptions pub_options;
#ifdef USE_MATCHED_EVENTS
    pub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & s) {
        if(is_subscribed_) {
            if(s.current_count == 0) {
                unsubscribe();
            }
        } else {
            if(s.current_count != 0) {
                subscribe();
            }
        }
    };
#endif

    detect_pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("/" + drone_namespace_ + "/tags", 10,
                                                                                     pub_options);

    scenario_detection_pub_ =
        this->create_publisher<std_msgs::msg::String>("/" + drone_namespace_ + "/scenario_detection", 10);

    scenario_image_pub_ = std::make_shared<image_transport::Publisher>(
                              image_transport::create_publisher(this, "/" + drone_namespace_ + "/scenario_img"));

#ifndef USE_MATCHED_EVENTS
    // Create subscription monitoring timer for early ROS2 versions
    subscription_check_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration(1, 0),
                                                     std::bind(&PerceptionNode::subscriptionCheckTimerExpired, this));
#endif

    // Initialise state tracking
    last_detection_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Perception Node initialised for %s", drone_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "AprilTag detector configured - Family: %s, Threads: %d",
                get_parameter_or("tag_family", std::string("tf36h11")).c_str(), get_parameter_or("num_threads", 1));
}

PerceptionNode::~PerceptionNode() {
    if(subscription_check_timer_) {
        subscription_check_timer_->cancel();
    }

    if(is_subscribed_) {
        unsubscribe();
    }

    RCLCPP_INFO(this->get_logger(), "Perception Node shutdown complete");
}

void PerceptionNode::loadPerceptionParams() {
    // Load core perception parameters (using get_parameter_or to avoid declaration conflicts)
    image_qos_profile_ = this->get_parameter_or("image_qos_profile", std::string("default"));
    in_transport_ = this->get_parameter_or("image_transport", std::string("raw"));
    front_camera_focal_length_ = this->get_parameter_or("front_camera_focal_length", 185.7);
    april_tag_size_ = this->get_parameter_or("april_tag_size", 1.0);
    position_tolerance_ = this->get_parameter_or("position_tolerance", 0.5);

    // Validate parameters
    if(front_camera_focal_length_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid focal length %.2f, using default 185.7", front_camera_focal_length_);
        front_camera_focal_length_ = 185.7;
    }

    RCLCPP_INFO(this->get_logger(), "Perception parameters loaded - Focal length: %.1fpx, Tag size: %.2fm",
                front_camera_focal_length_, april_tag_size_);
}

void PerceptionNode::subscribe() {
    if(is_subscribed_) {
        return;
    }

    const auto qos_profile = string_to_profile(image_qos_profile_);

    // Subscribe to front camera for AprilTag detection - Edited from front to bottom for Matt testing - can change back/refactor if needed
    front_image_sub_ = std::make_shared<image_transport::Subscriber>(
                           image_transport::create_subscription(this, "/" + drone_namespace_ + "/bottom/image",
                                                                std::bind(&PerceptionNode::frontImageCallback, this, std::placeholders::_1),
                                                                in_transport_, convert_profile(qos_profile)));

    is_subscribed_ = true;
    RCLCPP_INFO(this->get_logger(), "Subscribed to front camera image stream");
}

void PerceptionNode::unsubscribe() {
    if(!is_subscribed_) {
        return;
    }

    if(front_image_sub_) {
        front_image_sub_->shutdown();
    }

    if(bottom_image_sub_) {
        bottom_image_sub_->shutdown();
        bottom_image_sub_.reset();
    }

    is_subscribed_ = false;
    RCLCPP_INFO(this->get_logger(), "Unsubscribed from image streams");
}

void PerceptionNode::subscriptionCheckTimerExpired() {
    if(detect_pub_->get_subscription_count() > 0) {
        if(!is_subscribed_) {
            subscribe();
        }
    } else {
        if(is_subscribed_) {
            unsubscribe();
        }
    }
}

void PerceptionNode::frontImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    num_messages_++;

    if(detect_pub_->get_subscription_count() == 0) {
        return;
    }

    // Convert ROS image to OpenCV format
    cv_bridge::CvImageConstPtr cv_img;
    try {
        cv_img = cv_bridge::toCvShare(msg, "mono8");
    } catch(const cv_bridge::Exception& e) {
        if(msg->encoding == "8UC1") {
            // Handle incorrect encoding
            std::shared_ptr<sensor_msgs::msg::Image> img_copy(new sensor_msgs::msg::Image(*msg));
            img_copy->encoding = "mono8";
            cv_img = cv_bridge::toCvShare(img_copy, "mono8");
        }
    }

    if(!cv_img) {
        RCLCPP_WARN(this->get_logger(), "Cannot convert image to mono8 format");
        return;
    }

    // Perform AprilTag detection
    auto detection_array = detectAprilTags(cv_img->image, msg->header);

    if(!detection_array || detection_array->detections.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "No AprilTags detected in current frame");
        return;
    }

    // Update statistics and publish results
    num_tags_detected_ += detection_array->detections.size();
    last_detection_time_ = std::chrono::steady_clock::now();

    detect_pub_->publish(std::move(detection_array));

    RCLCPP_DEBUG(this->get_logger(), "Processed frame - Total messages: %zu, Total tags: %zu", num_messages_,
                 num_tags_detected_);
}
void PerceptionNode::bottomImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // Publish scenario documentation image
    scenario_image_pub_->publish(msg);

    // Unsubscribe after capturing one frame to avoid continuous streaming
    if(bottom_image_sub_) {
        bottom_image_sub_->shutdown();
        bottom_image_sub_.reset();
    }

    RCLCPP_DEBUG(this->get_logger(), "Captured and published scenario image from bottom camera");
}

void PerceptionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_odom_ = msg;

    RCLCPP_DEBUG(this->get_logger(), "Odometry updated - Position: [%.2f, %.2f, %.2f]", msg->pose.pose.position.x,
                 msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void PerceptionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_imu_ = msg;

    RCLCPP_DEBUG(this->get_logger(), "IMU data updated");
}

void PerceptionNode::tagDetectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_tags_ = msg;

    if(!current_odom_ || !current_imu_ || msg->detections.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Insufficient data for scenario processing");
        return;
    }

    // Process first detected tag for scenario recognition
    const auto& first_detection = msg->detections[0];
    int tag_id = first_detection.id;

    // Convert tag ID to scenario enum
    if(tag_id < 0 || tag_id > 2) {
        RCLCPP_WARN(this->get_logger(), "Unknown tag ID: %d", tag_id);
        return;
    }

    Scenario detected_scenario = static_cast<Scenario>(tag_id);

    // Calculate drone orientation from IMU
    tf2::Quaternion q(current_imu_->orientation.x, current_imu_->orientation.y, current_imu_->orientation.z,
                      current_imu_->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Estimate depth to target using corner geometry
    std::vector<cv::Point2f> corners;
    for(const auto& corner : first_detection.corners) {
        corners.emplace_back(corner.x, corner.y);
    }

    double depth_estimate = estimateTagDepth(corners);

    // Calculate adjusted target position in world coordinates
    geometry_msgs::msg::Point adjusted_position =
        calculateAdjustedPosition(current_odom_->pose.pose.position, yaw, depth_estimate);

    // Publish scenario detection
    publishScenarioDetection(detected_scenario, adjusted_position, yaw);

    // Subscribe to bottom camera for scenario documentation
    const auto qos_profile = string_to_profile(image_qos_profile_);
    bottom_image_sub_ = std::make_shared<image_transport::Subscriber>(
                            image_transport::create_subscription(this, "/" + drone_namespace_ + "/bottom/image",
                                                                 std::bind(&PerceptionNode::bottomImageCallback, this, std::placeholders::_1),
                                                                 "raw", convert_profile(qos_profile)));

    RCLCPP_INFO(this->get_logger(), "Scenario detected: %s at position [%.2f, %.2f, %.2f]",
                scenarioToString(detected_scenario).c_str(), adjusted_position.x, adjusted_position.y,
                adjusted_position.z);
}

apriltag_msgs::msg::AprilTagDetectionArray::UniquePtr
PerceptionNode::detectAprilTags(const cv::Mat& cv_img, const std_msgs::msg::Header& header) {
    auto detection_array = std::make_unique<apriltag_msgs::msg::AprilTagDetectionArray>();
    detection_array->header = header;

    // Perform detection using configured detector
    detector_->detect(cv_img, detection_array.get());

    return detection_array;
}

double PerceptionNode::estimateTagDepth(const std::vector<cv::Point2f>& tag_corners) const {
    if(tag_corners.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Insufficient corner points for depth estimation");
        return 0.0;
    }

    // Calculate depth using focal length and tag corner height
    // Assumes AprilTag is 1m x 1m physical size
    double pixel_height = std::abs(tag_corners[0].y - tag_corners[3].y);

    if(pixel_height <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid pixel height for depth calculation");
        return 0.0;
    }

    double depth = (front_camera_focal_length_ * april_tag_size_) / pixel_height;

    RCLCPP_DEBUG(this->get_logger(), "Estimated tag depth: %.2fm (pixel height: %.1fpx)", depth, pixel_height);

    return depth;
}

geometry_msgs::msg::Point PerceptionNode::calculateAdjustedPosition(const geometry_msgs::msg::Point& drone_pos,
                                                                    double yaw_angle, double depth_estimate) const {
    geometry_msgs::msg::Point adjusted_pos = drone_pos;

    const double tolerance = position_tolerance_;

    // Compensate for bottom camera 45deg downward angle
    adjusted_pos.z -= depth_estimate * std::sin(M_PI / 4);


    // Adjust position based on drone heading (yaw angle)
    if(std::abs(yaw_angle - 0.0) <= tolerance) {
        // Facing positive X direction
        adjusted_pos.x += depth_estimate * std::cos(M_PI / 4);
    } else if(std::abs(yaw_angle - M_PI) <= tolerance || std::abs(yaw_angle + M_PI) <= tolerance) {
        // Facing negative X direction
        adjusted_pos.x -= depth_estimate * std::cos(M_PI / 4);
    } else if(std::abs(yaw_angle - M_PI_2) <= tolerance) {
        // Facing positive Y direction
        adjusted_pos.y += depth_estimate * std::cos(M_PI / 4);
    } else if(std::abs(yaw_angle + M_PI_2) <= tolerance) {
        // Facing negative Y direction
        adjusted_pos.y -= depth_estimate * std::cos(M_PI / 4);
    } else {
        // General case - project based on yaw angle
        adjusted_pos.x += depth_estimate * std::cos(yaw_angle);
        adjusted_pos.y += depth_estimate * std::sin(yaw_angle);
    }

    RCLCPP_DEBUG(this->get_logger(), "Position adjusted from [%.2f, %.2f] to [%.2f, %.2f] (yaw: %.2f, depth: %.2f)",
                 drone_pos.x, drone_pos.y, adjusted_pos.x, adjusted_pos.y, yaw_angle, depth_estimate);

    return adjusted_pos;
}

void PerceptionNode::publishScenarioDetection(Scenario scenario_type, const geometry_msgs::msg::Point& target_position,
                                              double drone_heading) {
    std_msgs::msg::String scenario_msg;

    // Format: "SCENARIO_NAME,x,y,z,heading,can_respond"
    scenario_msg.data = scenarioToString(scenario_type) + "," + std::to_string(target_position.x) + "," +
                        std::to_string(target_position.y) + "," + std::to_string(target_position.z) + "," +
                        std::to_string(drone_heading) + "," +
                        "respond:1";  // TODO: Implement response capability assessment

    scenario_detection_pub_->publish(scenario_msg);

    RCLCPP_INFO(this->get_logger(), "Published scenario detection: %s", scenario_msg.data.c_str());
}

std::string PerceptionNode::scenarioToString(Scenario scenario) const {
    switch(scenario) {
    case Scenario::STRANDED_HIKER:
        return "STRANDED_HIKER";
    case Scenario::WILDFIRE:
        return "WILDFIRE";
    case Scenario::DEBRIS_OBSTRUCTION:
        return "DEBRIS_OBSTRUCTION";
    default:
        return "UNKNOWN_SCENARIO";
    }
}

rmw_qos_profile_t PerceptionNode::stringToQosProfile(const std::string& profile_str) const {
    return string_to_profile(profile_str);
}

}  // namespace rs1_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rs1_perception::PerceptionNode)