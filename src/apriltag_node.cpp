#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// You'll need to include the actual AprilTag detection library here
// This depends on which AprilTag library you want to use

class AprilTagDetectorNode : public rclcpp::Node
{
public:
    AprilTagDetectorNode() : Node("apriltag_detector")
    {
        // Declare parameters
        this->declare_parameter("drone_namespace", "rs1_drone");
        this->declare_parameter("drone_id", 1);
        this->declare_parameter("debug", false);
        this->declare_parameter("tag_family", "tag36h11");
        this->declare_parameter("tag_size", 0.2);

        // Get parameters
        std::string drone_namespace = this->get_parameter("drone_namespace").as_string();
        int drone_id = this->get_parameter("drone_id").as_int();
        debug_mode_ = this->get_parameter("debug").as_bool();
        std::string tag_family = this->get_parameter("tag_family").as_string();
        tag_size_ = this->get_parameter("tag_size").as_double();

        // Create topic names
        std::string camera_topic = "/" + drone_namespace + "_" + std::to_string(drone_id) + "/front/image";
        std::string output_topic = "/" + drone_namespace + "_" + std::to_string(drone_id) + "/apriltag_ids";

        // Subscribe directly to camera
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&AprilTagDetectorNode::imageCallback, this, std::placeholders::_1));

        // Publish detected IDs
        id_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(output_topic, 10);

        // Initialize AprilTag detector here
        initializeDetector(tag_family);

        RCLCPP_INFO(this->get_logger(), "AprilTag Detector Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing IDs to: %s", output_topic.c_str());
    }

private:
    void initializeDetector(const std::string& family)
    {
        // For now, just store the family name
        // Later you can initialize your chosen AprilTag detection library here
        tag_family_ = family;
        
        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Initialized detector for family: %s", family.c_str());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Convert to grayscale for AprilTag detection
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

            // Detect AprilTags using your chosen library
            std::vector<int> detected_ids = detectAprilTags(gray_image);

            // Publish results
            publishIds(detected_ids);

            if (debug_mode_) {
                RCLCPP_INFO(this->get_logger(), "Processed image, found %zu tags", detected_ids.size());
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }

    std::vector<int> detectAprilTags(const cv::Mat& gray_image)
    {
        std::vector<int> detected_ids;
        
        // Simple test detection based on image characteristics
        // This will help verify the pipeline is working
        
        if (gray_image.empty()) {
            return detected_ids;
        }
        
        // Calculate basic image statistics to simulate detection
        cv::Scalar mean_val = cv::mean(gray_image);
        double mean_brightness = mean_val[0];
        
        // Simple heuristic: if image has good contrast (not too bright/dark), 
        // simulate finding tags based on image content
        if (mean_brightness > 50 && mean_brightness < 200) {
            // Check for dark regions (potential tags)
            cv::Mat binary;
            cv::threshold(gray_image, binary, mean_brightness * 0.7, 255, cv::THRESH_BINARY);
            
            // Count contours as a proxy for potential tags
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // If we find rectangular-ish contours, simulate tag detection
            int tag_candidates = 0;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > 100 && area < 10000) {  // Reasonable size range
                    cv::RotatedRect rect = cv::minAreaRect(contour);
                    float aspect_ratio = rect.size.width / rect.size.height;
                    
                    // If roughly square (like AprilTags)
                    if (aspect_ratio > 0.7 && aspect_ratio < 1.3) {
                        tag_candidates++;
                    }
                }
            }
            
            // Simulate finding specific tag IDs based on number of candidates
            if (tag_candidates > 0) {
                detected_ids.push_back(0);  // Always detect tag 0 if conditions met
            }
            if (tag_candidates > 3) {
                detected_ids.push_back(1);  // Detect tag 1 if more candidates
            }
            if (tag_candidates > 6) {
                detected_ids.push_back(2);  // Detect tag 2 if many candidates
            }
        }
        
        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Processing image %dx%d, brightness: %.1f, found %zu tags", 
                       gray_image.cols, gray_image.rows, mean_brightness, detected_ids.size());
        }
        
        return detected_ids;
    }

    void publishIds(const std::vector<int>& detected_ids)
    {
        std_msgs::msg::Int32MultiArray id_msg;
        id_msg.data = detected_ids;
        
        // Add layout information
        id_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        id_msg.layout.dim[0].label = "detected_ids";
        id_msg.layout.dim[0].size = detected_ids.size();
        id_msg.layout.dim[0].stride = detected_ids.size();
        id_msg.layout.data_offset = 0;

        id_publisher_->publish(id_msg);

        if (debug_mode_ && !detected_ids.empty()) {
            std::string ids_str = "Detected IDs: ";
            for (size_t i = 0; i < detected_ids.size(); ++i) {
                ids_str += std::to_string(detected_ids[i]);
                if (i < detected_ids.size() - 1) ids_str += ", ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ids_str.c_str());
        }
    }

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr id_publisher_;

    // Configuration
    bool debug_mode_;
    double tag_size_;
    std::string tag_family_;

    // AprilTag detector components would go here when implemented
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}