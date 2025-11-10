#!/usr/bin/env python3
"""
Single detection test to verify system response
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as AprilTagPoint
import time


class SingleDetectionTest(Node):
    def __init__(self):
        super().__init__('single_detection_test')
        
        # Publisher to simulate AprilTag detection
        self.tag_pub = self.create_publisher(
            AprilTagDetectionArray, 
            '/rs1_drone_1/tags', 
            10
        )
        
        # Subscribers to monitor responses
        self.scenario_sub = self.create_subscription(
            String,
            '/rs1_drone_1/scenario_detection',
            self.scenario_callback,
            10
        )
        
        self.claim_sub = self.create_subscription(
            String,
            '/swarm/incident_claims',
            self.claim_callback,
            10
        )
        
        self.responses = []
        self.get_logger().info("Single detection test ready")
    
    def scenario_callback(self, msg):
        self.get_logger().info(f"SCENARIO DETECTED: {msg.data}")
        self.responses.append(f"SCENARIO: {msg.data}")
    
    def claim_callback(self, msg):
        self.get_logger().info(f"INCIDENT CLAIM: {msg.data}")
        self.responses.append(f"CLAIM: {msg.data}")
    
    def create_wildfire_detection(self):
        """Create a wildfire detection message"""
        detection_array = AprilTagDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"
        
        detection = AprilTagDetection()
        detection.family = "tf36h11"
        detection.id = 1  # WILDFIRE
        detection.hamming = 0
        detection.goodness = 0.8
        detection.decision_margin = 15.0
        
        # Set centre position
        detection.centre.x = 320.0
        detection.centre.y = 240.0
        
        # Create corners for depth estimation
        corners = []
        for i, (x, y) in enumerate([(300, 220), (340, 220), (340, 260), (300, 260)]):
            corner = AprilTagPoint()
            corner.x = float(x)
            corner.y = float(y)  
            corners.append(corner)
        
        detection.corners = corners
        detection.homography = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        detection_array.detections = [detection]
        return detection_array
    
    def run_test(self):
        """Run single detection test"""
        self.get_logger().info("\n=== Single Wildfire Detection Test ===")
        
        # Send 3 consecutive detections to trigger confirmation
        for i in range(3):
            self.get_logger().info(f"Sending detection frame {i+1}/3...")
            detection = self.create_wildfire_detection()
            self.tag_pub.publish(detection)
            time.sleep(0.2)  # Small delay between frames
        
        # Wait for system to process
        self.get_logger().info("Waiting for system response...")
        time.sleep(3.0)
        
        # Report results
        self.get_logger().info(f"\nTest completed. Responses received: {len(self.responses)}")
        for response in self.responses:
            self.get_logger().info(f"  - {response}")
        
        if len(self.responses) >= 2:
            self.get_logger().info("✅ SUCCESS: Both scenario detection and incident claim received!")
        elif len(self.responses) == 1:
            self.get_logger().info("⚠️  PARTIAL: Only one response received")
        else:
            self.get_logger().info("❌ FAILED: No responses received")


def main():
    rclpy.init()
    
    test_node = SingleDetectionTest()
    
    # Run test after delay
    import threading
    def delayed_test():
        time.sleep(1.0)
        test_node.run_test()
        time.sleep(2.0)
        rclpy.shutdown()
    
    test_thread = threading.Thread(target=delayed_test)
    test_thread.start()
    
    try:
        rclpy.spin(test_node)
    except:
        pass
    
    test_node.destroy_node()


if __name__ == '__main__':
    main()