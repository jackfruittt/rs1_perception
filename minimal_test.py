#!/usr/bin/env python3
"""
Minimal test to check if AprilTag messages are being received by perception node
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as AprilTagPoint
import time


class MinimalTest(Node):
    def __init__(self):
        super().__init__('minimal_test')
        
        # Publisher
        self.tag_pub = self.create_publisher(AprilTagDetectionArray, '/rs1_drone_1/tags', 10)
        self.get_logger().info("Minimal test - publishing to /rs1_drone_1/tags")
    
    def create_simple_detection(self):
        """Create a simple detection"""
        msg = AprilTagDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        
        detection = AprilTagDetection()
        detection.family = "tf36h11"
        detection.id = 1  # WILDFIRE
        detection.hamming = 0
        detection.goodness = 1.0
        detection.decision_margin = 50.0
        
        detection.centre.x = 320.0
        detection.centre.y = 240.0
        
        # Simple corners
        corners = []
        for x, y in [(300, 220), (340, 220), (340, 260), (300, 260)]:
            corner = AprilTagPoint()
            corner.x = float(x)
            corner.y = float(y)
            corners.append(corner)
        
        detection.corners = corners
        detection.homography = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        msg.detections = [detection]
        return msg
    
    def run_test(self):
        """Send a bunch of detections"""
        self.get_logger().info("Sending 10 detection messages...")
        
        for i in range(10):
            detection = self.create_simple_detection()
            self.tag_pub.publish(detection)
            self.get_logger().info(f"Published detection {i+1}/10")
            time.sleep(0.1)
        
        self.get_logger().info("All detections sent!")


def main():
    rclpy.init()
    
    node = MinimalTest()
    
    # Run test
    time.sleep(1.0)  # Let subscriptions establish
    node.run_test()
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()