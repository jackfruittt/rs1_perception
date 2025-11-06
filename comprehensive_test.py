#!/usr/bin/env python3
"""
Comprehensive test for event-driven perception system
Ensures all required data is available before testing
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as AprilTagPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time


class ComprehensiveTest(Node):
    def __init__(self):
        super().__init__('comprehensive_test')
        
        # Publishers
        self.tag_pub = self.create_publisher(AprilTagDetectionArray, '/rs1_drone_1/tags', 10)
        
        # Subscribers to verify data availability
        self.odom_sub = self.create_subscription(Odometry, '/rs1_drone_1/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/rs1_drone_1/imu', self.imu_callback, 10)
        self.incident_sub = self.create_subscription(String, '/swarm/incident_claims', self.incident_callback, 10)
        
        # Data availability flags
        self.has_odom = False
        self.has_imu = False
        self.incident_claims = []
        
        self.get_logger().info("Comprehensive test initialized")
    
    def odom_callback(self, msg):
        if not self.has_odom:
            self.has_odom = True
            self.get_logger().info("✅ Odometry data received")
    
    def imu_callback(self, msg):
        if not self.has_imu:
            self.has_imu = True
            self.get_logger().info("✅ IMU data received")
    
    def incident_callback(self, msg):
        self.incident_claims.append(msg.data)
        self.get_logger().info(f"🚨 Incident claim received: {msg.data}")
    
    def create_detection(self, tag_id, x_offset=0.0, y_offset=0.0):
        """Create an AprilTag detection"""
        msg = AprilTagDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        
        detection = AprilTagDetection()
        detection.family = "tf36h11"
        detection.id = tag_id
        detection.hamming = 0
        detection.goodness = 1.0
        detection.decision_margin = 50.0
        
        detection.centre.x = 320.0 + x_offset
        detection.centre.y = 240.0 + y_offset
        
        # Simple corners (square around centre)
        corners = []
        for dx, dy in [(-20, -20), (20, -20), (20, 20), (-20, 20)]:
            corner = AprilTagPoint()
            corner.x = detection.centre.x + dx
            corner.y = detection.centre.y + dy
            corners.append(corner)
        
        detection.corners = corners
        detection.homography = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        msg.detections = [detection]
        return msg
    
    def wait_for_data(self, timeout=10.0):
        """Wait for required data to be available"""
        self.get_logger().info("Waiting for odometry and IMU data...")
        
        start_time = time.time()
        while not (self.has_odom and self.has_imu):
            if time.time() - start_time > timeout:
                self.get_logger().error("❌ Timeout waiting for required data")
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("✅ All required data available")
        return True
    
    def test_event_driven_detection(self):
        """Test event-driven detection with confirmation logic"""
        self.get_logger().info("🎯 Testing event-driven detection...")
        
        # Test WILDFIRE detection (ID=1) - needs 3 confirmations
        self.get_logger().info("Sending WILDFIRE detections for confirmation...")
        
        for i in range(5):  # Send 5 detections (more than required 3)
            detection = self.create_detection(1)  # WILDFIRE
            self.tag_pub.publish(detection)
            self.get_logger().info(f"📡 Sent WILDFIRE detection {i+1}/5")
            time.sleep(0.2)  # 200ms between detections
        
        # Wait for incident claims
        self.get_logger().info("Waiting for incident claims...")
        wait_start = time.time()
        initial_claims = len(self.incident_claims)
        
        while len(self.incident_claims) == initial_claims and time.time() - wait_start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if len(self.incident_claims) > initial_claims:
            self.get_logger().info("✅ Event-driven detection successful!")
            return True
        else:
            self.get_logger().warn("⚠️  No incident claims received")
            return False
    
    def test_spatial_deduplication(self):
        """Test spatial deduplication"""
        self.get_logger().info("🗺️  Testing spatial deduplication...")
        
        # Send detections from same spatial location (should be deduplicated)
        for i in range(3):
            detection = self.create_detection(2, x_offset=1.0, y_offset=1.0)  # RESCUE
            self.tag_pub.publish(detection)
            self.get_logger().info(f"📡 Sent RESCUE detection {i+1}/3 (same location)")
            time.sleep(0.1)
        
        time.sleep(2.0)  # Wait for processing
        
        # Count incident claims for RESCUE
        rescue_claims = [claim for claim in self.incident_claims if "RESCUE" in claim]
        self.get_logger().info(f"RESCUE claims received: {len(rescue_claims)}")
        
        return True  # Just test the mechanism for now


def main():
    rclpy.init()
    
    node = ComprehensiveTest()
    
    try:
        # Wait for required data
        if not node.wait_for_data():
            return
        
        # Additional wait to ensure perception node is ready
        time.sleep(2.0)
        
        # Run tests
        node.test_event_driven_detection()
        time.sleep(3.0)
        
        node.test_spatial_deduplication()
        time.sleep(3.0)
        
        # Summary
        node.get_logger().info(f"🏁 Test complete. Total incident claims: {len(node.incident_claims)}")
        for i, claim in enumerate(node.incident_claims):
            node.get_logger().info(f"  {i+1}: {claim}")
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()