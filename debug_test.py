#!/usr/bin/env python3
"""
Debug test to show detailed system behavior
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as AprilTagPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time


class DebugEventTest(Node):
    def __init__(self):
        super().__init__('debug_event_test')
        
        # Publishers
        self.tag_pub = self.create_publisher(AprilTagDetectionArray, '/rs1_drone_1/tags', 10)
        
        # Subscribers to monitor all relevant topics
        self.scenario_sub = self.create_subscription(String, '/rs1_drone_1/scenario_detection', self.scenario_callback, 10)
        self.claim_sub = self.create_subscription(String, '/swarm/incident_claims', self.claim_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/rs1_drone_1/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/rs1_drone_1/imu', self.imu_callback, 10)
        
        self.odom_received = False
        self.imu_received = False
        self.responses = []
        
        self.get_logger().info("Debug test initialized - monitoring all relevant topics")
    
    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info(f"✅ Odometry received: position [{msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f}]")
            self.odom_received = True
    
    def imu_callback(self, msg):
        if not self.imu_received:
            self.get_logger().info(f"✅ IMU received: orientation [{msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f}]")
            self.imu_received = True
    
    def scenario_callback(self, msg):
        self.get_logger().info(f"🎯 SCENARIO DETECTED: {msg.data}")
        self.responses.append(f"SCENARIO: {msg.data}")
    
    def claim_callback(self, msg):
        self.get_logger().info(f"📢 INCIDENT CLAIM: {msg.data}")  
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
        detection.goodness = 0.9
        detection.decision_margin = 20.0
        
        # Set centre position
        detection.centre.x = 320.0
        detection.centre.y = 240.0
        
        # Create corners (50x50 pixel tag for good depth estimation)
        corners = []
        for x, y in [(295, 215), (345, 215), (345, 265), (295, 265)]:
            corner = AprilTagPoint()
            corner.x = float(x)
            corner.y = float(y)
            corners.append(corner)
        
        detection.corners = corners
        detection.homography = [1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0]
        
        detection_array.detections = [detection]
        return detection_array
    
    def run_debug_test(self):
        """Run comprehensive debug test"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("🔍 COMPREHENSIVE DEBUG TEST")
        self.get_logger().info("="*50)
        
        # Wait for sensor data
        self.get_logger().info("⏳ Waiting for sensor data...")
        start_time = time.time()
        while (not self.odom_received or not self.imu_received) and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.odom_received:
            self.get_logger().error("❌ No odometry data received!")
        if not self.imu_received:
            self.get_logger().error("❌ No IMU data received!")
        
        if not self.odom_received or not self.imu_received:
            self.get_logger().error("❌ Missing sensor data - perception node won't process detections")
            return
        
        self.get_logger().info("✅ All sensor data received, proceeding with detection test")
        
        # Send detection frames to trigger event-driven confirmation
        self.get_logger().info("\n📡 Sending AprilTag detection frames...")
        for i in range(4):  # Send 4 frames to be sure we hit the threshold
            self.get_logger().info(f"   📤 Frame {i+1}/4: Wildfire tag detected")
            detection = self.create_wildfire_detection()
            self.tag_pub.publish(detection)
            time.sleep(0.3)  # 300ms between frames
        
        # Wait for processing
        self.get_logger().info("\n⏳ Waiting for perception node to process detections...")
        wait_start = time.time()
        initial_responses = len(self.responses)
        
        while (time.time() - wait_start) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.responses) > initial_responses:
                self.get_logger().info(f"   📨 New response received! ({len(self.responses)} total)")
        
        # Report results
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("📊 TEST RESULTS")
        self.get_logger().info("="*50)
        
        self.get_logger().info(f"📈 Total responses received: {len(self.responses)}")
        
        if len(self.responses) == 0:
            self.get_logger().error("❌ NO RESPONSES - Check perception node processing")
            self.get_logger().info("💡 Troubleshooting steps:")
            self.get_logger().info("   1. Verify perception node is subscribed to /rs1_drone_1/tags")
            self.get_logger().info("   2. Check perception node logs for errors")
            self.get_logger().info("   3. Ensure new event-driven parameters are loaded")
        else:
            self.get_logger().info("✅ RESPONSES RECEIVED:")
            for i, response in enumerate(self.responses, 1):
                self.get_logger().info(f"   {i}. {response}")
        
        expected_responses = 2  # scenario_detection + incident_claim
        if len(self.responses) >= expected_responses:
            self.get_logger().info("🎉 SUCCESS: Event-driven detection system working!")
        elif len(self.responses) > 0:
            self.get_logger().info("⚠️  PARTIAL SUCCESS: Some responses received")
        else:
            self.get_logger().error("❌ FAILURE: No system responses detected")


def main():
    rclpy.init()
    
    test_node = DebugEventTest()
    
    # Run test after delay
    import threading
    def delayed_test():
        time.sleep(2.0)  # Longer delay for subscriptions to establish
        test_node.run_debug_test()
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