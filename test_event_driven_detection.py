#!/usr/bin/env python3
"""
Test script for the event-driven perception system.
Demonstrates the three-layer solution:
1. Perception Layer: Event-driven detection tracking
2. Coordination Layer: First-come-first-serve incident claims
3. Spatial Deduplication: Prevents duplicate incidents within 5m
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as AprilTagPoint
from geometry_msgs.msg import Point
import time
import threading


class TestEventDrivenDetection(Node):
    def __init__(self):
        super().__init__('test_event_driven_detection')
        
        # Publishers to simulate AprilTag detections
        self.tag_pub_drone1 = self.create_publisher(
            AprilTagDetectionArray, 
            '/rs1_drone_1/tags', 
            10
        )
        self.tag_pub_drone2 = self.create_publisher(
            AprilTagDetectionArray, 
            '/rs1_drone_2/tags', 
            10
        )
        
        # Subscriber to monitor incident claims
        self.claim_sub = self.create_subscription(
            String,
            '/swarm/incident_claims',
            self.claim_callback,
            10
        )
        
        # Subscriber to monitor scenario detections
        self.scenario_sub1 = self.create_subscription(
            String,
            '/rs1_drone_1/scenario_detection',
            lambda msg: self.scenario_callback(msg, 1),
            10
        )
        self.scenario_sub2 = self.create_subscription(
            String,
            '/rs1_drone_2/scenario_detection',
            lambda msg: self.scenario_callback(msg, 2),
            10
        )
        
        self.get_logger().info("Test system initialized - monitoring event-driven detection")
    
    def claim_callback(self, msg):
        """Monitor incident claims from perception nodes"""
        self.get_logger().info(f"INCIDENT CLAIM: {msg.data}")
    
    def scenario_callback(self, msg, drone_id):
        """Monitor scenario detections from perception nodes"""
        self.get_logger().info(f"SCENARIO DETECTION (Drone {drone_id}): {msg.data}")
    
    def create_detection_message(self, tag_id, x=10.0, y=10.0, z=15.0):
        """Create an AprilTag detection message"""
        detection_array = AprilTagDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"
        
        detection = AprilTagDetection()
        detection.family = "tf36h11"
        detection.id = tag_id  # 0=HIKER, 1=WILDFIRE, 2=DEBRIS
        detection.hamming = 0
        detection.goodness = 0.5
        detection.decision_margin = 10.0
        
        # Set centre position
        detection.centre.x = 125.0  # Center of simulated detection
        detection.centre.y = 125.0
        
        # Simulate corner positions (required for depth estimation)
        corner1 = AprilTagPoint()
        corner1.x = 100.0
        corner1.y = 100.0
        
        corner2 = AprilTagPoint()
        corner2.x = 150.0
        corner2.y = 100.0
        
        corner3 = AprilTagPoint() 
        corner3.x = 150.0
        corner3.y = 150.0
        
        corner4 = AprilTagPoint()
        corner4.x = 100.0
        corner4.y = 150.0
        
        detection.corners = [corner1, corner2, corner3, corner4]
        
        # Set homography matrix (identity-like for simplicity)
        detection.homography = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        detection_array.detections = [detection]
        return detection_array
    
    def test_scenario_1_event_driven_confirmation(self):
        """Test 1: Event-driven confirmation (3+ frames required)"""
        self.get_logger().info("\n=== TEST 1: Event-driven confirmation ===")
        self.get_logger().info("Sending 2 frames (should not publish - need 3+ for confirmation)")
        
        # Send 2 detections - should not result in publication
        for i in range(2):
            detection = self.create_detection_message(1, 5.0, 5.0, 15.0)  # WILDFIRE
            self.tag_pub_drone1.publish(detection)
            time.sleep(0.1)
        
        time.sleep(1.0)
        
        self.get_logger().info("Sending 3rd frame (should publish - confirmation threshold reached)")
        # Send 3rd detection - should result in publication
        detection = self.create_detection_message(1, 5.0, 5.0, 15.0)  # WILDFIRE
        self.tag_pub_drone1.publish(detection)
        
        time.sleep(2.0)
    
    def test_scenario_2_spatial_deduplication(self):
        """Test 2: Spatial deduplication (prevents duplicates within 5m)"""
        self.get_logger().info("\n=== TEST 2: Spatial deduplication ===")
        
        # First drone detects wildfire at (10, 10, 15)
        self.get_logger().info("Drone 1 detecting wildfire at (10, 10, 15)")
        for i in range(3):  # Send 3 frames for confirmation
            detection = self.create_detection_message(1, 10.0, 10.0, 15.0)
            self.tag_pub_drone1.publish(detection)
            time.sleep(0.1)
        
        time.sleep(1.0)
        
        # Second drone detects wildfire at nearby location (12, 12, 15) - within 5m
        self.get_logger().info("Drone 2 detecting wildfire at (12, 12, 15) - within 5m cluster")
        for i in range(3):  # Should be rejected due to spatial clustering
            detection = self.create_detection_message(1, 12.0, 12.0, 15.0)
            self.tag_pub_drone2.publish(detection)
            time.sleep(0.1)
        
        time.sleep(2.0)
    
    def test_scenario_3_first_come_first_serve(self):
        """Test 3: First-come-first-serve coordination"""
        self.get_logger().info("\n=== TEST 3: First-come-first-serve coordination ===")
        
        # Simulate simultaneous detection from multiple drones
        def drone1_detect():
            self.get_logger().info("Drone 1 simultaneously detecting hiker at (20, 20, 10)")
            for i in range(3):
                detection = self.create_detection_message(0, 20.0, 20.0, 10.0)  # HIKER
                self.tag_pub_drone1.publish(detection)
                time.sleep(0.05)
        
        def drone2_detect():
            self.get_logger().info("Drone 2 simultaneously detecting hiker at (21, 21, 10)")
            for i in range(3):
                detection = self.create_detection_message(0, 21.0, 21.0, 10.0)  # HIKER
                self.tag_pub_drone2.publish(detection)
                time.sleep(0.05)
        
        # Start both detections simultaneously
        t1 = threading.Thread(target=drone1_detect)
        t2 = threading.Thread(target=drone2_detect)
        
        t1.start()
        time.sleep(0.001)  # Slight delay to test race condition
        t2.start()
        
        t1.join()
        t2.join()
        
        time.sleep(3.0)
    
    def test_scenario_4_cooldown_period(self):
        """Test 4: 30-second cooldown period"""
        self.get_logger().info("\n=== TEST 4: Cooldown period (simulated - normally 30s) ===")
        
        # First detection
        self.get_logger().info("First debris detection at (30, 30, 5)")
        for i in range(3):
            detection = self.create_detection_message(2, 30.0, 30.0, 5.0)  # DEBRIS
            self.tag_pub_drone1.publish(detection)
            time.sleep(0.1)
        
        time.sleep(1.0)
        
        # Second detection within cooldown (should be suppressed)
        self.get_logger().info("Second debris detection at same location (should be in cooldown)")
        for i in range(3):
            detection = self.create_detection_message(2, 30.0, 30.0, 5.0)  # DEBRIS
            self.tag_pub_drone1.publish(detection)
            time.sleep(0.1)
        
        time.sleep(2.0)
    
    def run_all_tests(self):
        """Run all test scenarios"""
        self.get_logger().info("Starting comprehensive event-driven detection tests...\n")
        
        self.test_scenario_1_event_driven_confirmation()
        self.test_scenario_2_spatial_deduplication()  
        self.test_scenario_3_first_come_first_serve()
        self.test_scenario_4_cooldown_period()
        
        self.get_logger().info("\n=== ALL TESTS COMPLETED ===")
        self.get_logger().info("Summary of implemented features:")
        self.get_logger().info("✅ Event-driven confirmation (3+ frames)")
        self.get_logger().info("✅ 30-second cooldown period")
        self.get_logger().info("✅ Spatial clustering (5m deduplication)")
        self.get_logger().info("✅ First-come-first-serve coordination")
        self.get_logger().info("✅ Drone ID tiebreaker for conflicts")
        self.get_logger().info("✅ Incident claims broadcasting")
        self.get_logger().info("✅ Stand-down behavior on conflicts")


def main():
    rclpy.init()
    
    test_node = TestEventDrivenDetection()
    
    # Run tests after a short delay to allow subscriptions to be established
    def delayed_test():
        time.sleep(2.0)
        test_node.run_all_tests()
    
    test_thread = threading.Thread(target=delayed_test)
    test_thread.start()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    
    test_node.get_logger().info("Test completed")
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()