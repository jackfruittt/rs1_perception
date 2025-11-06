#!/usr/bin/env python3
"""
Simple test for the event-driven perception system.
Tests the basic functionality without requiring full drone system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimpleEventTest(Node):
    def __init__(self):
        super().__init__('simple_event_test')
        
        # Publisher to simulate scenario detection
        self.scenario_pub = self.create_publisher(
            String, 
            '/rs1_drone_1/scenario_detection', 
            10
        )
        
        # Subscriber to monitor incident claims
        self.claim_sub = self.create_subscription(
            String,
            '/swarm/incident_claims',
            self.claim_callback,
            10
        )
        
        self.claims_received = []
        self.get_logger().info("Simple event test initialized")
    
    def claim_callback(self, msg):
        """Monitor incident claims"""
        self.get_logger().info(f"RECEIVED CLAIM: {msg.data}")
        self.claims_received.append(msg.data)
    
    def test_basic_functionality(self):
        """Test basic scenario detection and claim system"""
        self.get_logger().info("\n=== Testing Basic Event-Driven System ===")
        
        # Test 1: Simple scenario detection
        self.get_logger().info("Publishing scenario detection...")
        scenario_msg = String()
        scenario_msg.data = "WILDFIRE,1.0,10.0,10.0,15.0,0.0,respond:1"
        self.scenario_pub.publish(scenario_msg)
        
        time.sleep(1.0)
        
        # Test 2: Another scenario
        self.get_logger().info("Publishing hiker scenario...")
        scenario_msg.data = "STRANDED_HIKER,0.8,20.0,20.0,10.0,1.57,respond:1"
        self.scenario_pub.publish(scenario_msg)
        
        time.sleep(2.0)
        
        self.get_logger().info(f"Test completed. Claims received: {len(self.claims_received)}")
        for claim in self.claims_received:
            self.get_logger().info(f"  - {claim}")


def main():
    rclpy.init()
    
    test_node = SimpleEventTest()
    
    # Run test after delay
    import threading
    def delayed_test():
        time.sleep(1.0)
        test_node.test_basic_functionality()
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