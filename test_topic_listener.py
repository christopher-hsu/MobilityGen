#!/usr/bin/env python3

"""
Simple ROS2 topic listener to check if topics are being published.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class TopicListener(Node):
    """Simple node to listen for topics."""
    
    def __init__(self):
        super().__init__("topic_listener")
        
        # Subscribe to mobility_gen topics
        self.common_state_sub = self.create_subscription(
            String, "/mobility_gen/common_state", self.common_state_callback, 10
        )
        
        self.rgb_sub = self.create_subscription(
            Image, "/mobility_gen/rgb/camera_left", self.rgb_callback, 10
        )
        
        print("✓ Topic listener created")
        print("Listening for topics:")
        print("  /mobility_gen/common_state")
        print("  /mobility_gen/rgb/camera_left")
    
    def common_state_callback(self, msg):
        """Callback for common state messages."""
        print(f"✓ Received common state message: {len(msg.data)} characters")
    
    def rgb_callback(self, msg):
        """Callback for RGB image messages."""
        print(f"✓ Received RGB image: {msg.width}x{msg.height}")

def main():
    """Main function."""
    print("=== ROS2 Topic Listener ===")
    
    try:
        rclpy.init()
        
        listener = TopicListener()
        
        print("Listening for messages... (Press Ctrl+C to stop)")
        
        # Spin for a few seconds
        start_time = time.time()
        while time.time() - start_time < 10:  # Listen for 10 seconds
            rclpy.spin_once(listener, timeout_sec=0.1)
            time.sleep(0.1)
        
        print("Listening complete")
        listener.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 