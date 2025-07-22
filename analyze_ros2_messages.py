#!/usr/bin/env python3

"""
Analyze ROS2 messages to see what data is being published.
"""

import sys
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class MessageAnalyzer(Node):
    """Node to analyze ROS2 messages."""
    
    def __init__(self):
        super().__init__("message_analyzer")
        
        # Subscribe to mobility_gen topics
        self.common_state_sub = self.create_subscription(
            String, "/mobility_gen/common_state", self.common_state_callback, 10
        )
        
        self.rgb_sub = self.create_subscription(
            Image, "/mobility_gen/rgb/camera_left", self.rgb_callback, 10
        )
        
        print("✓ Message analyzer created")
        print("Listening for topics:")
        print("  /mobility_gen/common_state")
        print("  /mobility_gen/rgb/camera_left")
    
    def common_state_callback(self, msg):
        """Callback for common state messages."""
        try:
            data = json.loads(msg.data)
            print(f"\n=== Common State Message ===")
            print(f"Step: {data.get('step', 'N/A')}")
            print(f"Timestamp: {data.get('timestamp', 'N/A')}")
            
            state_data = data.get('data', {})
            print(f"State data keys: {list(state_data.keys())}")
            
            # Show some sample data
            for key, value in state_data.items():
                if isinstance(value, dict):
                    print(f"  {key}: {list(value.keys())}")
                elif isinstance(value, list):
                    print(f"  {key}: list with {len(value)} items")
                else:
                    print(f"  {key}: {type(value).__name__}")
                    
        except Exception as e:
            print(f"Error parsing common state message: {e}")
    
    def rgb_callback(self, msg):
        """Callback for RGB image messages."""
        print(f"\n=== RGB Image Message ===")
        print(f"Image size: {msg.width}x{msg.height}")
        print(f"Encoding: {msg.encoding}")
        print(f"Frame ID: {msg.header.frame_id}")

def main():
    """Main function."""
    print("=== ROS2 Message Analyzer ===")
    
    try:
        rclpy.init()
        
        analyzer = MessageAnalyzer()
        
        print("Analyzing messages... (Press Ctrl+C to stop)")
        
        # Spin for a few seconds
        start_time = time.time()
        while time.time() - start_time < 30:  # Listen for 30 seconds
            rclpy.spin_once(analyzer, timeout_sec=0.1)
            time.sleep(0.1)
        
        print("Analysis complete")
        analyzer.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 