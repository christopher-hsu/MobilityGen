#!/usr/bin/env python3

"""
Test ROS2 availability in Isaac Sim's environment.
"""

import sys
import traceback

print("=== Testing ROS2 in Isaac Sim Environment ===")
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version}")

# Test each import individually
imports_to_test = [
    ("rclpy", "import rclpy"),
    ("rclpy.node", "from rclpy.node import Node"),
    ("rclpy.serialization", "from rclpy.serialization import serialize_message"),
    ("std_msgs.msg", "from std_msgs.msg import String"),
    ("sensor_msgs.msg", "from sensor_msgs.msg import Image"),
    ("geometry_msgs.msg", "from geometry_msgs.msg import PoseStamped"),
    ("cv_bridge", "from cv_bridge import CvBridge"),
]

for module_name, import_statement in imports_to_test:
    try:
        exec(import_statement)
        print(f"✓ {module_name} imported successfully")
    except ImportError as e:
        print(f"✗ {module_name} import failed: {e}")
    except Exception as e:
        print(f"✗ {module_name} unexpected error: {e}")
        traceback.print_exc()

# Test basic ROS2 functionality
try:
    import rclpy
    print("\n=== Testing ROS2 Initialization ===")
    
    if not rclpy.ok():
        rclpy.init()
        print("✓ ROS2 initialized")
    
    from rclpy.node import Node
    node = Node("test_node")
    print("✓ ROS2 node created")
    
    node.destroy_node()
    rclpy.shutdown()
    print("✓ ROS2 shutdown successfully")
    
except Exception as e:
    print(f"✗ ROS2 functionality test failed: {e}")
    traceback.print_exc()

print("\n=== Test Complete ===") 