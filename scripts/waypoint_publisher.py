#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
import json
import time
import argparse
import math

"""
Waypoint Publisher for MobilityGen

This script publishes waypoints to ROS2 topics that MobilityGen can subscribe to.
It's useful for testing robot navigation without using the LLM navigator.

Usage:
python3 waypoint_publisher.py --waypoints "[[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]]"
python3 waypoint_publisher.py --file waypoints.json
python3 waypoint_publisher.py --demo
"""

class WaypointPublisher(Node):
    def __init__(self, namespace: str = "/mobility_gen"):
        super().__init__("waypoint_publisher")
        
        self.namespace = namespace
        
        # Create publishers
        self.waypoints_pub = self.create_publisher(
            String, f"{self.namespace}/waypoints", 10
        )
        
        self.single_waypoint_pub = self.create_publisher(
            PoseStamped, f"{self.namespace}/waypoint", 10
        )
        
        # Create marker publisher for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Marker ID counter
        self.marker_id = 0
        
        print(f"✓ Waypoint publisher initialized on namespace: {self.namespace}")
        print(f"  Topics:")
        print(f"    - {self.namespace}/waypoints (String - JSON array of waypoints)")
        print(f"    - {self.namespace}/waypoint (PoseStamped - single waypoint)")
        print(f"    - /waypoint_markers (MarkerArray - visualization markers)")

    def publish_waypoints(self, waypoints):
        """Publish a list of waypoints as JSON string."""
        try:
            # Convert waypoints to the format expected by MobilityGen
            waypoint_list = []
            for wp in waypoints:
                if isinstance(wp, (list, tuple)):
                    # Convert [x, y, z] to dict format
                    waypoint_dict = {
                        "x": float(wp[0]),
                        "y": float(wp[1]),
                        "z": float(wp[2]) if len(wp) > 2 else 0.0,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": 0.0,
                        "qw": 1.0
                    }
                else:
                    # Already in dict format
                    waypoint_dict = wp
                
                waypoint_list.append(waypoint_dict)
            
            msg = String()
            msg.data = json.dumps({"waypoints": waypoint_list})
            self.waypoints_pub.publish(msg)
            self.get_logger().info(f"✓ Published {len(waypoints)} waypoints to {self.namespace}/waypoints")
            return True
        except Exception as e:
            self.get_logger().error(f"Error publishing waypoints: {e}")
            return False
    
    def publish_single_waypoint(self, waypoint):
        """Publish a single waypoint as PoseStamped."""
        try:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            if isinstance(waypoint, (list, tuple)):
                msg.pose.position.x = float(waypoint[0])
                msg.pose.position.y = float(waypoint[1])
                msg.pose.position.z = float(waypoint[2]) if len(waypoint) > 2 else 0.0
            else:
                msg.pose.position.x = float(waypoint.get('x', 0.0))
                msg.pose.position.y = float(waypoint.get('y', 0.0))
                msg.pose.position.z = float(waypoint.get('z', 0.0))
            
            msg.pose.orientation.x = float(waypoint.get('qx', 0.0)) if isinstance(waypoint, dict) else 0.0
            msg.pose.orientation.y = float(waypoint.get('qy', 0.0)) if isinstance(waypoint, dict) else 0.0
            msg.pose.orientation.z = float(waypoint.get('qz', 0.0)) if isinstance(waypoint, dict) else 0.0
            msg.pose.orientation.w = float(waypoint.get('qw', 1.0)) if isinstance(waypoint, dict) else 1.0
            
            self.single_waypoint_pub.publish(msg)
            self.get_logger().info(f"✓ Published single waypoint: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Error publishing single waypoint: {e}")
            return False

    def publish_waypoint_markers(self, waypoints):
        """Publish markers for all waypoints in the path."""
        marker_array = MarkerArray()
        
        # Create markers for each waypoint
        for i, waypoint in enumerate(waypoints):
            # Get coordinates
            if isinstance(waypoint, (list, tuple)):
                x, y, z = waypoint[0], waypoint[1], waypoint[2] if len(waypoint) > 2 else 0.0
            else:
                x = waypoint.get('x', 0.0)
                y = waypoint.get('y', 0.0)
                z = waypoint.get('z', 0.0)
            
            # Create sphere marker for waypoint position
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "waypoints"
            sphere_marker.id = i * 3
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = Point(x=float(x), y=float(y), z=float(z))
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.2  # 20cm diameter
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            sphere_marker.color.r = 0.0
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            marker_array.markers.append(sphere_marker)
            
            # Create text marker for waypoint label
            text_marker = Marker()
            text_marker.header = sphere_marker.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i * 3 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = Point(x=float(x), y=float(y), z=float(z) + 0.3)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{i+1}"
            marker_array.markers.append(text_marker)
            
            # Create arrow marker for orientation
            arrow_marker = Marker()
            arrow_marker.header = sphere_marker.header
            arrow_marker.ns = "waypoint_orientations"
            arrow_marker.id = i * 3 + 2
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position = Point(x=float(x), y=float(y), z=float(z))
            
            if isinstance(waypoint, dict):
                arrow_marker.pose.orientation.x = float(waypoint.get('qx', 0.0))
                arrow_marker.pose.orientation.y = float(waypoint.get('qy', 0.0))
                arrow_marker.pose.orientation.z = float(waypoint.get('qz', 0.0))
                arrow_marker.pose.orientation.w = float(waypoint.get('qw', 1.0))
            else:
                arrow_marker.pose.orientation.w = 1.0
            
            arrow_marker.scale.x = 0.3  # Shaft length
            arrow_marker.scale.y = 0.05  # Shaft diameter
            arrow_marker.scale.z = 0.05  # Head diameter
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 0.8
            marker_array.markers.append(arrow_marker)
        
        # Add a clear marker at the start
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, clear_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(waypoints)} waypoint markers")

    def load_waypoints_from_file(self, filename):
        """Load waypoints from a JSON file."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            # Handle different JSON formats
            if isinstance(data, dict):
                if 'waypoints' in data:
                    return data['waypoints']
                elif 'object_waypoints' in data:
                    return [wp['waypoint'] for wp in data['object_waypoints']]
                else:
                    return [data]  # Single waypoint
            elif isinstance(data, list):
                return data
            else:
                print(f"Warning: Unknown waypoint file format in {filename}")
                return []
                
        except Exception as e:
            print(f"Error loading waypoints from {filename}: {e}")
            return []

    def get_demo_waypoints(self):
        """Get demo waypoints for testing."""
        return [
            [5.0, 3.0, 0.0],
            [8.0, 6.0, 0.0],
            [12.0, 4.0, 0.0],
            [15.0, 8.0, 0.0],
            [10.0, 10.0, 0.0],
            [6.0, 12.0, 0.0],
            [2.0, 8.0, 0.0]
        ]


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Waypoint Publisher for MobilityGen')
    parser.add_argument('--namespace', default='/mobility_gen', help='ROS2 namespace')
    parser.add_argument('--waypoints', type=str, help='JSON string of waypoints')
    parser.add_argument('--file', type=str, help='JSON file containing waypoints')
    parser.add_argument('--demo', action='store_true', help='Use demo waypoints')
    parser.add_argument('--repeat', action='store_true', help='Repeat waypoints continuously')
    parser.add_argument('--interval', type=float, default=5.0, help='Interval between repeats (seconds)')
    parser.add_argument('--wait', type=float, default=1.0, help='Wait time after publishing (seconds)')
    
    args = parser.parse_args()
    
    publisher = WaypointPublisher(namespace=args.namespace)
    
    try:
        waypoints = []
        
        # Load waypoints based on arguments
        if args.demo:
            waypoints = publisher.get_demo_waypoints()
            print("Using demo waypoints")
        elif args.waypoints:
            try:
                waypoints = json.loads(args.waypoints)
                print(f"Using waypoints from command line: {len(waypoints)} waypoints")
            except json.JSONDecodeError as e:
                print(f"Error parsing waypoints JSON: {e}")
                return
        elif args.file:
            waypoints = publisher.load_waypoints_from_file(args.file)
            print(f"Loaded {len(waypoints)} waypoints from {args.file}")
        else:
            print("No waypoints specified. Use --demo, --waypoints, or --file")
            print("Example: python3 waypoint_publisher.py --demo")
            return
        
        if not waypoints:
            print("No waypoints to publish")
            return
        
        # Publish waypoints
        success = publisher.publish_waypoints(waypoints)
        if success:
            publisher.publish_waypoint_markers(waypoints)
            print(f"✓ Published {len(waypoints)} waypoints")
            print(f"✓ Published visualization markers")
        
        # Wait a bit to ensure messages are published
        time.sleep(args.wait)
        
        print(f"✓ Waypoint publisher is running. Press Ctrl+C to stop.")
        print(f"  Topics:")
        print(f"    - {args.namespace}/waypoints (String - JSON array of waypoints)")
        print(f"    - {args.namespace}/waypoint (PoseStamped - single waypoint)")
        print(f"    - /waypoint_markers (MarkerArray - visualization markers)")
        
        if args.repeat:
            print(f"✓ Repeating waypoints every {args.interval} seconds...")
            while rclpy.ok():
                time.sleep(args.interval)
                publisher.publish_waypoints(waypoints)
                publisher.publish_waypoint_markers(waypoints)
        else:
            # Keep the node running for a bit to ensure messages are published
            time.sleep(2.0)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 