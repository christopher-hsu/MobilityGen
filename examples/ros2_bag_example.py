# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
Example script for working with ROS2 bag files created by MobilityGen.

This script demonstrates how to:
1. Read ROS2 bag files
2. Extract sensor data
3. Analyze the recorded data
4. Convert bag data to other formats

Usage:
    python examples/ros2_bag_example.py --bag-file path/to/recording.db3
"""

import os
import sys
import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    import sqlite3
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"ROS2 not available: {e}")
    ROS2_AVAILABLE = False


class ROS2BagReader:
    """Reader for ROS2 bag files created by MobilityGen."""
    
    def __init__(self, bag_path: str):
        """
        Initialize the bag reader.
        
        Args:
            bag_path: Path to the ROS2 bag file (.db3)
        """
        self.bag_path = bag_path
        self.db_conn = sqlite3.connect(bag_path)
        self.db_cursor = self.db_conn.cursor()
        
        # Load topic information
        self._load_topics()
        
    def _load_topics(self):
        """Load topic information from the bag file."""
        self.db_cursor.execute('SELECT id, name, type FROM topics')
        self.topics = {row[0]: {'name': row[1], 'type': row[2]} for row in self.db_cursor.fetchall()}
        
    def get_topic_names(self):
        """Get list of all topic names in the bag."""
        return [topic['name'] for topic in self.topics.values()]
    
    def get_messages(self, topic_name: str, limit: int = None):
        """
        Get messages from a specific topic.
        
        Args:
            topic_name: Name of the topic
            limit: Maximum number of messages to return (None for all)
            
        Returns:
            List of (timestamp, message) tuples
        """
        # Find topic ID
        topic_id = None
        for tid, topic in self.topics.items():
            if topic['name'] == topic_name:
                topic_id = tid
                break
        
        if topic_id is None:
            raise ValueError(f"Topic '{topic_name}' not found in bag")
        
        # Query messages
        query = '''
            SELECT timestamp, data FROM messages 
            WHERE topic_id = ? 
            ORDER BY timestamp
        '''
        
        if limit:
            query += f' LIMIT {limit}'
        
        self.db_cursor.execute(query, (topic_id,))
        messages = []
        
        for row in self.db_cursor.fetchall():
            timestamp = row[0]
            data = row[1]
            
            # Deserialize message
            try:
                msg_type = self.topics[topic_id]['type']
                message = deserialize_message(data, msg_type)
                messages.append((timestamp, message))
            except Exception as e:
                print(f"Error deserializing message: {e}")
                continue
        
        return messages
    
    def get_common_state_messages(self):
        """Get all common state messages."""
        return self.get_messages('/mobility_gen/common_state')
    
    def get_rgb_messages(self, camera_name: str):
        """Get RGB image messages for a specific camera."""
        return self.get_messages(f'/mobility_gen/rgb/{camera_name}')
    
    def get_depth_messages(self, camera_name: str):
        """Get depth image messages for a specific camera."""
        return self.get_messages(f'/mobility_gen/depth/{camera_name}')
    
    def get_robot_pose_messages(self):
        """Get robot pose messages."""
        return self.get_messages('/mobility_gen/robot_pose')
    
    def get_occupancy_map_messages(self):
        """Get occupancy map messages."""
        return self.get_messages('/mobility_gen/occupancy_map')
    
    def close(self):
        """Close the database connection."""
        self.db_conn.close()


def analyze_bag_file(bag_path: str):
    """Analyze a ROS2 bag file and print statistics."""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, cannot analyze bag file")
        return
    
    reader = ROS2BagReader(bag_path)
    
    print(f"Analyzing bag file: {bag_path}")
    print(f"Available topics: {reader.get_topic_names()}")
    
    # Analyze common state messages
    common_state_msgs = reader.get_common_state_messages()
    print(f"Number of common state messages: {len(common_state_msgs)}")
    
    if common_state_msgs:
        # Parse first message to see structure
        timestamp, msg = common_state_msgs[0]
        try:
            state_data = json.loads(msg.data)
            print(f"State data keys: {list(state_data['data'].keys())}")
        except Exception as e:
            print(f"Error parsing state data: {e}")
    
    # Analyze robot pose messages
    pose_msgs = reader.get_robot_pose_messages()
    print(f"Number of robot pose messages: {len(pose_msgs)}")
    
    if pose_msgs:
        timestamp, msg = pose_msgs[0]
        print(f"First robot pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
    
    # Analyze RGB messages
    rgb_topics = [topic for topic in reader.get_topic_names() if '/rgb/' in topic and not '/compressed' in topic]
    for topic in rgb_topics:
        msgs = reader.get_messages(topic)
        print(f"Number of {topic} messages: {len(msgs)}")
        if msgs:
            timestamp, msg = msgs[0]
            print(f"  Image size: {msg.width}x{msg.height}")
    
    reader.close()


def extract_images_from_bag(bag_path: str, output_dir: str, camera_name: str = "camera_left"):
    """Extract RGB images from a bag file."""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, cannot extract images")
        return
    
    try:
        from cv_bridge import CvBridge
        cv_bridge = CvBridge()
    except ImportError:
        print("cv_bridge not available, cannot extract images")
        return
    
    reader = ROS2BagReader(bag_path)
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Get RGB messages
    rgb_msgs = reader.get_rgb_messages(camera_name)
    print(f"Extracting {len(rgb_msgs)} images from {camera_name}")
    
    for i, (timestamp, msg) in enumerate(rgb_msgs):
        try:
            # Convert ROS message to OpenCV image
            cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Save image
            output_path = os.path.join(output_dir, f"{i:06d}.jpg")
            import cv2
            cv2.imwrite(output_path, cv_image)
            
            if i % 100 == 0:
                print(f"Extracted {i} images...")
                
        except Exception as e:
            print(f"Error extracting image {i}: {e}")
    
    print(f"Extracted {len(rgb_msgs)} images to {output_dir}")
    reader.close()


def plot_robot_trajectory(bag_path: str):
    """Plot the robot trajectory from pose messages."""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, cannot plot trajectory")
        return
    
    reader = ROS2BagReader(bag_path)
    
    # Get robot pose messages
    pose_msgs = reader.get_robot_pose_messages()
    
    if not pose_msgs:
        print("No robot pose messages found")
        reader.close()
        return
    
    # Extract positions
    x_positions = []
    y_positions = []
    timestamps = []
    
    for timestamp, msg in pose_msgs:
        x_positions.append(msg.pose.position.x)
        y_positions.append(msg.pose.position.y)
        timestamps.append(timestamp)
    
    # Plot trajectory
    plt.figure(figsize=(10, 8))
    plt.plot(x_positions, y_positions, 'b-', linewidth=2, label='Robot Trajectory')
    plt.plot(x_positions[0], y_positions[0], 'go', markersize=10, label='Start')
    plt.plot(x_positions[-1], y_positions[-1], 'ro', markersize=10, label='End')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Robot Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Save plot
    output_path = os.path.join(os.path.dirname(bag_path), 'trajectory_plot.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Trajectory plot saved to: {output_path}")
    
    plt.show()
    reader.close()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='ROS2 Bag File Analysis')
    parser.add_argument('--bag-file', type=str, required=True,
                       help='Path to the ROS2 bag file (.db3)')
    parser.add_argument('--analyze', action='store_true',
                       help='Analyze the bag file and print statistics')
    parser.add_argument('--extract-images', action='store_true',
                       help='Extract RGB images from the bag file')
    parser.add_argument('--output-dir', type=str, default='./extracted_images',
                       help='Output directory for extracted images')
    parser.add_argument('--camera-name', type=str, default='camera_left',
                       help='Camera name for image extraction')
    parser.add_argument('--plot-trajectory', action='store_true',
                       help='Plot robot trajectory')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_file):
        print(f"Bag file not found: {args.bag_file}")
        return
    
    if args.analyze:
        analyze_bag_file(args.bag_file)
    
    if args.extract_images:
        extract_images_from_bag(args.bag_file, args.output_dir, args.camera_name)
    
    if args.plot_trajectory:
        plot_robot_trajectory(args.bag_file)
    
    # If no specific action is requested, just analyze
    if not any([args.analyze, args.extract_images, args.plot_trajectory]):
        analyze_bag_file(args.bag_file)


if __name__ == "__main__":
    main() 