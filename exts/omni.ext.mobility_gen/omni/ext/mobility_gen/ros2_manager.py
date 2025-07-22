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
ROS2 Manager for MobilityGen Extension

This module provides ROS2 functionality for the MobilityGen extension,
including streaming and bag file recording capabilities.
"""

import os
import sys
import json
import time
import sqlite3
import numpy as np
from typing import Optional, Dict, Any
import traceback

# Performance profiling
import time as time_module


# ROS2 availability flag
ROS2_AVAILABLE = False
ROS2Writer = None


def _setup_ros2_paths():
    """Add ROS2 Python paths to sys.path."""
    ros2_paths = [
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.8/site-packages",
        "/opt/ros/humble/lib/python3.9/site-packages",
    ]
    
    for path in ros2_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"Added ROS2 path: {path}")


def _convert_numpy_to_json(obj):
    """Convert numpy arrays to JSON-serializable format."""
    if hasattr(obj, 'tolist'):  # NumPy array
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: _convert_numpy_to_json(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_convert_numpy_to_json(item) for item in obj]
    else:
        return obj


try:
    # Setup ROS2 paths
    _setup_ros2_paths()
    
    # Import ROS2 modules
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge
    from std_msgs.msg import Header
    
    print("✓ All ROS2 imports successful")
    
    class SimpleROS2Writer(Node):
        """Simple ROS2 writer for streaming and bag recording."""
        
        # Pre-import ROS2 modules for performance
        _ros2_modules_imported = False
        _cv_bridge = None
        _geometry_msgs = None
        _sensor_msgs = None
        _std_msgs = None
        _traceback = None
        
        @classmethod
        def _import_ros2_modules(cls):
            """Import ROS2 modules once for performance."""
            if not cls._ros2_modules_imported:
                try:
                    from cv_bridge import CvBridge
                    import geometry_msgs.msg
                    import sensor_msgs.msg
                    import std_msgs.msg
                    import traceback
                    
                    cls._cv_bridge = CvBridge
                    cls._geometry_msgs = geometry_msgs.msg
                    cls._sensor_msgs = sensor_msgs.msg
                    cls._std_msgs = std_msgs.msg
                    cls._traceback = traceback
                    cls._ros2_modules_imported = True
                    print("✓ ROS2 modules pre-imported for performance")
                except Exception as e:
                    print(f"Error pre-importing ROS2 modules: {e}")
        
        def __init__(self, namespace: str = "/mobility_gen", enable_compression: bool = False, 
                     bag_path: str = None, mode: str = "Stream + Bag"):
            # Pre-import ROS2 modules
            self._import_ros2_modules()
            
            print(f"DEBUG: Creating ROS2 node for namespace: {namespace}")
            super().__init__(f"mobility_gen_writer_{int(time.time())}")
            print(f"DEBUG: ROS2 node created successfully")
            
            self.namespace = namespace
            self.enable_compression = enable_compression
            self.bag_path = bag_path
            self.mode = mode
            
            # Camera publishing settings
            self.publish_camera_images = True  # Default to True, can be overridden
            self.enable_processing = True  # Default to True, can be overridden
            self.camera_images_only = False  # Default to False, can be overridden
            
            # Aggressive throttling for performance
            self.ros2_publish_frequency = 10  # Only process ROS2 data every 5 steps (was 2)
            self.last_ros2_publish_step = -1
            
            # Throttling for performance: publish camera images every 3 steps (20Hz instead of 60Hz)
            self.camera_publish_frequency = 10  # Only publish camera images every 10 steps (was 3)
            self.last_camera_publish_step = -1
            
            print(f"DEBUG: Creating ROS2Writer...")
            self._init_publishers()
            print(f"✓ Publishers initialized")
            
            if self.mode in ["Bag Only", "Stream + Bag"] and self.bag_path:
                self._init_bag_database()
                print(f"✓ Bag database initialized")
            
            print(f"✓ ROS2 Writer initialized with namespace: {self.namespace}")
            print(f"✓ Mode: {self.mode}")
            print(f"✓ ROS2 processing frequency: {60/self.ros2_publish_frequency:.1f}Hz")
            print(f"✓ Camera publishing frequency: {60/self.camera_publish_frequency:.1f}Hz")
        
        def _convert_numpy_to_json(self, obj):
            """Convert numpy arrays to JSON-serializable format."""
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, dict):
                return {key: self._convert_numpy_to_json(value) for key, value in obj.items()}
            elif isinstance(obj, list):
                return [self._convert_numpy_to_json(item) for item in obj]
            else:
                return obj

        def _init_bag_database(self):
            """Initialize the SQLite database for the ROS2 bag."""
            os.makedirs(os.path.dirname(self.bag_path), exist_ok=True)
            
            self.db_conn = sqlite3.connect(self.bag_path)
            self.db_cursor = self.db_conn.cursor()
            
            # Create tables for ROS2 bag format
            self.db_cursor.execute('''
                CREATE TABLE IF NOT EXISTS topics (
                    id INTEGER PRIMARY KEY,
                    name TEXT NOT NULL,
                    type TEXT NOT NULL,
                    serialization_format TEXT NOT NULL,
                    offered_qos_profiles TEXT NOT NULL
                )
            ''')
            
            self.db_cursor.execute('''
                CREATE TABLE IF NOT EXISTS messages (
                    id INTEGER PRIMARY KEY,
                    topic_id INTEGER NOT NULL,
                    timestamp INTEGER NOT NULL,
                    data BLOB NOT NULL,
                    FOREIGN KEY (topic_id) REFERENCES topics (id)
                )
            ''')
            
            self.db_conn.commit()
            print("✓ Bag database initialized")
        
        def _init_publishers(self):
            """Initialize ROS2 publishers."""
            # Common state publisher
            self.common_state_pub = self.create_publisher(
                String, f"{self.namespace}/common_state", 10
            )
            
            # Camera publishers
            self.rgb_publishers = {}
            self.segmentation_publishers = {}
            self.depth_publishers = {}
            self.normals_publishers = {}
            
            print("✓ Publishers initialized")
        
        def _sanitize_topic_name(self, name: str) -> str:
            """Sanitize a name for use as a ROS2 topic name."""
            # Replace dots and other invalid characters with underscores
            sanitized = name.replace('.', '_').replace('-', '_')
            # Remove any other invalid characters
            sanitized = ''.join(c for c in sanitized if c.isalnum() or c in '_~{}')
            return sanitized
        
        def _get_rgb_publisher(self, camera_name: str):
            """Get or create RGB image publisher for a camera."""
            sanitized_name = self._sanitize_topic_name(camera_name)
            if sanitized_name not in self.rgb_publishers:
                topic_name = f"{self.namespace}/rgb/{sanitized_name}"
                self.rgb_publishers[sanitized_name] = self.create_publisher(
                    Image, topic_name, 10
                )
            return self.rgb_publishers[sanitized_name]
        
        def _get_segmentation_publisher(self, camera_name: str):
            """Get or create segmentation image publisher for a camera."""
            sanitized_name = self._sanitize_topic_name(camera_name)
            if sanitized_name not in self.segmentation_publishers:
                topic_name = f"{self.namespace}/segmentation/{sanitized_name}"
                self.segmentation_publishers[sanitized_name] = self.create_publisher(
                    Image, topic_name, 10
                )
            return self.segmentation_publishers[sanitized_name]
        
        def _get_depth_publisher(self, camera_name: str):
            """Get or create depth image publisher for a camera."""
            sanitized_name = self._sanitize_topic_name(camera_name)
            if sanitized_name not in self.depth_publishers:
                topic_name = f"{self.namespace}/depth/{sanitized_name}"
                self.depth_publishers[sanitized_name] = self.create_publisher(
                    Image, topic_name, 10
                )
            return self.depth_publishers[sanitized_name]
        
        def _get_normals_publisher(self, camera_name: str):
            """Get or create normals publisher for a camera."""
            sanitized_name = self._sanitize_topic_name(camera_name)
            if sanitized_name not in self.normals_publishers:
                topic_name = f"{self.namespace}/normals/{sanitized_name}"
                self.normals_publishers[sanitized_name] = self.create_publisher(
                    Image, topic_name, 10
                )
            return self.normals_publishers[sanitized_name]
        
        def _create_header(self):
            """Create a ROS2 header with current timestamp."""
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            return header
        
        def _get_or_create_topic_id(self, topic_name: str, msg_type: str) -> int:
            """Get or create a topic ID for bag recording."""
            self.db_cursor.execute(
                "SELECT id FROM topics WHERE name = ? AND type = ?",
                (topic_name, msg_type)
            )
            result = self.db_cursor.fetchone()
            
            if result:
                return result[0]
            else:
                self.db_cursor.execute(
                    "INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?)",
                    (topic_name, msg_type, "cdr", "[]")
                )
                self.db_conn.commit()
                return self.db_cursor.lastrowid
        
        def _write_to_bag(self, topic_name: str, msg, msg_type: str):
            """Write a message to the bag file."""
            topic_id = self._get_or_create_topic_id(topic_name, msg_type)
            
            # Serialize message
            serialized_data = serialize_message(msg)
            
            # Write to database
            timestamp = int(time.time() * 1e9)  # Nanoseconds
            self.db_cursor.execute(
                "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                (topic_id, timestamp, serialized_data)
            )
            self.db_conn.commit()
        
        def write_state_dict_common(self, state_dict: dict, step: int):
            """Write common state data to ROS2 topics and bag."""
            # Create ROS message
            msg = String()
            
            # Convert numpy arrays to lists for JSON serialization
            def convert_numpy(obj):
                if hasattr(obj, 'tolist'):
                    return obj.tolist()
                elif isinstance(obj, dict):
                    return {k: convert_numpy(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [convert_numpy(item) for item in obj]
                else:
                    return obj
            
            serializable_state = convert_numpy(state_dict)
            msg.data = json.dumps({
                'data': serializable_state,
                'step': step,
                'timestamp': time.time()
            })
            
            # Publish to topic if streaming
            if self.mode in ["Stream Only", "Stream + Bag"]:
                self.common_state_pub.publish(msg)
            
            # Write to bag if recording
            if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                topic_name = f"{self.namespace}/common_state"
                self._write_to_bag(topic_name, msg, "std_msgs/msg/String")
            
            if step % 60 == 0:  # Print every 60 steps (about once per second)  
                print(f"✓ Published common state (step {step}) - Mode: {self.mode}")
        
        def write_config(self, config):
            """Write config to bag (placeholder)."""
            pass
        
        def write_occupancy_map(self, occupancy_map):
            """Write occupancy map to bag (placeholder)."""
            pass
        
        def write_common_state_data(self, state_dict: dict, step: int):
            """Extract and publish all data from state to ROS2 topics."""
            try:
                # Performance profiling
                start_time = time_module.time()
                
                # Check if processing is enabled
                if not self.enable_processing:
                    return  # Skip all ROS2 processing
                
                # Check if ROS2 is still responsive
                try:
                    if not rclpy.ok():
                        print(f"WARNING: ROS2 context is not ok, skipping step {step}")
                        return
                except Exception as e:
                    print(f"WARNING: Error checking ROS2 context: {e}")
                    return
                
                # Aggressive throttling: only process ROS2 data every few steps
                should_process_ros2 = (step - self.last_ros2_publish_step) >= self.ros2_publish_frequency
                
                if not should_process_ros2:
                    return  # Skip all ROS2 processing
                
                # Profile data extraction
                extract_start = time_module.time()
                
                # Initialize CV bridge if not already done
                if not hasattr(self, 'cv_bridge'):
                    self.cv_bridge = CvBridge()
                
                # Extract robot data from state
                robot_data = {}
                camera_data = {}
                keyboard_data = {}
                
                for key, value in state_dict.items():
                    if key.startswith('robot.'):
                        if 'camera' in key and ('rgb_image' in key or 'depth_image' in key or 'segmentation_image' in key):
                            # Only process camera images if enabled
                            if self.publish_camera_images:
                                # Handle camera image data
                                camera_name = key.split('.')[2]  # robot.front_camera.left.rgb_image -> front_camera.left
                                image_type = key.split('.')[-1]  # rgb_image, depth_image, segmentation_image
                                
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name][image_type] = value
                                

                        elif 'camera' in key:
                            # Handle camera metadata (position, orientation, segmentation_info)
                            if 'position' in key:
                                camera_name = key.replace('.position', '')
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name]['position'] = value
                            elif 'orientation' in key:
                                camera_name = key.replace('.orientation', '')
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name]['orientation'] = value
                            elif 'segmentation_info' in key:
                                camera_name = key.replace('.segmentation_info', '')
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name]['segmentation_info'] = value
                            elif 'instance_id_segmentation_info' in key:
                                camera_name = key.replace('.instance_id_segmentation_info', '')
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name]['instance_id_segmentation_info'] = value
                        else:
                            # Handle robot data (action, position, orientation, joint data)
                            robot_data[key] = value
                    elif key.startswith('keyboard.'):
                        keyboard_data[key] = value
                
                extract_time = time_module.time() - extract_start
                
                # Profile robot data publishing
                robot_start = time_module.time()
                self._publish_robot_data(robot_data, step)
                robot_time = time_module.time() - robot_start
                
                # Profile camera data publishing
                camera_start = time_module.time()
                if camera_data:
                    self._publish_camera_data_with_images(camera_data, step)
                camera_time = time_module.time() - camera_start
                
                # Profile keyboard data publishing
                keyboard_start = time_module.time()
                self._publish_keyboard_data(keyboard_data, step)
                keyboard_time = time_module.time() - keyboard_start
                
                # Update last publish step
                self.last_ros2_publish_step = step
                
                # Performance reporting
                total_time = time_module.time() - start_time
                
                # Print detailed performance profile when processing
                print(f"🔍 ROS2 Performance Profile (step {step}):")
                print(f"  Total time: {total_time*1000:.2f}ms")
                print(f"  Data extraction: {extract_time*1000:.2f}ms")
                print(f"  Robot publishing: {robot_time*1000:.2f}ms")
                print(f"  Camera publishing: {camera_time*1000:.2f}ms")
                print(f"  Keyboard publishing: {keyboard_time*1000:.2f}ms")
                      
            except Exception as e:
                print(f"ERROR: Exception in write_common_state_data for step {step}: {e}")
                traceback.print_exc()
                # Don't let ROS2 errors affect the main simulation
                return
        
        def _publish_robot_data(self, robot_data: dict, step: int):
            """Publish robot data to ROS2 topics."""
            try:
                # Use pre-imported modules
                PoseStamped = self._geometry_msgs.PoseStamped
                TwistStamped = self._geometry_msgs.TwistStamped
                Float64MultiArray = self._std_msgs.Float64MultiArray
                
                # Profile pose publishing
                pose_start = time_module.time()
                
                # Publish robot pose
                if 'robot.position' in robot_data and 'robot.orientation' in robot_data:
                    pose_msg = PoseStamped()
                    pose_msg.header = self._create_header()
                    pose_msg.header.frame_id = "world"
                    
                    position = robot_data['robot.position']
                    orientation = robot_data['robot.orientation']
                    
                    pose_msg.pose.position.x = float(position[0])
                    pose_msg.pose.position.y = float(position[1])
                    pose_msg.pose.position.z = float(position[2])
                    pose_msg.pose.orientation.x = float(orientation[0])
                    pose_msg.pose.orientation.y = float(orientation[1])
                    pose_msg.pose.orientation.z = float(orientation[2])
                    pose_msg.pose.orientation.w = float(orientation[3])
                    
                    if self.mode in ["Stream Only", "Stream + Bag"]:
                        topic_name = f"{self.namespace}/robot/pose"
                        if not hasattr(self, 'robot_pose_publisher'):
                            self.robot_pose_publisher = self.create_publisher(PoseStamped, topic_name, 10)
                        self.robot_pose_publisher.publish(pose_msg)
                    
                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                        topic_name = f"{self.namespace}/robot/pose"
                        self._write_to_bag(topic_name, pose_msg, "geometry_msgs/msg/PoseStamped")
                
                pose_time = time_module.time() - pose_start
                
                # Profile twist publishing
                twist_start = time_module.time()
                
                # Publish robot twist (action)
                if 'robot.action' in robot_data:
                    twist_msg = TwistStamped()
                    twist_msg.header = self._create_header()
                    twist_msg.header.frame_id = "base_link"
                    
                    action = robot_data['robot.action']
                    twist_msg.twist.linear.x = float(action[0])
                    twist_msg.twist.angular.z = float(action[1])
                    
                    if self.mode in ["Stream Only", "Stream + Bag"]:
                        topic_name = f"{self.namespace}/robot/twist"
                        if not hasattr(self, 'robot_twist_publisher'):
                            self.robot_twist_publisher = self.create_publisher(TwistStamped, topic_name, 10)
                        self.robot_twist_publisher.publish(twist_msg)
                    
                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                        topic_name = f"{self.namespace}/robot/twist"
                        self._write_to_bag(topic_name, twist_msg, "geometry_msgs/msg/TwistStamped")
                
                twist_time = time_module.time() - twist_start
                
                # Profile joint publishing
                joint_start = time_module.time()
                
                # Publish joint positions
                if 'robot.joint_positions' in robot_data:
                    joint_pos_msg = Float64MultiArray()
                    joint_positions = robot_data['robot.joint_positions']
                    joint_pos_msg.data = [float(x) for x in joint_positions]
                    
                    if self.mode in ["Stream Only", "Stream + Bag"]:
                        topic_name = f"{self.namespace}/robot/joint_positions"
                        if not hasattr(self, 'joint_positions_publisher'):
                            self.joint_positions_publisher = self.create_publisher(Float64MultiArray, topic_name, 10)
                        self.joint_positions_publisher.publish(joint_pos_msg)
                    
                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                        topic_name = f"{self.namespace}/robot/joint_positions"
                        self._write_to_bag(topic_name, joint_pos_msg, "std_msgs/msg/Float64MultiArray")
                
                # Publish joint velocities
                if 'robot.joint_velocities' in robot_data:
                    joint_vel_msg = Float64MultiArray()
                    joint_velocities = robot_data['robot.joint_velocities']
                    joint_vel_msg.data = [float(x) for x in joint_velocities]
                    
                    if self.mode in ["Stream Only", "Stream + Bag"]:
                        topic_name = f"{self.namespace}/robot/joint_velocities"
                        if not hasattr(self, 'joint_velocities_publisher'):
                            self.joint_velocities_publisher = self.create_publisher(Float64MultiArray, topic_name, 10)
                        self.joint_velocities_publisher.publish(joint_vel_msg)
                    
                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                        topic_name = f"{self.namespace}/robot/joint_velocities"
                        self._write_to_bag(topic_name, joint_vel_msg, "std_msgs/msg/Float64MultiArray")
                
                joint_time = time_module.time() - joint_start
                
                # Performance reporting for robot data
                if step % 60 == 0:  # Print every 60 steps
                    print(f"  🤖 Robot publishing breakdown:")
                    print(f"    Pose: {pose_time*1000:.2f}ms")
                    print(f"    Twist: {twist_time*1000:.2f}ms")
                    print(f"    Joints: {joint_time*1000:.2f}ms")
                
            except Exception as e:
                print(f"ERROR: Exception in _publish_robot_data for step {step}: {e}")
                traceback.print_exc()
                # Don't let ROS2 errors affect the main simulation
                return
        
        def _publish_camera_data(self, camera_data: dict, step: int):
            """Publish camera data to ROS2 topics."""
            for camera_name, camera_metadata in camera_data.items():
                try:
                    from geometry_msgs.msg import PoseStamped
                    
                    # Publish camera pose
                    if 'position' in camera_metadata and 'orientation' in camera_metadata:
                        pose_msg = PoseStamped()
                        pose_msg.header = self._create_header()
                        pose_msg.header.frame_id = "world"
                        
                        # Convert to float for ROS2
                        position = camera_metadata['position']
                        orientation = camera_metadata['orientation']
                        
                        pose_msg.pose.position.x = float(position[0])
                        pose_msg.pose.position.y = float(position[1])
                        pose_msg.pose.position.z = float(position[2])
                        pose_msg.pose.orientation.x = float(orientation[0])
                        pose_msg.pose.orientation.y = float(orientation[1])
                        pose_msg.pose.orientation.z = float(orientation[2])
                        pose_msg.pose.orientation.w = float(orientation[3])
                        
                        # Publish pose to topic
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/camera_pose/{sanitized_name}"
                            if not hasattr(self, 'camera_pose_publishers'):
                                self.camera_pose_publishers = {}
                            if topic_name not in self.camera_pose_publishers:
                                self.camera_pose_publishers[topic_name] = self.create_publisher(
                                    PoseStamped, topic_name, 10
                                )
                            self.camera_pose_publishers[topic_name].publish(pose_msg)
                        
                        # Write to bag if recording
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/camera_pose/{sanitized_name}"
                            self._write_to_bag(topic_name, pose_msg, "geometry_msgs/msg/PoseStamped")
                    
                    # Publish segmentation info as JSON string
                    if 'segmentation_info' in camera_metadata and camera_metadata['segmentation_info'] is not None:
                        seg_info_msg = String()
                        seg_info_msg.data = json.dumps(camera_metadata['segmentation_info'])
                        
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/segmentation_info/{sanitized_name}"
                            if not hasattr(self, 'segmentation_info_publishers'):
                                self.segmentation_info_publishers = {}
                            if topic_name not in self.segmentation_info_publishers:
                                self.segmentation_info_publishers[topic_name] = self.create_publisher(
                                    String, topic_name, 10
                                )
                            self.segmentation_info_publishers[topic_name].publish(seg_info_msg)
                        
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/segmentation_info/{sanitized_name}"
                            self._write_to_bag(topic_name, seg_info_msg, "std_msgs/msg/String")
                    
                    # Publish instance ID segmentation info as JSON string
                    if 'instance_id_segmentation_info' in camera_metadata and camera_metadata['instance_id_segmentation_info'] is not None:
                        instance_info_msg = String()
                        instance_info_msg.data = json.dumps(camera_metadata['instance_id_segmentation_info'])
                        
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/instance_segmentation_info/{sanitized_name}"
                            if not hasattr(self, 'instance_segmentation_info_publishers'):
                                self.instance_segmentation_info_publishers = {}
                            if topic_name not in self.instance_segmentation_info_publishers:
                                self.instance_segmentation_info_publishers[topic_name] = self.create_publisher(
                                    String, topic_name, 10
                                )
                            self.instance_segmentation_info_publishers[topic_name].publish(instance_info_msg)
                        
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/instance_segmentation_info/{sanitized_name}"
                            self._write_to_bag(topic_name, instance_info_msg, "std_msgs/msg/String")
                    
                except Exception as e:
                    print(f"Error publishing camera metadata for {camera_name}: {e}")
        
        def _publish_keyboard_data(self, keyboard_data: dict, step: int):
            """Publish keyboard data to ROS2 topics."""
            try:
                # Performance profiling
                keyboard_start = time_module.time()
                
                # Use pre-imported modules
                String = self._std_msgs.String
                
                # Publish keyboard data as JSON string
                if keyboard_data:
                    # Convert keyboard data for JSON serialization
                    keyboard_data_json = self._convert_numpy_to_json(keyboard_data)
                    
                    keyboard_msg = String()
                    keyboard_msg.data = json.dumps(keyboard_data_json)
                    
                    if self.mode in ["Stream Only", "Stream + Bag"]:
                        topic_name = f"{self.namespace}/keyboard"
                        if not hasattr(self, 'keyboard_publisher'):
                            self.keyboard_publisher = self.create_publisher(String, topic_name, 10)
                        self.keyboard_publisher.publish(keyboard_msg)
                    
                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                        topic_name = f"{self.namespace}/keyboard"
                        self._write_to_bag(topic_name, keyboard_msg, "std_msgs/msg/String")
                
                keyboard_time = time_module.time() - keyboard_start
                
                # Performance reporting for keyboard data
                if step % 60 == 0:  # Print every 60 steps
                    print(f"  ⌨️  Keyboard publishing: {keyboard_time*1000:.2f}ms")
                    print(f"    Keyboard data keys: {list(keyboard_data.keys()) if keyboard_data else 'None'}")
                
            except Exception as e:
                print(f"Error publishing keyboard data: {e}")
                self._traceback.print_exc()

        def _publish_camera_data_with_images(self, camera_data: dict, step: int):
            """Publish camera data with images to ROS2 topics."""
            try:
                # Check if we should publish images at this step (throttled frequency)
                should_publish_images = (step - self.last_camera_publish_step) >= self.camera_publish_frequency
                
                # Always publish camera metadata (lightweight)
                for camera_name, camera_data_dict in camera_data.items():
                    if 'position' in camera_data_dict:
                        try:
                            position = camera_data_dict['position']
                            if position is not None:
                                pose_msg = PoseStamped()
                                pose_msg.header = self._create_header()
                                pose_msg.pose.position.x = float(position[0])
                                pose_msg.pose.position.y = float(position[1])
                                pose_msg.pose.position.z = float(position[2])
                                
                                if self.mode in ["Stream Only", "Stream + Bag"]:
                                    sanitized_name = self._sanitize_topic_name(camera_name)
                                    topic_name = f"{self.namespace}/camera/{sanitized_name}/pose"
                                    if not hasattr(self, 'camera_pose_publishers'):
                                        self.camera_pose_publishers = {}
                                    if topic_name not in self.camera_pose_publishers:
                                        self.camera_pose_publishers[topic_name] = self.create_publisher(
                                            PoseStamped, topic_name, 10
                                        )
                                    self.camera_pose_publishers[topic_name].publish(pose_msg)
                                
                                # Write to bag if recording
                                if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                                    sanitized_name = self._sanitize_topic_name(camera_name)
                                    topic_name = f"{self.namespace}/camera/{sanitized_name}/pose"
                                    self._write_to_bag(topic_name, pose_msg, "geometry_msgs/msg/PoseStamped")
                                    
                        except Exception as e:
                            print(f"Error publishing camera pose for {camera_name}: {e}")
                            traceback.print_exc()
                
                # Only publish images at throttled frequency (heavy operation)
                if should_publish_images and self.publish_camera_images:
                    print(f"📸 Publishing camera images at step {step} (last camera step: {self.last_camera_publish_step})")
                    
                    # Initialize CV bridge if not already done
                    if not hasattr(self, 'cv_bridge'):
                        self.cv_bridge = self._cv_bridge()
                    
                    # Initialize publishers dictionary if not exists
                    if not hasattr(self, 'rgb_publishers'):
                        self.rgb_publishers = {}
                    if not hasattr(self, 'depth_publishers'):
                        self.depth_publishers = {}
                    if not hasattr(self, 'segmentation_publishers'):
                        self.segmentation_publishers = {}
                    
                    # Profile RGB publishing
                    rgb_start = time_module.time()
                    
                    # Publish RGB images for all cameras
                    for camera_name, camera_data_dict in camera_data.items():
                        if 'rgb_image' in camera_data_dict:
                            rgb_image = camera_data_dict['rgb_image']
                            if rgb_image is not None and rgb_image.size > 0:
                                try:
                                    # Convert to ROS Image message
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(rgb_image, "rgb8")
                                    ros_image.header = self._create_header()
                                    
                                    # Publish RGB image
                                    if self.mode in ["Stream Only", "Stream + Bag"]:
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/rgb/{sanitized_name}"
                                        if topic_name not in self.rgb_publishers:
                                            self.rgb_publishers[topic_name] = self.create_publisher(
                                                Image, topic_name, 10
                                            )
                                        self.rgb_publishers[topic_name].publish(ros_image)
                                    
                                    # Write to bag if recording
                                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/rgb/{sanitized_name}"
                                        self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                                        
                                except Exception as e:
                                    print(f"Error publishing RGB image for {camera_name}: {e}")
                                    traceback.print_exc()
                    
                    rgb_time = time_module.time() - rgb_start
                    
                    # Profile depth publishing
                    depth_start = time_module.time()
                    
                    # Publish depth images for all cameras
                    for camera_name, camera_data_dict in camera_data.items():
                        if 'depth_image' in camera_data_dict:
                            depth_image = camera_data_dict['depth_image']
                            if depth_image is not None and depth_image.size > 0:
                                try:
                                    # Optimized depth processing - skip normalization for performance
                                    # Just convert to uint16 for ROS compatibility
                                    depth_uint16 = depth_image.astype(np.uint16)
                                    
                                    # Convert to ROS Image message (uint16 for depth)
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(depth_uint16, "mono16")
                                    ros_image.header = self._create_header()
                                    
                                    # Publish depth image
                                    if self.mode in ["Stream Only", "Stream + Bag"]:
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/depth/{sanitized_name}"
                                        if topic_name not in self.depth_publishers:
                                            self.depth_publishers[topic_name] = self.create_publisher(
                                                Image, topic_name, 10
                                            )
                                        self.depth_publishers[topic_name].publish(ros_image)
                                    
                                    # Write to bag if recording
                                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/depth/{sanitized_name}"
                                        self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                                        
                                except Exception as e:
                                    print(f"Error publishing depth image for {camera_name}: {e}")
                                    traceback.print_exc()
                    
                    depth_time = time_module.time() - depth_start
                    
                    # Profile segmentation publishing
                    seg_start = time_module.time()
                    
                    # Publish segmentation images for all cameras
                    for camera_name, camera_data_dict in camera_data.items():
                        if 'segmentation_image' in camera_data_dict:
                            seg_image = camera_data_dict['segmentation_image']
                            if seg_image is not None and seg_image.size > 0:
                                try:
                                    # Handle different segmentation image types
                                    if seg_image.dtype == np.uint32:
                                        # Convert uint32 to uint16 for ROS compatibility
                                        seg_image = seg_image.astype(np.uint16)
                                        encoding = "16UC1"
                                    else:
                                        # For other types, convert to uint8
                                        seg_image = seg_image.astype(np.uint8)
                                        encoding = "mono8"
                                    
                                    # Convert to ROS Image message
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(seg_image, encoding)
                                    ros_image.header = self._create_header()
                                    
                                    # Publish segmentation image
                                    if self.mode in ["Stream Only", "Stream + Bag"]:
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/segmentation/{sanitized_name}"
                                        if topic_name not in self.segmentation_publishers:
                                            self.segmentation_publishers[topic_name] = self.create_publisher(
                                                Image, topic_name, 10
                                            )
                                        self.segmentation_publishers[topic_name].publish(ros_image)
                                    
                                    # Write to bag if recording
                                    if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                                        sanitized_name = self._sanitize_topic_name(camera_name)
                                        topic_name = f"{self.namespace}/segmentation/{sanitized_name}"
                                        self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                                        
                                except Exception as e:
                                    print(f"Error publishing segmentation image for {camera_name}: {e}")
                                    traceback.print_exc()
                    
                    seg_time = time_module.time() - seg_start
                    
                    # Update last camera publish step
                    self.last_camera_publish_step = step
                    
                    # Performance reporting for image publishing
                    total_image_time = time_module.time() - rgb_start
                    if step % 60 == 0:  # Print every 60 steps
                        print(f"DEBUG: Camera image publishing performance:")
                        print(f"  RGB time: {rgb_time*1000:.2f}ms")
                        print(f"  Depth time: {depth_time*1000:.2f}ms")
                        print(f"  Segmentation time: {seg_time*1000:.2f}ms")
                        print(f"  Total image time: {total_image_time*1000:.2f}ms")
                

                
            except Exception as e:
                print(f"Error in _publish_camera_data_with_images: {e}")
                traceback.print_exc()
        
        def write_camera_data(self, rgb_data: dict, segmentation_data: dict, depth_data: dict, normals_data: dict, step: int):
            """Write camera data to ROS2 topics and bag."""
            # Initialize CV bridge if not already done
            if not hasattr(self, 'cv_bridge'):
                self.cv_bridge = CvBridge()
            
            # Process RGB images
            for camera_name, rgb_image in rgb_data.items():
                if rgb_image is not None:
                    try:
                        # Convert to BGR for OpenCV
                        if len(rgb_image.shape) == 3 and rgb_image.shape[2] == 3:
                            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                        else:
                            bgr_image = rgb_image
                        
                        # Create ROS Image message
                        ros_image = self.cv_bridge.cv2_to_imgmsg(bgr_image, "bgr8")
                        ros_image.header = self._create_header()
                        ros_image.header.frame_id = f"{camera_name}_optical_frame"
                        
                        # Publish to topic if streaming
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            publisher = self._get_rgb_publisher(camera_name)
                            publisher.publish(ros_image)
                        
                        # Write to bag if recording
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/rgb/{sanitized_name}"
                            self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                        
                    except Exception as e:
                        print(f"Error publishing RGB image for {camera_name}: {e}")
                        self._traceback.print_exc()
            
            # Process segmentation images
            for camera_name, seg_image in segmentation_data.items():
                if seg_image is not None:
                    try:
                        # Convert to 16-bit image for segmentation
                        if seg_image.dtype != np.uint16:
                            seg_image = seg_image.astype(np.uint16)
                        
                        # Create ROS Image message
                        ros_image = self.cv_bridge.cv2_to_imgmsg(seg_image, "mono16")
                        ros_image.header = self._create_header()
                        ros_image.header.frame_id = f"{camera_name}_optical_frame"
                        
                        # Publish to topic if streaming
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            publisher = self._get_segmentation_publisher(camera_name)
                            publisher.publish(ros_image)
                        
                        # Write to bag if recording
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/segmentation/{sanitized_name}"
                            self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                        
                    except Exception as e:
                        print(f"Error publishing segmentation image for {camera_name}: {e}")
                        self._traceback.print_exc()
            
            # Process depth images
            for camera_name, depth_image in depth_data.items():
                if depth_image is not None:
                    try:
                        # Convert depth to 16-bit format (millimeters)
                        depth_16bit = (depth_image * 1000).astype(np.uint16)
                        
                        # Create ROS Image message
                        ros_image = self.cv_bridge.cv2_to_imgmsg(depth_16bit, "mono16")
                        ros_image.header = self._create_header()
                        ros_image.header.frame_id = f"{camera_name}_optical_frame"
                        
                        # Publish to topic if streaming
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            publisher = self._get_depth_publisher(camera_name)
                            publisher.publish(ros_image)
                        
                        # Write to bag if recording
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/depth/{sanitized_name}"
                            self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                        
                    except Exception as e:
                        print(f"Error publishing depth image for {camera_name}: {e}")
                        self._traceback.print_exc()
            
            # Process normal maps
            for camera_name, normals_image in normals_data.items():
                if normals_image is not None:
                    try:
                        # Convert normals to RGB visualization
                        # Assuming normals are in [-1, 1] range, convert to [0, 255]
                        normals_rgb = ((normals_image[..., :3] + 1.0) * 127.5).astype(np.uint8)
                        
                        # Create ROS Image message
                        ros_image = self.cv_bridge.cv2_to_imgmsg(normals_rgb, "rgb8")
                        ros_image.header = self._create_header()
                        ros_image.header.frame_id = f"{camera_name}_optical_frame"
                        
                        # Publish to topic if streaming
                        if self.mode in ["Stream Only", "Stream + Bag"]:
                            publisher = self._get_normals_publisher(camera_name)
                            publisher.publish(ros_image)
                        
                        # Write to bag if recording
                        if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/normals/{sanitized_name}"
                            self._write_to_bag(topic_name, ros_image, "sensor_msgs/msg/Image")
                        
                    except Exception as e:
                        print(f"Error publishing normals for {camera_name}: {e}")
                        self._traceback.print_exc()
            
            if step % 60 == 0:  # Print every 60 steps (about once per second)
                print(f"✓ Published camera data (step {step}) - Mode: {self.mode}")
        
        def destroy_node(self):
            """Clean up the node and database."""
            if hasattr(self, 'db_conn') and self.db_conn is not None:
                self.db_conn.close()
            super().destroy_node()
    
    # Set the global variables
    ROS2Writer = SimpleROS2Writer
    ROS2_AVAILABLE = True
    print("✓ ROS2 functionality available")
    
except ImportError as e:
    print(f"✗ ROS2 import failed: {e}")
    import traceback
    traceback.print_exc()
    ROS2_AVAILABLE = False
except Exception as e:
    print(f"✗ Unexpected error during ROS2 setup: {e}")
    import traceback
    traceback.print_exc()
    ROS2_AVAILABLE = False


class ROS2Manager:
    """Manager class for ROS2 functionality in the MobilityGen extension."""
    
    def __init__(self):
        self.ros2_writer: Optional[ROS2Writer] = None
        self.ros2_enabled: bool = False
        self.ros2_namespace: str = "/mobility_gen"
        self.ros2_compression: bool = False
    
    def is_available(self) -> bool:
        """Check if ROS2 is available."""
        return ROS2_AVAILABLE
    
    def initialize_ros2(self):
        """Initialize ROS2 if not already done."""
        if not ROS2_AVAILABLE:
            return False
        
        try:
            if not rclpy.ok():
                rclpy.init()
                print("✓ ROS2 initialized successfully")
            
            # Set ros2_enabled to True when initialization is successful
            self.ros2_enabled = True
            return True
        except Exception as e:
            print(f"Error initializing ROS2: {e}")
            return False
    
    def create_writer(self, namespace: str, compression: bool, bag_path: str, mode: str) -> Optional[ROS2Writer]:
        """Create a ROS2 writer with the specified parameters."""
        if not ROS2_AVAILABLE:
            return None
        
        try:
            writer = ROS2Writer(
                namespace=namespace,
                enable_compression=compression,
                bag_path=bag_path,
                mode=mode
            )
            return writer
        except Exception as e:
            print(f"Error creating ROS2 writer: {e}")
            traceback.print_exc()
            return None
    
    def cleanup(self):
        """Clean up ROS2 resources."""
        if self.ros2_writer is not None:
            try:
                self.ros2_writer.destroy_node()
                self.ros2_writer = None
                print("ROS2 writer cleaned up")
            except Exception as e:
                print(f"Error cleaning up ROS2 writer: {e}")
                traceback.print_exc()
        
        if ROS2_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS2: {e}") 
    
    def write_state(self, state_dict: dict, step: int):
        """Write state data to ROS2 if enabled."""
        if self.ros2_writer is not None:
            try:
                # Only publish common state data here (fast)
                self.ros2_writer.write_state_dict_common(state_dict, step)
                
                # Camera data will be handled separately in the background thread
                # to avoid blocking the main physics thread
                
            except Exception as e:
                print(f"Error writing to ROS2: {e}")
                traceback.print_exc()
    
    def write_camera_data(self, rgb_data: dict, segmentation_data: dict, depth_data: dict, normals_data: dict, step: int):
        """Write camera data to ROS2 if enabled."""
        if self.ros2_writer is not None:
            try:
                self.ros2_writer.write_camera_data(rgb_data, segmentation_data, depth_data, normals_data, step)
            except Exception as e:
                print(f"Error writing camera data to ROS2: {e}")
                traceback.print_exc() 