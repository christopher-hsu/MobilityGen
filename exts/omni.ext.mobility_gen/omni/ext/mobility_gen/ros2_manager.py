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
Written by: @christopher-hsu

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
import threading
import queue

# Import scipy for quaternion transformations
from scipy.spatial.transform import Rotation

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
    from std_msgs.msg import String, Header, Float64MultiArray
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
    from cv_bridge import CvBridge

    # Import TF2 modules for transform publishing
    try:
        from tf2_ros import TransformBroadcaster
        try:
            from tf2_msgs.msg import TFMessage
            TF2_MSG_AVAILABLE = True
        except ImportError:
            TF2_MSG_AVAILABLE = False
            print("⚠ TF2 messages not available, TF bag recording disabled")
        TF2_AVAILABLE = True
        print("✓ TF2 imports successful")
    except ImportError as e:
        print(f"✗ TF2 import failed: {e}")
        TF2_AVAILABLE = False
        TF2_MSG_AVAILABLE = False
    
    print("✓ All ROS2 imports successful")
    
    class SimpleROS2Writer(Node):
        """Simple ROS2 writer for streaming and bag recording.
        
        This class provides ROS2 functionality for streaming and bag recording,
        including:
        - PoseStamped messages for robot poses
        - TransformStamped messages (TF2) for robot transforms
        - Camera data publishing (RGB, depth, segmentation)
        - Joint data publishing
        - Keyboard input publishing
        - Bag file recording with SQLite backend
        """
        
        def __init__(self, namespace: str = "/mobility_gen", enable_compression: bool = False, 
                     bag_path: str = None, mode: str = "Stream + Bag"):
            super().__init__(f"mobility_gen_writer_{int(time.time())}")
            
            self.namespace = namespace
            self.enable_compression = enable_compression
            self.bag_path = bag_path
            self.mode = mode
            self.scenario = None
            
            # Camera publishing settings
            self.publish_camera_images = True
            self.enable_processing = True
            self.camera_images_only = False
            
            # Performance throttling
            self.ros2_publish_frequency = 10
            self.last_ros2_publish_step = -1
            self.camera_publish_frequency = 10
            self.last_camera_publish_step = -1
            
            # Thread-safe database operations with better queue management
            self.db_queue = queue.Queue(maxsize=1000)  # Limit queue size
            self.db_thread = None
            self.db_conn = None
            self.db_cursor = None
            self.db_running = False
            self.topic_cache = {}  # Cache topic IDs to reduce database lookups
            self.dropped_messages = 0
            self.total_messages = 0
            
            # Initialize TF2 broadcaster for transform publishing
            if TF2_AVAILABLE:
                self.tf_broadcaster = TransformBroadcaster(self)
                print("✓ TF2 broadcaster initialized")
            else:
                self.tf_broadcaster = None
                print("⚠ TF2 not available, transform publishing disabled")
            
            self._init_publishers()
            print(f"✓ Publishers initialized")
            
            if self.mode in ["Bag Only", "Stream + Bag"] and self.bag_path:
                self._init_bag_database()
                print(f"✓ Bag database initialized")
            

        
        def _init_bag_database(self):
            """Initialize the SQLite database for the ROS2 bag with thread safety."""
            # Create the bag directory structure
            bag_dir = os.path.dirname(self.bag_path)
            if not bag_dir:
                bag_dir = "."
            
            # Create a unique bag directory name based on timestamp
            timestamp = int(time.time())
            bag_name = f"mobility_gen_ros2_{timestamp}"
            self.bag_dir = os.path.join(bag_dir, bag_name)
            
            # Create the bag directory
            os.makedirs(self.bag_dir, exist_ok=True)
            
            # Create the database file inside the bag directory
            self.db_path = os.path.join(self.bag_dir, f"{bag_name}.db3")
            
            # Create response queue for topic ID lookups
            self.response_queue = queue.Queue()
            
            # Start database thread
            self.db_running = True
            self.db_thread = threading.Thread(target=self._db_worker, daemon=True)
            self.db_thread.start()
            
            # Wait for database to be ready
            while not hasattr(self, '_db_ready') or not self._db_ready:
                time.sleep(0.01)
            
            print(f"✓ Bag database initialized at: {self.bag_dir}")
        
        def _db_worker(self):
            """Database worker thread that handles all database operations."""
            try:
                # Create database connection in this thread
                self.db_conn = sqlite3.connect(self.db_path)
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
                self._db_ready = True
                
                # Process database operations from queue
                while self.db_running:
                    try:
                        operation = self.db_queue.get(timeout=0.1)
                        if operation is None:  # Shutdown signal
                            break
                        
                        op_type, args = operation
                        if op_type == 'get_or_create_topic_id':
                            topic_name, msg_type = args
                            result = self._db_get_or_create_topic_id(topic_name, msg_type)
                            # Send result back through a separate response queue
                            if hasattr(self, 'response_queue'):
                                self.response_queue.put(('result', result))
                        elif op_type == 'write_message':
                            topic_id, timestamp, data = args
                            self._db_write_message(topic_id, timestamp, data)
                        elif op_type == 'close':
                            break
                            
                    except queue.Empty:
                        continue
                    except Exception as e:
                        print(f"Error in database worker: {e}")
                        traceback.print_exc()
                
                # Create metadata.yaml file when closing
                if hasattr(self, 'db_conn') and self.db_conn:
                    self._create_metadata_yaml()
                
            except Exception as e:
                print(f"Error initializing database worker: {e}")
                traceback.print_exc()
            finally:
                if hasattr(self, 'db_conn') and self.db_conn:
                    self.db_conn.close()
        
        def _create_metadata_yaml(self):
            """Create the metadata.yaml file for the ROS2 bag."""
            try:
                # Get bag information from the database
                self.db_cursor.execute("SELECT COUNT(*) FROM messages")
                message_count = self.db_cursor.fetchone()[0]
                
                self.db_cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
                time_result = self.db_cursor.fetchone()
                min_time = time_result[0] if time_result[0] else 0
                max_time = time_result[1] if time_result[1] else 0
                
                # Get topics with message counts
                self.db_cursor.execute("""
                    SELECT name, type, COUNT(*) as msg_count 
                    FROM topics 
                    JOIN messages ON topics.id = messages.topic_id 
                    GROUP BY topics.id, name, type
                """)
                topics_info = self.db_cursor.fetchall()
                
                # Create topics_with_message_count list
                topics_with_message_count = []
                for topic_name, topic_type, msg_count in topics_info:
                    topics_with_message_count.append({
                        'topic_metadata': {
                            'name': topic_name,
                            'type': topic_type,
                            'serialization_format': 'cdr',
                            'offered_qos_profiles': ''
                        },
                        'message_count': msg_count
                    })
                
                # Create metadata dictionary
                metadata = {
                    'rosbag2_bagfile_information': {
                        'version': 4,
                        'storage_identifier': 'sqlite3',
                        'relative_file_paths': [os.path.basename(self.db_path)],
                        'duration': {
                            'nanoseconds': max_time - min_time if max_time and min_time else 0
                        },
                        'starting_time': {
                            'nanoseconds_since_epoch': min_time if min_time else 0
                        },
                        'message_count': message_count,
                        'topics_with_message_count': topics_with_message_count
                    }
                }
                
                # Write metadata.yaml file
                metadata_path = os.path.join(self.bag_dir, 'metadata.yaml')
                import yaml
                with open(metadata_path, 'w') as f:
                    yaml.dump(metadata, f)
                
                print(f"✓ Created metadata.yaml at: {metadata_path}")
                
            except Exception as e:
                print(f"Error creating metadata.yaml: {e}")
                traceback.print_exc()
        
        def _db_get_or_create_topic_id(self, topic_name: str, msg_type: str) -> int:
            """Get or create a topic ID (called from database thread)."""
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
        
        def _db_write_message(self, topic_id: int, timestamp: int, data: bytes):
            """Write a message to database (called from database thread)."""
            self.db_cursor.execute(
                "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                (topic_id, timestamp, data)
            )
            self.db_conn.commit()
        
        def _init_publishers(self):
            """Initialize ROS2 publishers."""
            # Common state publisher
            self.common_state_pub = self.create_publisher(
                String, f"{self.namespace}/common_state", 10
            )
            
            # Camera publishers (lazy initialization)
            self.rgb_publishers = {}
            self.segmentation_publishers = {}
            self.depth_publishers = {}
            self.normals_publishers = {}
            self.camera_info_publishers = {}
            
            print("✓ Publishers initialized")
        
        def _sanitize_topic_name(self, name: str) -> str:
            """Sanitize a name for use as a ROS2 topic name."""
            sanitized = name.replace('.', '_').replace('-', '_')
            sanitized = ''.join(c for c in sanitized if c.isalnum() or c in '_~{}')
            return sanitized
        
        def _get_publisher(self, publisher_dict: dict, camera_name: str, topic_suffix: str, msg_type):
            """Get or create a publisher for a camera."""
            sanitized_name = self._sanitize_topic_name(camera_name)
            if sanitized_name not in publisher_dict:
                topic_name = f"{self.namespace}/{topic_suffix}/{sanitized_name}"
                publisher_dict[sanitized_name] = self.create_publisher(msg_type, topic_name, 10)
            return publisher_dict[sanitized_name]
        
        def _create_header(self, frame_id: str = None):
            """Create a ROS2 header with current timestamp and unified frame_id.
            
            Args:
                frame_id: Specific frame_id to use. If None, uses namespace-based default.
            """
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            
            if frame_id is None:
                # Use namespace-based frame_id
                namespace_clean = self.namespace.lstrip('/')
                header.frame_id = f"{namespace_clean}/world"
            else:
                header.frame_id = frame_id
            
            return header
        
        def _publish_robot_transform(self, position, orientation, child_frame_id: str = None):
            """Publish robot pose as a transform using TF2.
            
            This method publishes the robot's current pose as a transform from the namespace-based world frame
            to the specified child frame. This allows other ROS2 nodes to easily access the robot's pose 
            in the world coordinate system.
            
            Args:
                position: Robot position as [x, y, z]
                orientation: Robot orientation as [x, y, z, w] quaternion
                child_frame_id: The child frame ID for the transform (default: uses namespace + "/forward_link")
            """
            if not TF2_AVAILABLE or self.tf_broadcaster is None:
                return
            
            try:
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                
                # Use namespace-based frame_id
                namespace_clean = self.namespace.lstrip('/')
                transform.header.frame_id = f"{namespace_clean}/world"
                
                # Use namespace for child_frame_id if not specified
                if child_frame_id is None:
                    transform.child_frame_id = f"{namespace_clean}/forward_link"
                else:
                    transform.child_frame_id = child_frame_id
                
                # Set translation
                transform.transform.translation.x = float(position[0])
                transform.transform.translation.y = float(position[1])
                transform.transform.translation.z = float(position[2])
                
                # Set rotation
                # isaac sim uses wxyz convention for quaternions
                transform.transform.rotation.x = float(orientation[1])
                transform.transform.rotation.y = float(orientation[2])
                transform.transform.rotation.z = float(orientation[3])
                transform.transform.rotation.w = float(orientation[0])
                
                # Publish the transform via TF2
                self.tf_broadcaster.sendTransform(transform)
                
                # Also record the transform in the bag if bag recording is enabled
                if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor') and TF2_MSG_AVAILABLE:
                    try:
                        # Create TFMessage for bag recording
                        tf_message = TFMessage()
                        tf_message.transforms = [transform]
                        
                        # Record to bag using the /tf topic
                        self._write_to_bag("/tf", tf_message, "tf2_msgs/msg/TFMessage")
                    except Exception as e:
                        print(f"Error recording transform to bag: {e}")
                        traceback.print_exc()
                
            except Exception as e:
                print(f"Error publishing robot transform: {e}")
                traceback.print_exc()
        
        def _get_or_create_topic_id(self, topic_name: str, msg_type: str) -> int:
            """Get or create a topic ID for bag recording (thread-safe with caching)."""
            # Check cache first
            cache_key = f"{topic_name}:{msg_type}"
            if cache_key in self.topic_cache:
                return self.topic_cache[cache_key]
            
            # Try to send request to database thread (non-blocking)
            try:
                self.db_queue.put_nowait(('get_or_create_topic_id', (topic_name, msg_type)))
                
                # Wait for result with timeout
                try:
                    result = self.response_queue.get(timeout=0.5)
                    if result[0] == 'result':
                        topic_id = result[1]
                        # Cache the result
                        self.topic_cache[cache_key] = topic_id
                        return topic_id
                except queue.Empty:
                    print(f"WARNING: Timeout waiting for topic ID for {topic_name}")
                    # Return a default topic ID to prevent blocking
                    return 1
                    
            except queue.Full:
                print(f"WARNING: ROS2 queue full, dropping data for topic {topic_name}")
                self.dropped_messages += 1
                # Return a default topic ID to prevent blocking
                return 1
        
        def _write_to_bag(self, topic_name: str, msg, msg_type: str):
            """Write a message to the bag file (thread-safe, non-blocking)."""
            try:
                topic_id = self._get_or_create_topic_id(topic_name, msg_type)
                serialized_data = serialize_message(msg)
                timestamp = int(time.time() * 1e9)  # Nanoseconds
                
                # Try to send write request to database thread (non-blocking)
                try:
                    self.db_queue.put_nowait(('write_message', (topic_id, timestamp, serialized_data)))
                    self.total_messages += 1
                except queue.Full:
                    print(f"WARNING: ROS2 queue full, dropping data for step")
                    self.dropped_messages += 1
                    
            except Exception as e:
                print(f"Error writing to bag: {e}")
                traceback.print_exc()
        
        def _publish_message(self, msg, topic_name: str, msg_type: str, publisher=None):
            """Publish a message to topic and/or bag."""
            if self.mode in ["Stream Only", "Stream + Bag"] and publisher:
                publisher.publish(msg)
            
            if self.mode in ["Bag Only", "Stream + Bag"] and hasattr(self, 'db_cursor'):
                self._write_to_bag(topic_name, msg, msg_type)
        
        def write_state_dict_common(self, state_dict: dict, step: int):
            """Write common state data to ROS2 topics and bag."""
            msg = String()
            serializable_state = _convert_numpy_to_json(state_dict)
            msg.data = json.dumps({
                'data': serializable_state,
                'step': step,
                'timestamp': time.time()
            })
            
            topic_name = f"{self.namespace}/common_state"
            self._publish_message(msg, topic_name, "std_msgs/msg/String", self.common_state_pub)
        
        def write_common_state_data(self, state_dict: dict, step: int, scenario=None):
            """Extract and publish all data from state to ROS2 topics."""
            try:
                if self.scenario is None:
                    self.scenario = scenario
                
                # Check if processing is enabled
                if not self.enable_processing:
                    return
                
                # Check if ROS2 is still responsive
                try:
                    if not rclpy.ok():
                        print(f"WARNING: ROS2 context is not ok, skipping step {step}")
                        return
                except Exception as e:
                    print(f"WARNING: Error checking ROS2 context: {e}")
                    return
                
                # Throttling: only process ROS2 data every few steps
                should_process_ros2 = (step - self.last_ros2_publish_step) >= self.ros2_publish_frequency
                if not should_process_ros2:
                    return
                
                # Extract data from state
                robot_data = {}
                camera_data = {}
                keyboard_data = {}
                
                for key, value in state_dict.items():
                    if key.startswith('robot.'):
                        if 'camera' in key and ('rgb_image' in key or 'depth_image' in key or 'segmentation_image' in key):
                            if self.publish_camera_images:
                                camera_name = key.split('.')[2]
                                image_type = key.split('.')[-1]
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name][image_type] = value
                        elif 'camera' in key:
                            # Handle camera metadata
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
                            elif 'intrinsics' in key:
                                # Handle camera intrinsics
                                camera_name = key.replace('.intrinsics', '')
                                if camera_name not in camera_data:
                                    camera_data[camera_name] = {}
                                camera_data[camera_name]['intrinsics'] = value
                        else:
                            robot_data[key] = value
                    elif key.startswith('keyboard.'):
                        keyboard_data[key] = value
                
                # Publish data
                self._publish_robot_data(robot_data, step)
                if camera_data:
                    self._publish_camera_data_with_images(camera_data, step)
                self._publish_keyboard_data(keyboard_data, step)
                
                self.last_ros2_publish_step = step
                      
            except Exception as e:
                print(f"ERROR: Exception in write_common_state_data for step {step}: {e}")
                traceback.print_exc()
        
        def _publish_robot_data(self, robot_data: dict, step: int):
            """Publish robot data to ROS2 topics."""
            try:
                # Publish robot pose
                if 'robot.position' in robot_data and 'robot.orientation' in robot_data:
                    pose_msg = PoseStamped()
                    pose_msg.header = self._create_header()  # Uses unified frame_id
                    
                    position = robot_data['robot.position']
                    orientation = robot_data['robot.orientation']
                    
                    pose_msg.pose.position.x = float(position[0])
                    pose_msg.pose.position.y = float(position[1])
                    pose_msg.pose.position.z = float(position[2])
                    # isaac sim uses wxyz convention for quaternions
                    pose_msg.pose.orientation.x = float(orientation[1])
                    pose_msg.pose.orientation.y = float(orientation[2])
                    pose_msg.pose.orientation.z = float(orientation[3])
                    pose_msg.pose.orientation.w = float(orientation[0])
                    
                    topic_name = f"{self.namespace}/robot/pose"
                    if not hasattr(self, 'robot_pose_publisher'):
                        self.robot_pose_publisher = self.create_publisher(PoseStamped, topic_name, 10)
                    self._publish_message(pose_msg, topic_name, "geometry_msgs/msg/PoseStamped", self.robot_pose_publisher)
                    
                    # Publish robot pose as transform
                    self._publish_robot_transform(position, orientation)
                
                # Publish robot twist (action)
                if 'robot.action' in robot_data:
                    twist_msg = TwistStamped()
                    # Use namespace-based base_link frame_id
                    namespace_clean = self.namespace.lstrip('/')
                    twist_msg.header = self._create_header(f"{namespace_clean}/base_link")
                    
                    action = robot_data['robot.action']
                    twist_msg.twist.linear.x = float(action[0])
                    twist_msg.twist.angular.z = float(action[1])
                    
                    topic_name = f"{self.namespace}/robot/twist"
                    if not hasattr(self, 'robot_twist_publisher'):
                        self.robot_twist_publisher = self.create_publisher(TwistStamped, topic_name, 10)
                    self._publish_message(twist_msg, topic_name, "geometry_msgs/msg/TwistStamped", self.robot_twist_publisher)
                
                # Publish joint data
                for joint_type in ['joint_positions', 'joint_velocities']:
                    if f'robot.{joint_type}' in robot_data:
                        joint_msg = Float64MultiArray()
                        joint_data = robot_data[f'robot.{joint_type}']
                        joint_msg.data = [float(x) for x in joint_data]
                        
                        topic_name = f"{self.namespace}/robot/{joint_type}"
                        publisher_attr = f'{joint_type}_publisher'
                        if not hasattr(self, publisher_attr):
                            setattr(self, publisher_attr, self.create_publisher(Float64MultiArray, topic_name, 10))
                        publisher = getattr(self, publisher_attr)
                        self._publish_message(joint_msg, topic_name, "std_msgs/msg/Float64MultiArray", publisher)
                
            except Exception as e:
                print(f"ERROR: Exception in _publish_robot_data for step {step}: {e}")
                traceback.print_exc()
        
        def _publish_keyboard_data(self, keyboard_data: dict, step: int):
            """Publish keyboard data to ROS2 topics."""
            try:
                if keyboard_data:
                    keyboard_data_json = _convert_numpy_to_json(keyboard_data)
                    keyboard_msg = String()
                    keyboard_msg.data = json.dumps(keyboard_data_json)
                    
                    topic_name = f"{self.namespace}/keyboard"
                    if not hasattr(self, 'keyboard_publisher'):
                        self.keyboard_publisher = self.create_publisher(String, topic_name, 10)
                    self._publish_message(keyboard_msg, topic_name, "std_msgs/msg/String", self.keyboard_publisher)
                
            except Exception as e:
                print(f"Error publishing keyboard data: {e}")
                traceback.print_exc()

        def _publish_camera_data_with_images(self, camera_data: dict, step: int):
            """Publish camera data with images to ROS2 topics.
            Camera pose in the Isaac Sim USD camera prim coordinate system, ie. forward -Z and Up +Y
            We need to transform the camera pose to the ROS2 coordinate system, ie. forward +Z and Up -Y
            https://docs.isaacsim.omniverse.nvidia.com/4.2.0/reference_conventions.html
            """
            try:
                # Always publish camera metadata (lightweight)
                for camera_name, camera_data_dict in camera_data.items():
                    if 'position' in camera_data_dict:
                        try:
                            position = camera_data_dict['position']
                            orientation = camera_data_dict.get('orientation')

                            # Validate position and orientation data
                            if position is None or orientation is None:
                                print(f"Warning: Missing position or orientation data for camera {camera_name}")
                                continue
                            
                            # Ensure orientation is a valid array
                            if not isinstance(orientation, (list, np.ndarray)) or len(orientation) != 4:
                                print(f"Warning: Invalid orientation data for camera {camera_name}: {orientation}")
                                continue

                            # Convert quaternion from WXYZ to XYZW format for scipy
                            orientation_xyzw = np.array([orientation[1], orientation[2], orientation[3], orientation[0]])
                            
                            try:
                                # Create rotation object from quaternion
                                R_camera_prim = Rotation.from_quat(orientation_xyzw)
                                
                                # The transformation needed:
                                # Camera Z (blue, -X) -> Robot X (red, +X) - need 180° around Y
                                # Camera Y (green, +Z) -> Robot -Z (down) - need 180° around X
                                # This requires both rotations: first around Y, then around X
                                T_rotation_y = Rotation.from_rotvec([0, np.pi, 0])  # 180 degrees around Y-axis
                                T_rotation_x = Rotation.from_rotvec([np.pi, 0, 0])  # 180 degrees around X-axis
                                
                                # Apply the transformations: R_ros2 = T_x * T_y * R_camera_prim * T_y^(-1) * T_x^(-1)
                                R_ros2 = T_rotation_x * T_rotation_y * R_camera_prim * T_rotation_y.inv() * T_rotation_x.inv()
                                
                                # Convert back to WXYZ format
                                orientationXYZW = R_ros2.as_quat()
                                orientationWXYZ = [orientationXYZW[3], orientationXYZW[0], orientationXYZW[1], orientationXYZW[2]]
                                
                                # Position remains unchanged (already in world frame)
                                position_transformed = position
                                
                                # Debug output (only print occasionally to avoid spam)
                                if step % 1000 == 0:
                                    print(f"Camera {camera_name} transformation:")
                                    print(f"  Position (world frame, unchanged): {position}")
                                    print(f"  Original orientation (camera prim convention): {orientation}")
                                    print(f"  Transformed orientation (ROS2 convention): {orientationWXYZ}")
                                    
                            except Exception as e:
                                print(f"Warning: Failed to transform camera orientation for {camera_name}: {e}")
                                # Use original orientation if transformation fails
                                orientationWXYZ = orientation
                                position_transformed = position

                            pose_msg = PoseStamped()
                            pose_msg.header = self._create_header()  # Uses unified frame_id
                            pose_msg.pose.position.x = float(position_transformed[0])
                            pose_msg.pose.position.y = float(position_transformed[1])
                            pose_msg.pose.position.z = float(position_transformed[2])
                            # isaac sim uses wxyz convention for quaternions
                            pose_msg.pose.orientation.x = float(orientationWXYZ[1])
                            pose_msg.pose.orientation.y = float(orientationWXYZ[2])
                            pose_msg.pose.orientation.z = float(orientationWXYZ[3])
                            pose_msg.pose.orientation.w = float(orientationWXYZ[0])
                            
                            sanitized_name = self._sanitize_topic_name(camera_name)
                            topic_name = f"{self.namespace}/camera/{sanitized_name}/pose"
                            if not hasattr(self, 'camera_pose_publishers'):
                                self.camera_pose_publishers = {}
                            if topic_name not in self.camera_pose_publishers:
                                self.camera_pose_publishers[topic_name] = self.create_publisher(
                                    PoseStamped, topic_name, 10
                                )
                            self._publish_message(pose_msg, topic_name, "geometry_msgs/msg/PoseStamped", 
                                                self.camera_pose_publishers[topic_name])

                            # Publish camera pose as transform
                            self._publish_robot_transform(position_transformed, orientationWXYZ, f"{self.namespace}/camera/{sanitized_name}_optical_frame")
                                    
                        except Exception as e:
                            print(f"Error publishing camera pose for {camera_name}: {e}")
                            traceback.print_exc()
                    
                    # Publish camera intrinsics (if available)
                    if 'intrinsics' in camera_data_dict:
                        try:
                            intrinsics = self._extract_camera_intrinsics(camera_name, camera_data_dict)
                            if intrinsics is not None:
                                # Get resolution from camera object
                                resolution = None
                                if hasattr(self, 'scenario') and self.scenario is not None:
                                    # Get the appropriate camera based on camera name
                                    if camera_name == 'robot.front_camera':
                                        # Single camera (RealSense, etc.)
                                        resolution = self.scenario.robot.front_camera.resolution
                                    elif camera_name == 'robot.front_camera.left':
                                        # Left camera from stereo setup (HawkCamera)
                                        resolution = self.scenario.robot.front_camera.left.resolution
                                    elif camera_name == 'robot.front_camera.right':
                                        # Right camera from stereo setup (HawkCamera)
                                        resolution = self.scenario.robot.front_camera.right.resolution
                                
                                # Final fallback to default
                                if resolution is None:
                                    print(f"WARNING: No resolution found for camera {camera_name}, using default")
                                    resolution = (848, 480)  # Default fallback
                                
                                camera_info_msg = self._create_camera_info_message(camera_name, intrinsics, resolution)
                                sanitized_name = self._sanitize_topic_name(camera_name)
                                topic_name = f"{self.namespace}/camera/{sanitized_name}/camera_info"
                                
                                if topic_name not in self.camera_info_publishers:
                                    self.camera_info_publishers[topic_name] = self.create_publisher(
                                        CameraInfo, topic_name, 10
                                    )
                                self._publish_message(camera_info_msg, topic_name, "sensor_msgs/msg/CameraInfo", 
                                                    self.camera_info_publishers[topic_name])
                                
                                if step % 1000 == 0:  # Log occasionally
                                    pass
                                
                        except Exception as e:
                            print(f"Error publishing camera intrinsics for {camera_name}: {e}")
                            traceback.print_exc()
                
                # Publish images at throttled frequency
                should_publish_images = (step - self.last_camera_publish_step) >= self.camera_publish_frequency
                if should_publish_images and self.publish_camera_images:
                    if not hasattr(self, 'cv_bridge'):
                        self.cv_bridge = CvBridge()
                    
                    # Initialize publishers if needed
                    if not hasattr(self, 'rgb_publishers'):
                        self.rgb_publishers = {}
                    if not hasattr(self, 'depth_publishers'):
                        self.depth_publishers = {}
                    if not hasattr(self, 'segmentation_publishers'):
                        self.segmentation_publishers = {}
                    
                    # Publish images for all cameras
                    for camera_name, camera_data_dict in camera_data.items():
                        sanitized_name = self._sanitize_topic_name(camera_name)
                        
                        # Publish RGB images
                        if 'rgb_image' in camera_data_dict:
                            rgb_image = camera_data_dict['rgb_image']
                            if rgb_image is not None and rgb_image.size > 0:
                                try:
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(rgb_image, "rgb8")
                                    # Use camera-specific frame_id
                                    namespace_clean = self.namespace.lstrip('/')
                                    ros_image.header = self._create_header(f"{namespace_clean}/{sanitized_name}_optical_frame")
                                    
                                    topic_name = f"{self.namespace}/rgb/{sanitized_name}"
                                    if topic_name not in self.rgb_publishers:
                                        self.rgb_publishers[topic_name] = self.create_publisher(Image, topic_name, 10)
                                    self._publish_message(ros_image, topic_name, "sensor_msgs/msg/Image", 
                                                        self.rgb_publishers[topic_name])
                                        
                                except Exception as e:
                                    print(f"Error publishing RGB image for {camera_name}: {e}")
                                    traceback.print_exc()
                        
                        # Publish depth images
                        if 'depth_image' in camera_data_dict:
                            depth_image = camera_data_dict['depth_image']
                            if depth_image is not None and depth_image.size > 0:
                                try:
                                    depth_uint16 = depth_image.astype(np.uint16)
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(depth_uint16, "mono16")
                                    # Use camera-specific frame_id
                                    namespace_clean = self.namespace.lstrip('/')
                                    ros_image.header = self._create_header(f"{namespace_clean}/{sanitized_name}_optical_frame")
                                    
                                    topic_name = f"{self.namespace}/depth/{sanitized_name}"
                                    if topic_name not in self.depth_publishers:
                                        self.depth_publishers[topic_name] = self.create_publisher(Image, topic_name, 10)
                                    self._publish_message(ros_image, topic_name, "sensor_msgs/msg/Image", 
                                                        self.depth_publishers[topic_name])
                                        
                                except Exception as e:
                                    print(f"Error publishing depth image for {camera_name}: {e}")
                                    traceback.print_exc()
                        
                        # Publish segmentation images
                        if 'segmentation_image' in camera_data_dict:
                            seg_image = camera_data_dict['segmentation_image']
                            if seg_image is not None and seg_image.size > 0:
                                try:
                                    if seg_image.dtype == np.uint32:
                                        seg_image = seg_image.astype(np.uint16)
                                        encoding = "16UC1"
                                    else:
                                        seg_image = seg_image.astype(np.uint8)
                                        encoding = "mono8"
                                    
                                    ros_image = self.cv_bridge.cv2_to_imgmsg(seg_image, encoding)
                                    # Use camera-specific frame_id
                                    namespace_clean = self.namespace.lstrip('/')
                                    ros_image.header = self._create_header(f"{namespace_clean}/{sanitized_name}_optical_frame")
                                    
                                    topic_name = f"{self.namespace}/segmentation/{sanitized_name}"
                                    if topic_name not in self.segmentation_publishers:
                                        self.segmentation_publishers[topic_name] = self.create_publisher(Image, topic_name, 10)
                                    self._publish_message(ros_image, topic_name, "sensor_msgs/msg/Image", 
                                                        self.segmentation_publishers[topic_name])
                                        
                                except Exception as e:
                                    print(f"Error publishing segmentation image for {camera_name}: {e}")
                                    traceback.print_exc()
                    
                    self.last_camera_publish_step = step
                
            except Exception as e:
                print(f"Error in _publish_camera_data_with_images: {e}")
                traceback.print_exc()
        
        def _create_camera_info_message(self, camera_name: str, intrinsics: tuple, resolution: tuple) -> CameraInfo:
            """
            Create a CameraInfo message from camera intrinsics.
            
            Args:
                camera_name (str): Name of the camera
                intrinsics (tuple): (fx, fy, cx, cy) camera intrinsics
                resolution (tuple): (width, height) camera resolution
                
            Returns:
                CameraInfo: ROS2 CameraInfo message
            """
            fx, fy, cx, cy = intrinsics
            width, height = resolution
            
            camera_info = CameraInfo()
            camera_info.header = self._create_header()
            
            # Use namespace-based camera frame_id
            namespace_clean = self.namespace.lstrip('/')
            sanitized_camera_name = self._sanitize_topic_name(camera_name)
            camera_info.header.frame_id = f"{namespace_clean}/{sanitized_camera_name}_optical_frame"
            
            # Set camera matrix (3x3)
            camera_info.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ]
            
            # Set distortion coefficients (assuming no distortion for now)
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Set projection matrix (3x4)
            camera_info.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Set image dimensions
            camera_info.width = width
            camera_info.height = height
            
            return camera_info
        
        def _extract_camera_intrinsics(self, camera_name: str, camera_data_dict: dict) -> tuple:
            """
            Extract camera intrinsics from camera data.
            
            Args:
                camera_name (str): Name of the camera
                camera_data_dict (dict): Camera data dictionary
                
            Returns:
                tuple: (fx, fy, cx, cy) camera intrinsics or None if not available
            """
            try:
                # Check if we have intrinsics in the camera data
                if 'intrinsics' in camera_data_dict:
                    intrinsics = camera_data_dict['intrinsics']
                    if intrinsics is not None and len(intrinsics) == 4:
                        return intrinsics
                    else:
                        print(f"Warning: Intrinsics for {camera_name} are None or invalid: {intrinsics}")
                
                # If not, try to get intrinsics from the camera object
                # This would require access to the actual camera object
                # For now, we'll return None and let the calling code handle it
                print(f"Warning: No intrinsics found for camera {camera_name}")
                return None
                
            except Exception as e:
                print(f"Error extracting intrinsics for camera {camera_name}: {e}")
                return None
        
        def get_queue_stats(self):
            """Get queue statistics for monitoring."""
            return {
                'queue_size': self.db_queue.qsize(),
                'max_queue_size': self.db_queue.maxsize,
                'dropped_messages': self.dropped_messages,
                'total_messages': self.total_messages,
                'drop_rate': self.dropped_messages / max(self.total_messages, 1) * 100
            }
        
        def destroy_node(self):
            """Clean up the node and database."""
            # Stop database thread
            if hasattr(self, 'db_running') and self.db_running:
                self.db_running = False
                if hasattr(self, 'db_queue'):
                    self.db_queue.put(('close', None))
                
                # Wait for database thread to finish
                if hasattr(self, 'db_thread') and self.db_thread:
                    self.db_thread.join(timeout=2.0)
                    if self.db_thread.is_alive():
                        print("Warning: Database thread did not shut down cleanly")
                
                # Print final statistics
                stats = self.get_queue_stats()
                print(f"Final queue stats: {stats['total_messages']} messages, {stats['dropped_messages']} dropped ({stats['drop_rate']:.1f}%)")
                
                # Create metadata.yaml if bag was created
                if hasattr(self, 'bag_dir') and os.path.exists(self.bag_dir):
                    print(f"✓ ROS2 bag created at: {self.bag_dir}")
                    print(f"  - Database: {self.db_path}")
                    print(f"  - Metadata: {os.path.join(self.bag_dir, 'metadata.yaml')}")
                    print(f"  - This bag can be converted to ROS1 using: rosbags-convert --src {self.bag_dir} --dst output.bag --src-typestore ros2_humble")
            
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
        """Clean up ROS2 and ROS1 bag resources."""
        # Clean up ROS2 writer
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
        # Write to ROS2
        if self.ros2_writer is not None:
            try:
                self.ros2_writer.write_state_dict_common(state_dict, step)
            except Exception as e:
                print(f"Error writing to ROS2: {e}")
                traceback.print_exc()
    
    def write_camera_data(self, rgb_data: dict, segmentation_data: dict, depth_data: dict, normals_data: dict, step: int):
        """Write camera data to ROS2 if enabled."""
        if self.ros2_writer is not None:
            try:
                # This method is kept for backward compatibility but the main camera data
                # is now handled in write_common_state_data
                pass
            except Exception as e:
                print(f"Error writing camera data to ROS2: {e}")
                traceback.print_exc() 