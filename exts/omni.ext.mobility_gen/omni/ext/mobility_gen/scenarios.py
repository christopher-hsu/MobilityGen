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


import numpy as np
from typing import Tuple
import json
import os
import time

from mobility_gen_path_planner import generate_paths

from omni.ext.mobility_gen.utils.path_utils import PathHelper, vector_angle
from omni.ext.mobility_gen.utils.registry import Registry
from omni.ext.mobility_gen.common import Module, Buffer
from omni.ext.mobility_gen.robots import Robot
from omni.ext.mobility_gen.occupancy_map import OccupancyMap

import omni.ext.mobility_gen.pose_samplers as pose_samplers
import omni.ext.mobility_gen.inputs as inputs


class Scenario(Module):

    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap
        ):
        self.robot = robot
        self.occupancy_map = occupancy_map
        self.buffered_occupancy_map = occupancy_map.buffered_meters(self.robot.occupancy_map_radius)

    @classmethod
    def from_robot_occupancy_map(cls, robot: Robot, occupancy_map: OccupancyMap):
        return cls(robot, occupancy_map)
    
    def reset(self):
        raise NotImplementedError
    
    def step(self, step_size: float) -> bool:
        raise NotImplementedError


SCENARIOS = Registry[Scenario]()


@SCENARIOS.register()
class KeyboardTeleoperationScenario(Scenario):

    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap
        ):
        super().__init__(robot, occupancy_map)
        self.keyboard = inputs.Keyboard()
        self.pose_sampler = pose_samplers.UniformPoseSampler()

    def reset(self):
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)

    def step(self, step_size):

        self.update_state()

        # Explicitly update keyboard state to ensure responsiveness
        self.keyboard.update_state()

        buttons = self.keyboard.buttons.get_value()



        # Handle case where buttons is None (keyboard not initialized)
        if buttons is None:
            buttons = [0.0, 0.0, 0.0, 0.0]  # Default to no keys pressed

        w_val = float(buttons[0])
        a_val = float(buttons[1])
        s_val = float(buttons[2])
        d_val = float(buttons[3])

        linear_velocity = (w_val - s_val) * self.robot.keyboard_linear_velocity_gain
        angular_velocity = (a_val - d_val) * self.robot.keyboard_angular_velocity_gain



        self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))

        self.robot.write_action(step_size)

        self.update_state()

        return True
    

@SCENARIOS.register()
class GamepadTeleoperationScenario(Scenario):

    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap
        ):
        super().__init__(robot, occupancy_map)
        self.gamepad = inputs.Gamepad()
        self.pose_sampler = pose_samplers.UniformPoseSampler()

    def reset(self):
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)

    def step(self, step_size: float):

        self.gamepad.update_state()

        axes = self.gamepad.axes.get_value()
        linear_velocity = axes[0] * self.robot.gamepad_linear_velocity_gain
        angular_velocity = axes[3] * self.robot.gamepad_angular_velocity_gain

        self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))
        self.robot.write_action(step_size)

        self.update_state()

        return True
    

@SCENARIOS.register()
class RandomAccelerationScenario(Scenario):

    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap
        ):
        super().__init__(robot, occupancy_map)
        self.pose_sampler = pose_samplers.GridPoseSampler(robot.random_action_grid_pose_sampler_grid_size)
        self.is_alive = True
        self.collision_occupancy_map = occupancy_map.buffered(robot.occupancy_map_collision_radius)

    def reset(self):
        self.robot.action.set_value(np.zeros(2))
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)
        self.is_alive = True

    def step(self, step_size: float):

        self.update_state()

        current_action = self.robot.action.get_value()

        linear_velocity = current_action[0] + step_size * np.random.randn(1) * self.robot.random_action_linear_acceleration_std
        angular_velocity = current_action[1] + step_size * np.random.randn(1) * self.robot.random_action_angular_acceleration_std
        
        linear_velocity = np.clip(linear_velocity, *self.robot.random_action_linear_velocity_range)[0]
        angular_velocity = np.clip(angular_velocity, *self.robot.random_action_angular_velocity_range)[0]

        self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))
        self.robot.write_action(step_size)

        self.update_state()

        # Check out of bounds or collision
        pose = self.robot.get_pose_2d()
        if not self.collision_occupancy_map.check_world_point_in_bounds(pose):
            self.is_alive = False
        elif not self.collision_occupancy_map.check_world_point_in_freespace(pose):
            self.is_alive = False

        return self.is_alive



@SCENARIOS.register()
class RandomPathFollowingScenario(Scenario):
    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap
        ):
        super().__init__(robot, occupancy_map)
        self.pose_sampler = pose_samplers.UniformPoseSampler()
        self.is_alive = True
        self.target_path = Buffer()
        self.collision_occupancy_map = occupancy_map.buffered(robot.occupancy_map_collision_radius)

    def set_random_target_path(self):
        current_pose = self.robot.get_pose_2d()

        start_px = self.occupancy_map.world_to_pixel_numpy(np.array([
            [current_pose.x, current_pose.y]
        ]))
        freespace = self.buffered_occupancy_map.freespace_mask()

        start = (start_px[0, 1], start_px[0, 0])

        output = generate_paths(start, freespace)
        end = output.sample_random_end_point()
        path = output.unroll_path(end)
        path = path[:, ::-1] # y,x -> x,y coordinates
        path = self.occupancy_map.pixel_to_world_numpy(path)
        self.target_path.set_value(path)
        self.target_path_helper = PathHelper(path)

    def reset(self):
        self.robot.action.set_value(np.zeros(2))
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)
        self.set_random_target_path()
        self.is_alive = True
    
    def step(self, step_size: float):

        self.update_state()
        target_path = self.target_path.get_value()
        current_pose = self.robot.get_pose_2d()

        if not self.collision_occupancy_map.check_world_point_in_bounds(current_pose):
            self.is_alive = False
            return self.is_alive
        elif not self.collision_occupancy_map.check_world_point_in_freespace(current_pose):
            self.is_alive = False
            return self.is_alive
    
        pt_robot = np.array([current_pose.x, current_pose.y])
        pt_path, pt_path_length, _, _ = self.target_path_helper.find_nearest(pt_robot)
        pt_target = self.target_path_helper.get_point_by_distance(distance=
            pt_path_length + self.robot.path_following_target_point_offset_meters
        )

        path_end = target_path[-1]
        dist_to_target = np.sqrt(np.sum((pt_robot - path_end)**2))

        if dist_to_target < self.robot.path_following_stop_distance_threshold:
            self.set_random_target_path()
        else:
            vec_robot_unit = np.array([np.cos(current_pose.theta), np.sin(current_pose.theta)])
            vec_target = (pt_target - pt_robot)
            vec_target_unit = vec_target / np.sqrt(np.sum(vec_target**2))
            d_theta = vector_angle(vec_robot_unit, vec_target_unit)

            if abs(d_theta) > self.robot.path_following_forward_angle_threshold:
                linear_velocity = 0.
            else:
                linear_velocity = self.robot.path_following_speed

            angular_gain: float = self.robot.path_following_angular_gain
            angular_velocity = - angular_gain * d_theta
            self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))

        self.robot.write_action(step_size=step_size)

        return self.is_alive


@SCENARIOS.register()
class FilePathFollowingScenario(Scenario):
    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap,
            waypoint_file: str = None
        ):
        super().__init__(robot, occupancy_map)
        self.pose_sampler = pose_samplers.UniformPoseSampler()
        self.is_alive = True
        self.target_path = Buffer()
        self.collision_occupancy_map = occupancy_map.buffered(robot.occupancy_map_collision_radius)
        
        # Waypoint file handling
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 0.5  # meters
        
        # Load waypoints if file is provided
        if self.waypoint_file and os.path.exists(self.waypoint_file):
            self.load_waypoints_from_file()

    def load_waypoints_from_file(self):
        """Load waypoints from a JSON file."""
        try:
            with open(self.waypoint_file, 'r') as f:
                data = json.load(f)
            
            # Handle different JSON formats
            if isinstance(data, dict):
                if 'waypoints' in data:
                    # Format: {"waypoints": [{"x": 1.0, "y": 2.0, "z": 0.0, ...}, ...]}
                    self.waypoints = data['waypoints']
                elif 'object_waypoints' in data:
                    # Format: {"object_waypoints": [{"waypoint": {"x": 1.0, "y": 2.0, ...}, "object_name": "chair"}, ...]}
                    self.waypoints = [wp['waypoint'] for wp in data['object_waypoints']]
                else:
                    # Single waypoint format: {"x": 1.0, "y": 2.0, "z": 0.0, ...}
                    self.waypoints = [data]
            elif isinstance(data, list):
                # Format: [{"x": 1.0, "y": 2.0, "z": 0.0, ...}, ...]
                self.waypoints = data
            else:
                print(f"Warning: Unknown waypoint file format in {self.waypoint_file}")
                return
            
            print(f"Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}")
            
        except Exception as e:
            print(f"Error loading waypoints from {self.waypoint_file}: {e}")
            self.waypoints = []

    def set_target_path_to_waypoint(self, waypoint):
        """Generate a path to the specified waypoint."""
        current_pose = self.robot.get_pose_2d()
        
        # Convert waypoint to world coordinates if needed
        if isinstance(waypoint, dict):
            target_x = waypoint.get('x', 0.0)
            target_y = waypoint.get('y', 0.0)
        else:
            target_x, target_y = waypoint[0], waypoint[1]
        
        # Convert current position to pixel coordinates
        start_px = self.occupancy_map.world_to_pixel_numpy(np.array([
            [current_pose.x, current_pose.y]
        ]))
        
        # Convert target to pixel coordinates
        target_px = self.occupancy_map.world_to_pixel_numpy(np.array([
            [target_x, target_y]
        ]))
        
        freespace = self.buffered_occupancy_map.freespace_mask()
        
        start = (int(start_px[0, 1]), int(start_px[0, 0]))
        end = (int(target_px[0, 1]), int(target_px[0, 0]))
        
        # Check if start and end are in freespace
        if (start[0] < 0 or start[0] >= freespace.shape[0] or 
            start[1] < 0 or start[1] >= freespace.shape[1] or
            end[0] < 0 or end[0] >= freespace.shape[0] or 
            end[1] < 0 or end[1] >= freespace.shape[1]):
            print(f"Warning: Start or end point outside freespace bounds")
            return False
        
        if not freespace[start[0], start[1]] or not freespace[end[0], end[1]]:
            print(f"Warning: Start or end point not in freespace")
            return False
        
        try:
            output = generate_paths(start, freespace)
            path = output.unroll_path(end)
            path = path[:, ::-1]  # y,x -> x,y coordinates
            path = self.occupancy_map.pixel_to_world_numpy(path)
            self.target_path.set_value(path)
            self.target_path_helper = PathHelper(path)
            return True
        except Exception as e:
            print(f"Error generating path to waypoint: {e}")
            return False

    def reset(self):
        self.robot.action.set_value(np.zeros(2))
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)
        
        # Reset waypoint index and set first waypoint
        self.current_waypoint_index = 0
        if self.waypoints:
            self.set_target_path_to_waypoint(self.waypoints[0])
        
        self.is_alive = True
    
    def step(self, step_size: float):
        super().update_state()
        current_pose = self.robot.get_pose_2d()

        if not self.collision_occupancy_map.check_world_point_in_bounds(current_pose):
            self.is_alive = False
            return self.is_alive
        elif not self.collision_occupancy_map.check_world_point_in_freespace(current_pose):
            self.is_alive = False
            return self.is_alive
        
        # Check if we have waypoints to follow
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            # No more waypoints, stop
            self.robot.action.set_value(np.zeros(2))
            self.robot.write_action(step_size)
            return self.is_alive
        
        target_path = self.target_path.get_value()
        if target_path is None or len(target_path) == 0:
            # No valid path, try next waypoint
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.set_target_path_to_waypoint(self.waypoints[self.current_waypoint_index])
            return self.is_alive
        
        pt_robot = np.array([current_pose.x, current_pose.y])
        pt_path, pt_path_length, _, _ = self.target_path_helper.find_nearest(pt_robot)
        pt_target = self.target_path_helper.get_point_by_distance(distance=
            pt_path_length + self.robot.path_following_target_point_offset_meters
        )

        path_end = target_path[-1]
        dist_to_target = np.sqrt(np.sum((pt_robot - path_end)**2))

        if dist_to_target < self.robot.path_following_stop_distance_threshold:
            # Reached current waypoint, move to next
            print(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                # Set path to next waypoint
                success = self.set_target_path_to_waypoint(self.waypoints[self.current_waypoint_index])
                if not success:
                    # Skip this waypoint if path generation fails
                    self.current_waypoint_index += 1
            else:
                # All waypoints completed
                print("All waypoints completed!")
                self.robot.action.set_value(np.zeros(2))
        else:
            # Follow current path
            vec_robot_unit = np.array([np.cos(current_pose.theta), np.sin(current_pose.theta)])
            vec_target = (pt_target - pt_robot)
            vec_target_unit = vec_target / np.sqrt(np.sum(vec_target**2))
            d_theta = vector_angle(vec_robot_unit, vec_target_unit)

            if abs(d_theta) > self.robot.path_following_forward_angle_threshold:
                linear_velocity = 0.
            else:
                linear_velocity = self.robot.path_following_speed

            angular_gain: float = self.robot.path_following_angular_gain
            angular_velocity = - angular_gain * d_theta
            self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))

        self.robot.write_action(step_size=step_size)
        return self.is_alive


@SCENARIOS.register()
class ROS2WaypointScenario(Scenario):
    def __init__(self, 
            robot: Robot, 
            occupancy_map: OccupancyMap,
            ros2_namespace: str = "/mobility_gen"
        ):
        super().__init__(robot, occupancy_map)
        self.pose_sampler = pose_samplers.UniformPoseSampler()
        self.is_alive = True
        self.target_path = Buffer()
        self.collision_occupancy_map = occupancy_map.buffered(robot.occupancy_map_collision_radius)
        
        # ROS2 waypoint handling
        self.ros2_namespace = ros2_namespace
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 0.5  # meters
        self.last_waypoint_update_time = 0.0
        self.waypoint_update_interval = 1.0  # seconds
        
        # Initialize ROS2 waypoint subscriber
        self.init_ros2_waypoint_subscriber()

    def init_ros2_waypoint_subscriber(self):
        """Initialize ROS2 waypoint subscriber."""
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from geometry_msgs.msg import PoseStamped
            
            # Create a simple ROS2 node for waypoint subscription
            if not rclpy.ok():
                rclpy.init()
            
            self.ros2_node = Node(f"mobility_gen_waypoint_subscriber_{int(time.time())}")
            
            # Subscribe to waypoint topics
            self.waypoint_sub = self.ros2_node.create_subscription(
                String, f"{self.ros2_namespace}/waypoints", self.waypoint_callback, 10
            )
            
            self.single_waypoint_sub = self.ros2_node.create_subscription(
                PoseStamped, f"{self.ros2_namespace}/waypoint", self.single_waypoint_callback, 10
            )
            
            print(f"✓ ROS2 waypoint subscriber initialized on namespace: {self.ros2_namespace}")
            
        except Exception as e:
            print(f"Error initializing ROS2 waypoint subscriber: {e}")
            self.ros2_node = None

    def waypoint_callback(self, msg):
        """Callback for waypoint list messages."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and 'waypoints' in data:
                self.waypoints = data['waypoints']
                self.current_waypoint_index = 0
                print(f"Received {len(self.waypoints)} waypoints via ROS2")
                for i, wp in enumerate(self.waypoints):
                    print(f"  Waypoint {i+1}: ({wp.get('x', 0):.2f}, {wp.get('y', 0):.2f}, {wp.get('z', 0):.2f})")
                
                # Get current robot position for comparison
                current_pose = self.robot.get_pose_2d()
                
                # Try to set path to first waypoint immediately
                if self.waypoints:
                    success = self.set_target_path_to_waypoint(self.waypoints[0])
                    print(f"DEBUG - Path setting result: {success}")
            elif isinstance(data, list):
                self.waypoints = data
                self.current_waypoint_index = 0
                print(f"Received {len(self.waypoints)} waypoints via ROS2")
        except Exception as e:
            print(f"Error processing waypoint message: {e}")

    def single_waypoint_callback(self, msg):
        """Callback for single waypoint messages."""
        try:
            waypoint = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'qx': msg.pose.orientation.x,
                'qy': msg.pose.orientation.y,
                'qz': msg.pose.orientation.z,
                'qw': msg.pose.orientation.w
            }
            
            # Add to waypoints list or replace current
            if self.waypoints:
                self.waypoints.append(waypoint)
            else:
                self.waypoints = [waypoint]
                self.current_waypoint_index = 0
            
            print(f"Received single waypoint via ROS2: ({waypoint['x']:.2f}, {waypoint['y']:.2f})")
        except Exception as e:
            print(f"Error processing single waypoint message: {e}")

    def adjust_waypoint_to_freespace(self, waypoint):
        """Adjust a waypoint to be in freespace by searching nearby positions."""
        from omni.ext.mobility_gen.occupancy_map import Point2d
        
        # Convert waypoint to coordinates
        if isinstance(waypoint, dict):
            target_x = waypoint.get('x', 0.0)
            target_y = waypoint.get('y', 0.0)
        else:
            target_x, target_y = waypoint[0], waypoint[1]
        
        original_x, original_y = target_x, target_y
        
        # Check if original position is in freespace
        point = Point2d(x=original_x, y=original_y)
        if self.buffered_occupancy_map.check_world_point_in_freespace(point):
            return waypoint
        
        # Original waypoint is not in freespace, searching for nearby valid position
        print(f"Original waypoint ({original_x:.2f}, {original_y:.2f}) is not in freespace. Searching for nearby valid position...")
        
        # Search in a spiral pattern around the original position
        search_radius = 2.0  # meters
        search_steps = 8  # number of directions to try
        
        for radius in np.linspace(0.5, search_radius, 5):
            for angle in np.linspace(0, 2*np.pi, search_steps, endpoint=False):
                test_x = original_x + radius * np.cos(angle)
                test_y = original_y + radius * np.sin(angle)
                
                test_point = Point2d(x=test_x, y=test_y)
                if self.buffered_occupancy_map.check_world_point_in_freespace(test_point):
                    print(f"Adjusted waypoint from ({original_x:.2f}, {original_y:.2f}) to ({test_x:.2f}, {test_y:.2f})")
                    
                    # Update the waypoint
                    if isinstance(waypoint, dict):
                        waypoint['x'] = test_x
                        waypoint['y'] = test_y
                    else:
                        waypoint[0] = test_x
                        waypoint[1] = test_y
                    
                    return waypoint
        
        # If no freespace found, try to move towards the center of the map
        print(f"No freespace found near original waypoint ({original_x:.2f}, {original_y:.2f}). Suggesting new waypoint towards map center...")
        
        center_x, center_y = 0.0, 0.0  # Assuming map center
        direction_x = center_x - original_x
        direction_y = center_y - original_y
        distance = np.sqrt(direction_x**2 + direction_y**2)
        
        if distance > 0:
            # Normalize and scale
            direction_x /= distance
            direction_y /= distance
            
            # Move towards center by 1 meter
            adjusted_x = original_x + direction_x * 1.0
            adjusted_y = original_y + direction_y * 1.0
            
            print(f"Could not find freespace, moving towards center: ({original_x:.2f}, {original_y:.2f}) -> ({adjusted_x:.2f}, {adjusted_y:.2f})")
            
            # Update the waypoint
            if isinstance(waypoint, dict):
                waypoint['x'] = adjusted_x
                waypoint['y'] = adjusted_y
            else:
                waypoint[0] = adjusted_x
                waypoint[1] = adjusted_y
        
        return waypoint

    def set_target_path_to_waypoint(self, waypoint):
        """Generate a path to the specified waypoint."""
        # First adjust the waypoint to be in freespace
        adjusted_waypoint = self.adjust_waypoint_to_freespace(waypoint)
        print(f"Attempting to plan path to waypoint: {adjusted_waypoint}")
        
        current_pose = self.robot.get_pose_2d()
        
        # Convert waypoint to world coordinates if needed
        if isinstance(adjusted_waypoint, dict):
            target_x = adjusted_waypoint.get('x', 0.0)
            target_y = adjusted_waypoint.get('y', 0.0)
        else:
            target_x, target_y = adjusted_waypoint[0], adjusted_waypoint[1]
        
        # Convert current position to pixel coordinates
        start_px = self.occupancy_map.world_to_pixel_numpy(np.array([
            [current_pose.x, current_pose.y]
        ]))
        
        # Convert target to pixel coordinates
        target_px = self.occupancy_map.world_to_pixel_numpy(np.array([
            [target_x, target_y]
        ]))
        
        freespace = self.buffered_occupancy_map.freespace_mask()
        
        start = (int(start_px[0, 1]), int(start_px[0, 0]))
        end = (int(target_px[0, 1]), int(target_px[0, 0]))
        
        # Check if start and end are in freespace
        # print(f"DEBUG - Path planning: start=({start[0]}, {start[1]}), end=({end[0]}, {end[1]})")
        # print(f"DEBUG - Freespace bounds: {freespace.shape}")
        
        if (start[0] < 0 or start[0] >= freespace.shape[0] or 
            start[1] < 0 or start[1] >= freespace.shape[1] or
            end[0] < 0 or end[0] >= freespace.shape[0] or 
            end[1] < 0 or end[1] >= freespace.shape[1]):
            print(f"Warning: Start or end point outside freespace bounds")
            print(f"DEBUG - Start bounds check: {start[0]} < 0 or {start[0]} >= {freespace.shape[0]} or {start[1]} < 0 or {start[1]} >= {freespace.shape[1]}")
            print(f"DEBUG - End bounds check: {end[0]} < 0 or {end[0]} >= {freespace.shape[0]} or {end[1]} < 0 or {end[1]} >= {freespace.shape[1]}")
            return False
        
        if not freespace[start[0], start[1]] or not freespace[end[0], end[1]]:
            print(f"Warning: Start or end point not in freespace")
            print(f"DEBUG - Start freespace check: {freespace[start[0], start[1]]}")
            print(f"DEBUG - End freespace check: {freespace[end[0], end[1]]}")
            return False
        
        try:
            output = generate_paths(start, freespace)
            path = output.unroll_path(end)
            path = path[:, ::-1]  # y,x -> x,y coordinates
            path = self.occupancy_map.pixel_to_world_numpy(path)
            self.target_path.set_value(path)
            self.target_path_helper = PathHelper(path)
            print(f"Path planning successful, path length: {len(path)}")
            return True
        except Exception as e:
            print(f"Error generating path to waypoint: {e}")
            return False

    def reset(self):
        self.robot.action.set_value(np.zeros(2))
        pose = self.pose_sampler.sample(self.buffered_occupancy_map)
        self.robot.set_pose_2d(pose)
        
        # Reset waypoint index and set first waypoint
        self.current_waypoint_index = 0
        if self.waypoints:
            self.set_target_path_to_waypoint(self.waypoints[0])
        
        self.is_alive = True
    
    def step(self, step_size: float):
        # Process ROS2 messages
        if self.ros2_node:
            try:
                import rclpy
                rclpy.spin_once(self.ros2_node, timeout_sec=0.001)
            except Exception as e:
                print(f"Error spinning ROS2 node: {e}")
        
        super().update_state()
        current_pose = self.robot.get_pose_2d()
        
        # Debug: Print robot's current position (less frequent)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
        
        if self._debug_counter % 100 == 0:  # Print every 100 steps
            print(f"Robot current position: ({current_pose.x:.2f}, {current_pose.y:.2f}, {current_pose.theta:.2f})")

        if not self.collision_occupancy_map.check_world_point_in_bounds(current_pose):
            self.is_alive = False
            return self.is_alive
        elif not self.collision_occupancy_map.check_world_point_in_freespace(current_pose):
            self.is_alive = False
            return self.is_alive
        
        # Check if we have waypoints to follow
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            # No more waypoints, stop
            self.robot.action.set_value(np.zeros(2))
            self.robot.write_action(step_size)
            return self.is_alive
        
        target_path = self.target_path.get_value()
        if target_path is None or len(target_path) == 0:
            # No valid path, try next waypoint
            print(f"DEBUG - No valid path available, robot not moving")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.set_target_path_to_waypoint(self.waypoints[self.current_waypoint_index])
            return self.is_alive
        
        pt_robot = np.array([current_pose.x, current_pose.y])
        pt_path, pt_path_length, _, _ = self.target_path_helper.find_nearest(pt_robot)
        
        # Check if path helper found a valid path
        if pt_path_length is None:
            print("DEBUG - No valid path found (pt_path_length is None), attempting to adjust waypoint to freespace...")
            # Try to adjust the current waypoint to a nearby freespace position
            current_goal = self.waypoints[self.current_waypoint_index]
            adjusted_goal = self.adjust_waypoint_to_freespace(current_goal)
            
            # Update the waypoint with the adjusted position
            self.waypoints[self.current_waypoint_index] = adjusted_goal
            print(f"DEBUG - Adjusted waypoint from {current_goal} to {adjusted_goal}")
            
            # Try to set path to the adjusted waypoint
            success = self.set_target_path_to_waypoint(adjusted_goal)
            if success:
                print("DEBUG - Path planning successful with adjusted waypoint, continuing...")
                # Try again with the new path
                target_path = self.target_path.get_value()
                pt_path, pt_path_length, _, _ = self.target_path_helper.find_nearest(pt_robot)
                if pt_path_length is None:
                    print("DEBUG - Still no valid path after adjustment, skipping this waypoint.")
                    self.current_waypoint_index += 1
                    return self.is_alive
            else:
                print("DEBUG - Path planning failed even with adjusted waypoint, skipping this waypoint.")
                self.current_waypoint_index += 1
                return self.is_alive
            
        pt_target = self.target_path_helper.get_point_by_distance(
            distance=pt_path_length + self.robot.path_following_target_point_offset_meters
        )

        path_end = target_path[-1]
        dist_to_target = np.sqrt(np.sum((pt_robot - path_end)**2))

        if dist_to_target < self.robot.path_following_stop_distance_threshold:
            # Reached current waypoint, move to next
            print(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} at {self.waypoints[self.current_waypoint_index]} with distance {dist_to_target:.2f}m")
            print("pt_target", pt_target, "pt_robot", pt_robot, "path_end", path_end)
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                # Set path to next waypoint
                success = self.set_target_path_to_waypoint(self.waypoints[self.current_waypoint_index])
                if not success:
                    # Skip this waypoint if path generation fails
                    self.current_waypoint_index += 1
            else:
                # All waypoints completed
                print("All waypoints completed!")
                self.robot.action.set_value(np.zeros(2))
        else:
            # Follow current path
            vec_robot_unit = np.array([np.cos(current_pose.theta), np.sin(current_pose.theta)])
            vec_target = (pt_target - pt_robot)
            vec_target_unit = vec_target / np.sqrt(np.sum(vec_target**2))
            d_theta = vector_angle(vec_robot_unit, vec_target_unit)

            if abs(d_theta) > self.robot.path_following_forward_angle_threshold:
                linear_velocity = 0.
            else:
                linear_velocity = self.robot.path_following_speed

            angular_gain: float = self.robot.path_following_angular_gain
            angular_velocity = - angular_gain * d_theta
            # print(f"DEBUG - Setting robot velocity: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}")
            self.robot.action.set_value(np.array([linear_velocity, angular_velocity]))

        self.robot.write_action(step_size=step_size)
        return self.is_alive

