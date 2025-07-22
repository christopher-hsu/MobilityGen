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

This script launches a simulation app for replaying and rendering
a recording with ROS2 streaming capabilities.

"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import argparse
import os
import shutil
import numpy as np
from PIL import Image
import glob
import tqdm
import rclpy

import omni.replicator.core as rep

from omni.ext.mobility_gen.utils.global_utils import get_world
from omni.ext.mobility_gen.writer import Writer
from omni.ext.mobility_gen.hybrid_writer import HybridWriter, WriterMode
from omni.ext.mobility_gen.ros2_writer import ROS2Writer
from omni.ext.mobility_gen.reader import Reader
from omni.ext.mobility_gen.build import load_scenario


def create_ros2_writer(namespace: str = "/mobility_gen", enable_compression: bool = True) -> ROS2Writer:
    """Create and initialize a ROS2 writer node."""
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 writer node
    ros2_writer = ROS2Writer(
        node_name="mobility_gen_replay_ros2_writer",
        namespace=namespace,
        enable_compression=enable_compression
    )
    
    return ros2_writer


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--input_path", type=str, required=True)
    parser.add_argument("--output_path", type=str, default=None)
    parser.add_argument("--rgb_enabled", type=bool, default=True)
    parser.add_argument("--segmentation_enabled", type=bool, default=True)
    parser.add_argument("--depth_enabled", type=bool, default=True)
    parser.add_argument("--instance_id_segmentation_enabled", type=bool, default=True)
    parser.add_argument("--normals_enabled", type=bool, default=False)
    parser.add_argument("--render_rt_subframes", type=int, default=1)
    parser.add_argument("--render_interval", type=int, default=1)
    
    # ROS2 specific arguments
    parser.add_argument("--enable_ros2", action="store_true", 
                       help="Enable ROS2 streaming")
    parser.add_argument("--ros2_namespace", type=str, default="/mobility_gen",
                       help="ROS2 namespace for topics")
    parser.add_argument("--enable_compression", action="store_true",
                       help="Enable compressed image topics")
    parser.add_argument("--writer_mode", choices=['file', 'ros2', 'hybrid'], 
                       default='hybrid',
                       help="Writer mode: file, ros2, or hybrid")

    args, unknown = parser.parse_known_args()

    # Validate arguments
    if args.writer_mode == 'file' and args.output_path is None:
        parser.error("--output_path is required when writer_mode is 'file' or 'hybrid'")
    
    if args.writer_mode in ['ros2', 'hybrid'] and not args.enable_ros2:
        args.enable_ros2 = True

    scenario = load_scenario(os.path.join(args.input_path))

    world = get_world()
    world.reset()

    print(scenario)

    if args.rgb_enabled:
        scenario.enable_rgb_rendering()

    if args.segmentation_enabled:
        scenario.enable_segmentation_rendering()

    if args.depth_enabled:
        scenario.enable_depth_rendering()

    if args.instance_id_segmentation_enabled:
        scenario.enable_instance_id_segmentation_rendering()

    if args.normals_enabled:
        scenario.enable_normals_rendering()

    simulation_app.update()
    rep.orchestrator.step(
        rt_subframes=args.render_rt_subframes,
        delta_time=0.0,
        pause_timeline=False
    )

    reader = Reader(args.input_path)
    num_steps = len(reader)

    # Create writer based on mode
    writer = None
    ros2_writer = None
    
    mode_map = {
        'file': WriterMode.FILE,
        'ros2': WriterMode.ROS2,
        'hybrid': WriterMode.HYBRID
    }
    mode = mode_map[args.writer_mode]
    
    try:
        if mode in [WriterMode.ROS2, WriterMode.HYBRID]:
            print("Initializing ROS2 writer...")
            ros2_writer = create_ros2_writer(
                namespace=args.ros2_namespace,
                enable_compression=args.enable_compression
            )
            print("ROS2 writer initialized successfully")
        
        if mode == WriterMode.FILE:
            writer = Writer(args.output_path)
            writer.copy_init(args.input_path)
        elif mode == WriterMode.ROS2:
            writer = HybridWriter.create_ros2_writer(ros2_writer)
        elif mode == WriterMode.HYBRID:
            writer = HybridWriter.create_hybrid_writer(args.output_path, ros2_writer)
            writer.copy_init(args.input_path)
        
        print(f"Writer created successfully with mode: {args.writer_mode}")
        
    except Exception as e:
        print(f"Error creating writer: {e}")
        if ros2_writer is not None:
            ros2_writer.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
        exit(1)

    print(f"============== Replaying with ROS2 ==============")
    print(f"\tInput path: {args.input_path}")
    print(f"\tOutput path: {args.output_path}")
    print(f"\tWriter mode: {args.writer_mode}")
    print(f"\tROS2 enabled: {args.enable_ros2}")
    print(f"\tROS2 namespace: {args.ros2_namespace}")
    print(f"\tRgb enabled: {args.rgb_enabled}")
    print(f"\tSegmentation enabled: {args.segmentation_enabled}")
    print(f"\tRendering RT subframes: {args.render_rt_subframes}")
    print(f"\tRender interval: {args.render_interval}")

    try:
        for step in tqdm.tqdm(range(0, num_steps, args.render_interval)):
            
            state_dict = reader.read_state_dict(index=step)

            scenario.load_state_dict(state_dict)
            scenario.write_replay_data()

            simulation_app.update()

            rep.orchestrator.step(
                rt_subframes=args.render_rt_subframes,
                delta_time=0.00,
                pause_timeline=False
            )
            
            scenario.update_state()

            state_dict = scenario.state_dict_common()
            state_rgb = scenario.state_dict_rgb()
            state_segmentation = scenario.state_dict_segmentation()
            state_depth = scenario.state_dict_depth()
            state_normals = scenario.state_dict_normals()

            # Write data using the hybrid writer
            writer.write_state_dict_common(state_dict, step)
            writer.write_state_dict_rgb(state_rgb, step)
            writer.write_state_dict_segmentation(state_segmentation, step)
            writer.write_state_dict_depth(state_depth, step)
            writer.write_state_dict_normals(state_normals, step)

        print("Replay completed successfully")
        
    except KeyboardInterrupt:
        print("\nReplay interrupted by user")
    except Exception as e:
        print(f"Error during replay: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        print("Cleaning up...")
        if hasattr(writer, 'destroy'):
            writer.destroy()
        
        if ros2_writer is not None:
            ros2_writer.destroy_node()
        
        rclpy.shutdown()
        simulation_app.close()
        print("Cleanup completed") 