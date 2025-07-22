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


import asyncio
import datetime
import glob
import os
import tempfile
import threading
import queue
import time
from typing import Optional

import omni.ext
import omni.ui as ui
import numpy as np

from omni.ext.mobility_gen.build import build_scenario_from_config
from omni.ext.mobility_gen.config import Config
from omni.ext.mobility_gen.robots import ROBOTS
from omni.ext.mobility_gen.scenarios import SCENARIOS
from omni.ext.mobility_gen.ros2_manager import ROS2Manager
from omni.ext.mobility_gen.utils.global_utils import get_world, save_stage
from omni.ext.mobility_gen.writer import Writer
from omni.ext.mobility_gen.inputs import GamepadDriver, KeyboardDriver


if "MOBILITY_GEN_DATA" in os.environ:
    DATA_DIR = os.environ['MOBILITY_GEN_DATA']
else:
    DATA_DIR = os.path.expanduser("~/MobilityGenData")

RECORDINGS_DIR = os.path.join(DATA_DIR, "recordings")
SCENARIOS_DIR = os.path.join(DATA_DIR, "scenarios")


class MobilityGenExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("MobilityGen Extension Startup")

        # Initialize ROS2 manager
        self.ros2_manager = ROS2Manager()
        
        # ROS2 threading
        self.ros2_thread = None
        self.ros2_queue = queue.Queue(maxsize=100)  # Limit queue size to prevent memory issues
        self.ros2_thread_running = False
        
        # Recording state
        self.recording_enabled = False
        self.writer = None
        self.step = 0
        self.recording_time = 0.
        
        # ROS2 streaming state
        self.ros2_streaming_enabled = False
        self.ros2_streaming_step = 0
        self.ros2_streaming_time = 0.0
        
        # Input drivers
        self.keyboard_driver = KeyboardDriver.connect()
        self.gamepad_driver = GamepadDriver.connect()
        
        # Scenario state
        self.scenario = None
        self.config = None
        self.cached_stage_path = None
        
        # Data directories
        self.recording_dir = os.path.expanduser("~/MobilityGenData/recordings")
        os.makedirs(self.recording_dir, exist_ok=True)
        
        # UI components
        self._occupancy_map_image_provider = omni.ui.ByteImageProvider()
        
        # UI setup
        self._setup_ui()
        
        # Update recording count
        self.update_recording_count()

    def _setup_ui(self):
        self._visualize_window = omni.ui.Window("MobilityGen - Occupancy Map", width=300, height=300)
        with self._visualize_window.frame:
            self._occ_map_frame = ui.Frame()
            self._occ_map_frame.set_build_fn(self.build_occ_map_frame)
            

        self._teleop_window = omni.ui.Window("MobilityGen", width=300, height=300)

        with self._teleop_window.frame:
            with ui.VStack():
                with ui.VStack():
                    with ui.HStack():
                        ui.Label("USD Path / URL")
                        self.scene_usd_field_string_model = ui.SimpleStringModel()
                        self.scene_usd_field = ui.StringField(model=self.scene_usd_field_string_model, height=25)

                    with ui.HStack():
                        ui.Label("Scenario Type")
                        self.scenario_combo_box = ui.ComboBox(0, *SCENARIOS.names())

                    with ui.HStack():
                        ui.Label("Robot Type")
                        self.robot_combo_box = ui.ComboBox(0, *ROBOTS.names())
                
                    ui.Button("Build", clicked_fn=self.build_scenario)

                with ui.VStack():
                    self.recording_count_label = ui.Label("")
                    self.recording_dir_label = ui.Label(f"Output directory: {RECORDINGS_DIR}")
                    self.recording_name_label = ui.Label("")
                    self.recording_step_label = ui.Label("")

                    ui.Button("Reset", clicked_fn=self.reset)
                    with ui.HStack():
                        ui.Button("Start Recording", clicked_fn=self.enable_recording)
                        ui.Button("Stop Recording", clicked_fn=self.disable_recording)

                # Separate ROS2 Streaming Section
                if self.ros2_manager.is_available():
                    with ui.VStack():
                        ui.Label("ROS2 Streaming")
                        with ui.HStack():
                            ui.Label("Enable ROS2")
                            self.ros2_streaming_enabled_checkbox = ui.CheckBox()
                            self.ros2_streaming_enabled_checkbox.model.set_value(False)
                        
                        with ui.HStack():
                            ui.Label("Mode")
                            self.ros2_streaming_mode_combo = ui.ComboBox(0, "Stream Only", "Bag Only", "Stream + Bag")
                        
                        with ui.HStack():
                            ui.Label("Namespace")
                            self.ros2_streaming_namespace_field = ui.StringField(height=20)
                            self.ros2_streaming_namespace_field.model.set_value("/mobility_gen")
                        
                        with ui.HStack():
                            ui.Label("Publish Images")
                            self.ros2_streaming_camera_images_checkbox = ui.CheckBox()
                            self.ros2_streaming_camera_images_checkbox.model.set_value(True)
                        
                        with ui.HStack():
                            ui.Button("Start ROS2 Streaming", clicked_fn=self.start_ros2_streaming)
                            ui.Button("Stop ROS2 Streaming", clicked_fn=self.stop_ros2_streaming)
                        
                        # ROS2 streaming status
                        self.ros2_streaming_status_label = ui.Label("ROS2 Streaming: Disabled")

    def build_occ_map_frame(self):
        if self.scenario is not None:
            with ui.VStack():
                image_widget = ui.ImageWithProvider(
                    self._occupancy_map_image_provider
                )

    def draw_occ_map(self):
        if self.scenario is not None and hasattr(self, '_occupancy_map_image_provider'):
            image = self.scenario.occupancy_map.ros_image().copy().convert("RGBA")
            data = list(image.tobytes())
            self._occupancy_map_image_provider.set_bytes_data(data, [image.width, image.height])
            self._occ_map_frame.rebuild()


    def update_recording_count(self):
        num_recordings = len(glob.glob(os.path.join(RECORDINGS_DIR, "*")))
        self.recording_count_label.text = f"Number of recordings: {num_recordings}"

    def create_config(self):
        config = Config(
            scenario_type=list(SCENARIOS.names())[self.scenario_combo_box.model.get_item_value_model().get_value_as_int()],
            robot_type=list(ROBOTS.names())[self.robot_combo_box.model.get_item_value_model().get_value_as_int()],
            scene_usd=self.scene_usd_field_string_model.as_string
        )
        return config
    
    def scenario_type(self):
        index = self.scenario_combo_box.model.get_item_value_model().get_value_as_int()
        return SCENARIOS.get_index(index)
    
    def on_shutdown(self):
        """Cleanup on extension shutdown."""
        try:
            # Stop ROS2 thread if running
            if self.ros2_thread_running:
                self._stop_ros2_thread()
            
            # Disconnect input drivers
            self.keyboard_driver.disconnect()
            self.gamepad_driver.disconnect()
            
            # Remove physics callback
            world = get_world()
            world.remove_physics_callback("scenario_physics")
            
            print("MobilityGen Extension Shutdown Complete")
            
        except Exception as e:
            print(f"Error during shutdown: {e}")
            import traceback
            traceback.print_exc()

    def start_new_recording(self):
        recording_name = datetime.datetime.now().isoformat()
        recording_path = os.path.join(RECORDINGS_DIR, recording_name)
        writer = Writer(recording_path)
        writer.write_config(self.config)
        writer.write_occupancy_map(self.scenario.occupancy_map)
        writer.copy_stage(self.cached_stage_path)
        self.step = 0
        self.recording_time = 0.
        self.recording_name_label.text = f"Current recording name: {recording_name}"
        self.recording_step_label.text = f"Current recording duration: {self.recording_time:.2f}s"
        self.writer = writer
        self.update_recording_count()
    
    def clear_recording(self):
        self.writer = None
        self.recording_name_label.text = "Current recording name: "
        self.recording_step_label.text = "Current recording duration: "

    def clear_scenario(self):
        self.scenario = None
        self.cached_stage_path = None

    def enable_recording(self):
        if not self.recording_enabled:
            if self.scenario is not None:
                self.start_new_recording()
            self.recording_enabled = True

    def disable_recording(self):
        self.recording_enabled = False
        self.clear_recording()

    def start_ros2_streaming(self):
        """Start ROS2 streaming."""
        try:
            # Get UI settings
            namespace = self.ros2_streaming_namespace_field.model.get_value_as_string()
            mode_index = self.ros2_streaming_mode_combo.model.get_item_value_model().get_value_as_int()
            mode_options = ["Stream Only", "Bag Only", "Stream + Bag"]
            mode = mode_options[mode_index]
            publish_camera_images = self.ros2_streaming_camera_images_checkbox.model.get_value_as_bool()
            
            print(f"Starting ROS2 streaming:")
            print(f"  Namespace: {namespace}")
            print(f"  Mode: {mode}")
            print(f"  Publish Images: {publish_camera_images}")
            
            try:
                # Initialize ROS2 for streaming
                self.ros2_manager.initialize_ros2()
                
                # Create ROS2 writer for streaming
                bag_path = None
                if mode in ["Bag Only", "Stream + Bag"]:
                    # Create a temporary bag file for streaming
                    import tempfile
                    bag_path = os.path.join(tempfile.gettempdir(), f"mobility_gen_streaming_{int(time.time())}.db3")
                
                # Create and store the ROS2 writer
                self.ros2_manager.ros2_writer = self.ros2_manager.create_writer(
                    namespace=namespace,
                    compression=False,  # No compression option in new UI
                    bag_path=bag_path,
                    mode=mode
                )
                
                if self.ros2_manager.ros2_writer is None:
                    print("Failed to create ROS2 writer")
                    return
                
                # Store settings
                self.ros2_manager.ros2_writer.publish_camera_images = publish_camera_images
                self.ros2_manager.ros2_writer.enable_processing = True  # Always enable processing for streaming
                self.ros2_manager.ros2_writer.camera_images_only = False  # No camera images only option in new UI
                
                # Reset streaming counters
                self.ros2_streaming_step = 0
                self.ros2_streaming_time = 0.0
                
                # Start ROS2 thread
                print("DEBUG: Starting ROS2 thread...")
                self._start_ros2_thread()
                
                # Update UI
                self.ros2_streaming_enabled = True
                self.ros2_streaming_status_label.text = f"ROS2 Streaming: Enabled ({mode})"
                
                print(f"ROS2 streaming started successfully")
                
            except Exception as e:
                print(f"Error starting ROS2 streaming: {e}")
                import traceback
                traceback.print_exc()
                
        except Exception as e:
            print(f"Error in start_ros2_streaming: {e}")
            import traceback
            traceback.print_exc()

    def stop_ros2_streaming(self):
        """Stop ROS2 streaming."""
        try:
            # Stop ROS2 thread
            self._stop_ros2_thread()
            
            # Cleanup ROS2
            self.ros2_manager.cleanup()
            
            # Update UI
            self.ros2_streaming_enabled = False
            self.ros2_streaming_status_label.text = "ROS2 Streaming: Disabled"
            
            print("ROS2 streaming stopped")
            
        except Exception as e:
            print(f"Error stopping ROS2 streaming: {e}")
            import traceback
            traceback.print_exc()

    def reset(self):
        self.writer = None
        self.scenario.reset()
        if self.recording_enabled:
            self.start_new_recording()

    def on_physics(self, step_size: float):
        """Called on every physics step."""
        
        # Update scenario
        if self.scenario is not None:
            self.scenario.step(step_size)
        
        # ROS2 processing in separate thread - ONLY put common data in queue
        if self.ros2_streaming_enabled and self.ros2_manager.ros2_enabled:
            self.ros2_streaming_step += 1
            
            # Get ONLY common state data for physics step (keep it fast)
            # NO camera data collection here - that happens in ROS2 thread
            state_dict_common = self.scenario.state_dict_common()
            
            # Put common data in queue for ROS2 thread (non-blocking)
            try:
                self.ros2_queue.put_nowait((state_dict_common, self.ros2_streaming_step))
            except queue.Full:
                print(f"WARNING: ROS2 queue full, dropping data for step {self.ros2_streaming_step}")

    def build_scenario(self):

        async def _build_scenario_async():
            
            self.clear_recording()
            self.clear_scenario()

            config = self.create_config()

            self.config = config
            self.scenario = await build_scenario_from_config(config)

            # Enable camera rendering for ROS2 streaming
            print("Enabling camera rendering for recording...")
            self.scenario.enable_rgb_rendering()
            self.scenario.enable_depth_rendering()
            self.scenario.enable_segmentation_rendering()
            self.scenario.enable_instance_id_segmentation_rendering()
            print("Camera rendering enabled")

            self.draw_occ_map()
            
            world = get_world()
            await world.reset_async()

            self.scenario.reset()

            world.add_physics_callback("scenario_physics", self.on_physics)

            # cache stage
            self.cached_stage_path = os.path.join(tempfile.mkdtemp(), "stage.usd")
            save_stage(self.cached_stage_path)

            if self.recording_enabled:
                self.start_new_recording()

            # self.scenario.save(path)

        asyncio.ensure_future(_build_scenario_async())

    def _ros2_thread_worker(self):
        """Worker function for ROS2 publishing thread."""
        print("ROS2 thread started")
        
        while self.ros2_thread_running:
            try:
                # Get data from queue with timeout
                data = self.ros2_queue.get(timeout=0.1)  # 100ms timeout
                
                if data is None:  # Shutdown signal
                    break
                
                # Extract common data
                state_dict_common, step = data
                
                # Process ALL camera data in this thread to avoid blocking physics
                if self.scenario is not None:
                    # Collect ALL camera data in this thread (not in physics thread)
                    state_dict_rgb = self.scenario.state_dict_rgb()
                    state_dict_depth = self.scenario.state_dict_depth()
                    state_dict_segmentation = self.scenario.state_dict_segmentation()
                    
                    # Combine all state data for ROS2 publishing
                    full_state_dict = {}
                    full_state_dict.update(state_dict_common)
                    full_state_dict.update(state_dict_rgb)
                    full_state_dict.update(state_dict_depth)
                    full_state_dict.update(state_dict_segmentation)
                    
                    # Publish to ROS2 (including camera data) in this thread
                    if self.ros2_manager.ros2_writer is not None:
                        # Publish common state data
                        self.ros2_manager.ros2_writer.write_state_dict_common(state_dict_common, step)
                        
                        # Publish camera data separately in this thread
                        self.ros2_manager.ros2_writer.write_common_state_data(full_state_dict, step)
                        
                        # Print status every 60 steps (about once per second)
                        if step % 60 == 0:
                            print(f"ROS2: Streaming step {step} with camera data")
                    else:
                        print(f"WARNING: ROS2 writer is None, cannot publish step {step}")
                    
            except queue.Empty:
                # No data available, continue
                continue
            except Exception as e:
                print(f"Error in ROS2 thread: {e}")
                import traceback
                traceback.print_exc()
        
        print("ROS2 thread stopped")

    def _start_ros2_thread(self):
        """Start the ROS2 publishing thread."""
        if self.ros2_thread is None or not self.ros2_thread.is_alive():
            self.ros2_thread_running = True
            self.ros2_thread = threading.Thread(target=self._ros2_thread_worker, daemon=True)
            self.ros2_thread.start()
            print("ROS2 thread started")

    def _stop_ros2_thread(self):
        """Stop the ROS2 publishing thread."""
        if self.ros2_thread is not None and self.ros2_thread.is_alive():
            self.ros2_thread_running = False
            # Send shutdown signal
            self.ros2_queue.put(None)
            self.ros2_thread.join(timeout=1.0)
            print("ROS2 thread stopped")