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
Camera Configuration Loader

This module provides utilities for loading camera intrinsics from YAML configuration files.
"""

import os
import yaml
import numpy as np
from typing import Dict, Any, Optional, Tuple


class CameraConfigLoader:
    """Loader for camera intrinsics configuration from YAML files."""
    
    def __init__(self, config_path: Optional[str] = None):
        """Initialize the camera config loader.
        
        Args:
            config_path: Path to the YAML configuration file. If None, uses default path.
        """
        if config_path is None:
            # Use default config path relative to this module
            module_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(module_dir, "camera_intrinsics.yaml")
        
        self.config_path = config_path
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load the YAML configuration file."""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"✓ Loaded camera intrinsics config from: {self.config_path}")
            return config
        except Exception as e:
            print(f"Error loading camera config from {self.config_path}: {e}")
            return {}
    
    def get_camera_intrinsics(self, camera_type: str, camera_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """Get camera intrinsics from configuration.
        
        Args:
            camera_type: Type of camera (e.g., 'hawk_camera', 'realsense_camera', 'single_camera')
            camera_name: Name of the camera (e.g., 'left', 'right'). For stereo cameras.
            
        Returns:
            Dictionary containing camera intrinsics parameters, or None if not found.
        """
        if camera_type not in self.config:
            print(f"Warning: Camera type '{camera_type}' not found in config")
            return None
        
        camera_config = self.config[camera_type]
        
        if camera_name is None:
            # For single cameras, return the config directly
            if 'width' in camera_config:
                return camera_config
            else:
                print(f"Warning: Camera type '{camera_type}' requires a camera name")
                return None
        
        # For stereo cameras, get the specific camera
        if camera_name not in camera_config:
            print(f"Warning: Camera '{camera_name}' not found in '{camera_type}' config")
            return None
        
        return camera_config[camera_name]
    
    def get_intrinsics_matrix(self, camera_type: str, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        """Get camera intrinsics as a 3x3 numpy matrix.
        
        Args:
            camera_type: Type of camera (e.g., 'hawk_camera', 'realsense_camera', 'single_camera')
            camera_name: Name of the camera (e.g., 'left', 'right'). For stereo cameras.
            
        Returns:
            3x3 intrinsics matrix, or None if not found.
        """
        intrinsics_dict = self.get_camera_intrinsics(camera_type, camera_name)
        if intrinsics_dict is None:
            return None
        
        # Create 3x3 intrinsics matrix
        # [fx  0  cx]
        # [0   fy cy]
        # [0   0   1]
        intrinsics_matrix = np.array([
            [intrinsics_dict["fx"], 0, intrinsics_dict["cx"]],
            [0, intrinsics_dict["fy"], intrinsics_dict["cy"]],
            [0, 0, 1]
        ], dtype=np.float64)
        
        return intrinsics_matrix
    
    def get_distortion_coefficients(self, camera_type: str, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        """Get distortion coefficients from configuration.
        
        Args:
            camera_type: Type of camera (e.g., 'hawk_camera', 'realsense_camera', 'single_camera')
            camera_name: Name of the camera (e.g., 'left', 'right'). For stereo cameras.
            
        Returns:
            Distortion coefficients as numpy array, or None if not found.
        """
        intrinsics_dict = self.get_camera_intrinsics(camera_type, camera_name)
        if intrinsics_dict is None or "distortion_coefficients" not in intrinsics_dict:
            return None
        
        return np.array(intrinsics_dict["distortion_coefficients"], dtype=np.float64)
    
    def get_resolution(self, camera_type: str, camera_name: Optional[str] = None) -> Optional[Tuple[int, int]]:
        """Get camera resolution from configuration.
        
        Args:
            camera_type: Type of camera (e.g., 'hawk_camera', 'realsense_camera', 'single_camera')
            camera_name: Name of the camera (e.g., 'left', 'right'). For stereo cameras.
            
        Returns:
            Tuple of (width, height), or None if not found.
        """
        intrinsics_dict = self.get_camera_intrinsics(camera_type, camera_name)
        if intrinsics_dict is None:
            return None
        
        return (intrinsics_dict["width"], intrinsics_dict["height"])

    def get_sensor_dimensions(self, camera_type: str, camera_name: Optional[str] = None) -> Optional[Tuple[float, float]]:
        """Get sensor dimensions from configuration.
        
        Args:
            camera_type: Type of camera (e.g., 'hawk_camera', 'realsense_camera', 'single_camera')
            camera_name: Name of the camera (e.g., 'left', 'right'). For stereo cameras.
            
        Returns:
            Tuple of (sensor_width_mm, sensor_height_mm), or None if not found.
        """
        intrinsics_dict = self.get_camera_intrinsics(camera_type, camera_name)
        if intrinsics_dict is None:
            return None
        
        if "sensor_width_mm" not in intrinsics_dict or "sensor_height_mm" not in intrinsics_dict:
            print(f"Warning: Sensor dimensions not found for {camera_type}/{camera_name}")
            return None
        
        return (intrinsics_dict["sensor_width_mm"], intrinsics_dict["sensor_height_mm"])


# Global config loader instance
_camera_config_loader = None

def get_camera_config_loader() -> CameraConfigLoader:
    """Get the global camera config loader instance."""
    global _camera_config_loader
    if _camera_config_loader is None:
        _camera_config_loader = CameraConfigLoader()
    return _camera_config_loader 