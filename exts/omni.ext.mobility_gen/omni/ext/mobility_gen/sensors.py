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


import os
import numpy as np
from typing import Tuple


import omni.replicator.core as rep
from isaacsim.core.prims import SingleXFormPrim as XFormPrim


from omni.ext.mobility_gen.utils.global_utils import get_stage
from omni.ext.mobility_gen.utils.stage_utils import stage_add_usd_ref
from omni.ext.mobility_gen.common import Module, Buffer


class Sensor(Module):

    def build(self, prim_path: str):
        raise NotImplementedError
    
    def attach(self, prim_path: str):
        raise NotImplementedError


class Camera(Sensor):

    def __init__(self,
            prim_path: str,
            resolution: Tuple[int, int]
        ):

        self._prim_path = prim_path
        self._resolution = resolution
        self._render_product = None
        self._rgb_annotator = None
        self._segmentation_annotator = None
        self._instance_id_segmentation_annotator = None
        self._normals_annotator = None
        self._depth_annotator = None
        self._xform_prim = XFormPrim(self._prim_path)

        self.rgb_image = Buffer(tags=["rgb"])
        self.segmentation_image = Buffer(tags=["segmentation"])
        self.segmentation_info = Buffer()
        self.depth_image = Buffer(tags=["depth"])
        self.instance_id_segmentation_image = Buffer(tags=["segmentation"])
        self.instance_id_segmentation_info = Buffer()
        self.normals_image = Buffer(tags=['normals'])
        self.position = Buffer()
        self.orientation = Buffer()

    def enable_rendering(self):
        
        self._render_product = rep.create.render_product(
            self._prim_path,
            self._resolution,
            force_new=False
        )

    def disable_rendering(self):
        if self._render_product is None:
            return
        
        if self._rgb_annotator is not None:
            self._rgb_annotator.detach()
            self._rgb_annotator = None
        
        if self._segmentation_annotator is not None:
            self._segmentation_annotator.detach()
            self._segmentation_annotator = None

        if self._depth_annotator is not None:
            self._depth_annotator.detach()
            self._depth_annotator = None

        self._render_product.destroy()
        self._render_product = None
    
    def enable_rgb_rendering(self):
        if self._render_product is None:
            self.enable_rendering()
        if self._rgb_annotator is not None:
            return
        self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("LdrColor")
        self._rgb_annotator.attach(self._render_product)

    def enable_segmentation_rendering(self):
        if self._render_product is None:
            self.enable_rendering()
        if self._segmentation_annotator is not None:
            return
        self._segmentation_annotator = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params=dict(colorize=False)
        )
        self._segmentation_annotator.attach(self._render_product)

    def enable_instance_id_segmentation_rendering(self):
        if self._render_product is None:
            self.enable_rendering()
        if self._instance_id_segmentation_annotator is not None:
            return
        self._instance_id_segmentation_annotator = rep.AnnotatorRegistry.get_annotator(
            "instance_id_segmentation", init_params=dict(colorize=False)
        )
        self._instance_id_segmentation_annotator.attach(self._render_product)

    def enable_depth_rendering(self):
        if self._render_product is None:
            self.enable_rendering()
        if self._depth_annotator is not None:
            return
        self._depth_annotator = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self._depth_annotator.attach(self._render_product)

    def enable_normals_rendering(self):
        if self._render_product is None:
            self.enable_rendering()
        if self._normals_annotator is not None:
            return
        self._normals_annotator = rep.AnnotatorRegistry.get_annotator(
            "normals"
        )
        self._normals_annotator.attach(self._render_product)

    def update_state(self):
        if self._rgb_annotator is not None:
            try:
                rgb_data = self._rgb_annotator.get_data()
                # Check if data is 3D (height, width, channels)
                if len(rgb_data.shape) == 3:
                    self.rgb_image.set_value(rgb_data[:, :, :3])
                else:
                    print(f"Warning: RGB data has unexpected shape: {rgb_data.shape}")
                    # Try to reshape if it's 1D
                    if len(rgb_data.shape) == 1:
                        # This might be a flattened image, skip for now
                        print(f"RGB data is 1D with shape {rgb_data.shape}, skipping")
                        pass
            except Exception as e:
                print(f"Error processing RGB data: {e}")
        else:
            print(f"Warning: RGB annotator is None for camera at {self._prim_path}")
        
        if self._segmentation_annotator is not None:
            data = self._segmentation_annotator.get_data()
            seg_image = data['data']
            seg_info = data['info']
            self.segmentation_image.set_value(seg_image)
            self.segmentation_info.set_value(seg_info)

        if self._depth_annotator is not None:
            self.depth_image.set_value(
                self._depth_annotator.get_data()
            )

        if self._instance_id_segmentation_annotator is not None:
            data = self._instance_id_segmentation_annotator.get_data()
            id_seg_image = data['data']
            id_seg_info = data['info']
            self.instance_id_segmentation_image.set_value(id_seg_image)
            self.instance_id_segmentation_info.set_value(id_seg_info)

        if self._normals_annotator is not None:
            data = self._normals_annotator.get_data()
            self.normals_image.set_value(data)
            
        position, orientation = self._xform_prim.get_world_pose()
        self.position.set_value(position)
        self.orientation.set_value(orientation)
        
        super().update_state()


#=========================================================
#  FINAL CLASSES
#=========================================================


class HawkCamera(Sensor):

    usd_url: str = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
    resolution: Tuple[int, int] = (960, 600)
    left_camera_path: str = "left/camera_left"
    right_camera_path: str = "right/camera_right"

    def __init__(self, 
            left: Camera, 
            right: Camera
        ):
        self.left = left
        self.right = right
    
    @classmethod
    def build(cls, prim_path: str) -> "HawkCamera":
        
        stage = get_stage()

        stage_add_usd_ref(
            stage=stage,
            path=prim_path,
            usd_path=cls.usd_url
        )

        return cls.attach(prim_path)
    
    @classmethod
    def attach(cls, prim_path: str) -> "HawkCamera":
        
        left_camera = Camera(os.path.join(prim_path, cls.left_camera_path), cls.resolution)
        right_camera = Camera(os.path.join(prim_path, cls.right_camera_path), cls.resolution)

        return HawkCamera(left_camera, right_camera)


class RealSense2Camera(Sensor):
    """RealSense2 camera sensor implementation.
    
    This class provides a single camera interface for RealSense2 cameras,
    supporting RGB and depth data. Unlike HawkCamera which is stereo,
    RealSense2Camera provides a single camera with both RGB and depth capabilities.
    
    Note: Since the RealSense D455 USD asset is not accessible, this implementation
    uses the Hawk camera asset but only uses one camera to simulate RealSense behavior.
    """

    # Use Hawk camera asset as fallback since RealSense D455 USD is not accessible
    usd_url: str = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
    resolution: Tuple[int, int] = (640, 480)
    camera_path: str = "left/camera_left"  # Use only the left camera to simulate single camera
    
    # RealSense D455 camera intrinsics (typical values)
    # These can be calibrated for your specific camera
    focal_length_mm: float = 1.93  # Focal length in mm
    sensor_width_mm: float = 3.68  # Sensor width in mm
    sensor_height_mm: float = 2.76  # Sensor height in mm
    fx: float = 386.0  # Focal length in pixels (x)
    fy: float = 386.0  # Focal length in pixels (y)
    cx: float = 320.0  # Principal point x
    cy: float = 240.0  # Principal point y

    def __init__(self, camera: Camera):
        self.camera = camera
        
        # Initialize buffers to match the interface expected by robots
        # These will be forwarded from the underlying camera
        self.rgb_image = Buffer(tags=["rgb"])
        self.depth_image = Buffer(tags=["depth"])
        self.segmentation_image = Buffer(tags=["segmentation"])
        self.segmentation_info = Buffer()
        self.instance_id_segmentation_image = Buffer(tags=["segmentation"])
        self.instance_id_segmentation_info = Buffer()
        self.normals_image = Buffer(tags=['normals'])
        self.position = Buffer()
        self.orientation = Buffer()
    
    @classmethod
    def get_realsense_d455_intrinsics(cls) -> dict:
        """Get the camera intrinsics for RealSense D455.
        
        Returns:
            dict: Dictionary containing camera intrinsics including:
                - fx, fy: Focal lengths in pixels
                - cx, cy: Principal point coordinates
                - width, height: Image dimensions
                - focal_length_mm: Focal length in mm
                - sensor_width_mm, sensor_height_mm: Sensor dimensions
        """
        return {
            'fx': cls.fx,
            'fy': cls.fy,
            'cx': cls.cx,
            'cy': cls.cy,
            'width': cls.resolution[0],
            'height': cls.resolution[1],
            'focal_length_mm': cls.focal_length_mm,
            'sensor_width_mm': cls.sensor_width_mm,
            'sensor_height_mm': cls.sensor_height_mm
        }
    
    @classmethod
    def get_realsense_d455_intrinsics_matrix(cls) -> np.ndarray:
        """Get the camera intrinsics matrix for RealSense D455.
        
        Returns:
            np.ndarray: 3x3 camera intrinsics matrix in the format:
                [[fx, 0,  cx],
                 [0,  fy, cy],
                 [0,  0,  1 ]]
        """
        import numpy as np
        return np.array([
            [cls.fx, 0, cls.cx],
            [0, cls.fy, cls.cy],
            [0, 0, 1]
        ])
    
    @classmethod
    def build(cls, prim_path: str) -> "RealSense2Camera":
        
        stage = get_stage()

        stage_add_usd_ref(
            stage=stage,
            path=prim_path,
            usd_path=cls.usd_url
        )

        return cls.attach(prim_path)
    
    @classmethod
    def attach(cls, prim_path: str) -> "RealSense2Camera":
        
        camera = Camera(os.path.join(prim_path, cls.camera_path), cls.resolution)
        
        # Enable both RGB and depth rendering for RealSense2
        camera.enable_rgb_rendering()
        camera.enable_depth_rendering()

        return RealSense2Camera(camera)
    
    def update_state(self):
        # Update the underlying camera state
        self.camera.update_state()
        
        # Forward the camera's buffer values to this sensor's interface
        # This allows the RealSense2Camera to be used as a drop-in replacement
        # for HawkCamera in robot configurations
        # Simply forward the values - the underlying camera will handle the logic
        self.rgb_image.set_value(self.camera.rgb_image.get_value())
        self.depth_image.set_value(self.camera.depth_image.get_value())
        self.segmentation_image.set_value(self.camera.segmentation_image.get_value())
        self.segmentation_info.set_value(self.camera.segmentation_info.get_value())
        self.instance_id_segmentation_image.set_value(self.camera.instance_id_segmentation_image.get_value())
        self.instance_id_segmentation_info.set_value(self.camera.instance_id_segmentation_info.get_value())
        self.normals_image.set_value(self.camera.normals_image.get_value())
        self.position.set_value(self.camera.position.get_value())
        self.orientation.set_value(self.camera.orientation.get_value())
        
        super().update_state()