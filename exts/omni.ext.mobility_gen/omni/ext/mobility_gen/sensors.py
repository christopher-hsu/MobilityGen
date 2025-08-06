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
from typing import Tuple, Optional


import omni.replicator.core as rep
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from pxr import UsdGeom


from omni.ext.mobility_gen.utils.global_utils import get_stage
from omni.ext.mobility_gen.utils.stage_utils import stage_add_usd_ref, stage_get_prim
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
        
        # Store the USD prim reference for accessing camera intrinsics
        stage = get_stage()
        self._prim = stage_get_prim(stage, prim_path)

        self.rgb_image = Buffer(tags=["rgb"])
        self.segmentation_image = Buffer(tags=["segmentation"])
        self.segmentation_info = Buffer()
        self.depth_image = Buffer(tags=["depth"])
        self.instance_id_segmentation_image = Buffer(tags=["segmentation"])
        self.instance_id_segmentation_info = Buffer()
        self.normals_image = Buffer(tags=['normals'])
        self.position = Buffer()
        self.orientation = Buffer()
        self.intrinsics = Buffer(tags=["intrinsics"])

    @property
    def prim(self):
        """Get the USD prim reference for this camera."""
        return self._prim
    
    @property
    def resolution(self):
        """Get the camera resolution."""
        return self._resolution

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
        
        # Calculate and store camera intrinsics
        try:
            intrinsics = self._calculate_intrinsics()
            if intrinsics is not None:
                self.intrinsics.set_value(intrinsics)
        except Exception as e:
            print(f"Warning: Could not calculate intrinsics for camera at {self._prim_path}: {e}")
        
        super().update_state()

    def _calculate_intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        """
        Calculate camera intrinsics from USD prim attributes.
        
        Returns:
            Optional[Tuple[float, float, float, float]]: (fx, fy, cx, cy) or None if not available
        """
        try:
            # Check if prim is valid
            if not self._prim.IsValid():
                print(f"Warning: Camera prim at {self._prim_path} is not valid")
                return None
            
            # Find the actual camera prim with camera attributes
            camera_prim = self._find_camera_prim()
            if camera_prim is None:
                print(f"Warning: No camera prim found in hierarchy at {self._prim_path}")
                return None
            
            # Get the USD camera attributes from the found camera prim
            focal_length_attr = camera_prim.GetAttribute("focalLength")
            horizontal_aperture_attr = camera_prim.GetAttribute("horizontalAperture")
            vertical_aperture_attr = camera_prim.GetAttribute("verticalAperture")

            # Check if attributes have values
            if not focal_length_attr.HasValue():
                print(f"Warning: focalLength attribute not found for camera at {camera_prim.GetPath()}")
                return None
            if not horizontal_aperture_attr.HasValue():
                print(f"Warning: horizontalAperture attribute not found for camera at {camera_prim.GetPath()}")
                return None
            if not vertical_aperture_attr.HasValue():
                print(f"Warning: verticalAperture attribute not found for camera at {camera_prim.GetPath()}")
                return None

            focal_length = focal_length_attr.Get()
            horizontal_aperture = horizontal_aperture_attr.Get()
            vertical_aperture = vertical_aperture_attr.Get()

            # Calculate the intrinsic matrix values
            image_width, image_height = self._resolution
            fx = (image_width * focal_length) / horizontal_aperture
            fy = (image_height * focal_length) / vertical_aperture
            cx = image_width / 2.0
            cy = image_height / 2.0

            return (fx, fy, cx, cy)
            
        except Exception as e:
            print(f"Error calculating intrinsics for camera at {self._prim_path}: {e}")
            return None

    def _find_camera_prim(self) -> Optional[object]:
        """
        Find the actual camera prim with camera attributes in the hierarchy.
        
        Returns:
            Optional[object]: The camera prim with camera attributes, or None if not found
        """
        try:
            # First check if the current prim is a camera
            if self._prim.GetTypeName() == "Camera":
                return self._prim
            
            # Search children for camera prims
            for child in self._prim.GetChildren():
                if child.GetTypeName() == "Camera":
                    return child
            
            # Search deeper in the hierarchy
            def search_camera_prim(prim):
                if prim.GetTypeName() == "Camera":
                    return prim
                for child in prim.GetChildren():
                    result = search_camera_prim(child)
                    if result is not None:
                        return result
                return None
            
            camera_prim = search_camera_prim(self._prim)
            return camera_prim
            
        except Exception as e:
            print(f"Error searching for camera prim: {e}")
            return None


#=========================================================
#  FINAL CLASSES
#=========================================================


class HawkCamera(Sensor):

    usd_url: str = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
    resolution: Tuple[int, int] = (960, 600)
    left_camera_path: str = "left/camera_left"
    right_camera_path: str = "right/camera_right"

    def __init__(self, 
            left: Camera = None, 
            right: Camera = None,
            single_camera: Camera = None
        ):
        self.left = left
        self.right = right
        self.single_camera = single_camera
    
    @classmethod
    def build(cls, prim_path: str) -> "HawkCamera":
        
        stage = get_stage()

        # First, add the USD reference to get the camera prims
        stage_add_usd_ref(
            stage=stage,
            path=prim_path,
            usd_path=cls.usd_url
        )

        # Check if this is a stereo camera setup (has left/right paths)
        left_path = os.path.join(prim_path, cls.left_camera_path)
        right_path = os.path.join(prim_path, cls.right_camera_path)
        
        # Check if both left and right cameras exist
        left_prim = stage.GetPrimAtPath(left_path)
        right_prim = stage.GetPrimAtPath(right_path)
        
        if left_prim.IsValid() and right_prim.IsValid():
            # This is a stereo camera setup
            return cls.attach(prim_path)
        else:
            # This is a single camera setup
            return cls.attach_single(prim_path)
    
    @classmethod
    def attach(cls, prim_path: str) -> "HawkCamera":
        
        left_camera = Camera(os.path.join(prim_path, cls.left_camera_path), cls.resolution)
        right_camera = Camera(os.path.join(prim_path, cls.right_camera_path), cls.resolution)

        return HawkCamera(left=left_camera, right=right_camera)
    
    @classmethod
    def attach_single(cls, prim_path: str) -> "HawkCamera":
        """Attach to a single camera (for robots like Spot that use HawkCamera as single camera)"""
        single_camera = Camera(prim_path, cls.resolution)
        return HawkCamera(single_camera=single_camera)

    def update_state(self):
        """Update the state of the HawkCamera, including intrinsics for all cameras."""
        # Update individual cameras
        if self.left is not None:
            self.left.update_state()
        if self.right is not None:
            self.right.update_state()
        if self.single_camera is not None:
            self.single_camera.update_state()
        
        super().update_state()