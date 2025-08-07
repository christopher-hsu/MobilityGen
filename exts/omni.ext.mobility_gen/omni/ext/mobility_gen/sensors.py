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
        
        # Find the actual camera prim for rendering
        camera_prim = self._find_camera_prim()
        if camera_prim is None:
            # Fallback to using the prim_path
            render_path = self._prim_path
        else:
            render_path = str(camera_prim.GetPath())
        
        # For RealSense, use the Color camera for RGB data
        stage = get_stage()
        if "RSD455" in self._prim_path:
            rsd455_path = self._prim_path
            if not rsd455_path.endswith("RSD455"):
                rsd455_path = os.path.join(self._prim_path, "RSD455")
            
            rsd455_prim = stage_get_prim(stage, rsd455_path)
            if rsd455_prim.IsValid():
                for child in rsd455_prim.GetChildren():
                    if child.GetTypeName() == "Camera" and "Color" in str(child.GetPath()):
                        render_path = str(child.GetPath())
                        break
        
        self._render_product = rep.create.render_product(
            render_path,
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
            except Exception as e:
                pass
        
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


class RealSenseCamera(Camera):
    """RealSense D455 camera class for handling Intel RealSense D455 sensor."""
    
    usd_url: str = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Sensors/Intel/RealSense/rsd455.usd"
    resolution: Tuple[int, int] = (848, 480)  # Standard RealSense D455 resolution
    
    @classmethod
    def _fix_physics_hierarchy(cls, prim_path: str):
        """Fix the physics hierarchy issue by removing RigidBodyAPI from camera prims."""
        from pxr import UsdPhysics
        from omni.ext.mobility_gen.utils.global_utils import get_stage
        from omni.ext.mobility_gen.utils.stage_utils import stage_get_prim
        
        stage = get_stage()
        
        # Find the camera prim
        camera_prim = stage_get_prim(stage, prim_path)
        if not camera_prim.IsValid():
            print(f"Warning: Camera prim not found at {prim_path}")
            return
        
        # Remove RigidBodyAPI from the camera and its children
        def remove_rigid_body_api(prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f"Removed RigidBodyAPI from {prim.GetPath()}")
            
            # Recursively process children
            for child in prim.GetChildren():
                remove_rigid_body_api(child)
        
        remove_rigid_body_api(camera_prim)
    
    @classmethod
    def build(cls, prim_path: str) -> "RealSenseCamera":
        """Build a RealSense camera at the specified prim path."""
        stage = get_stage()

        # Add the USD reference to get the RealSense camera prims
        stage_add_usd_ref(
            stage=stage,
            path=prim_path,
            usd_path=cls.usd_url
        )

        # Fix the physics hierarchy issue
        cls._fix_physics_hierarchy(prim_path)

        return cls.attach(prim_path)
    
    @classmethod
    def attach(cls, prim_path: str) -> "RealSenseCamera":
        """Attach to an existing RealSense camera at the specified prim path."""
        
        # Find the RSD455 camera
        rgb_path = os.path.join(prim_path, "RSD455")
        if not stage_get_prim(get_stage(), rgb_path).IsValid():
            return None
        
        # Create camera at RSD455 path with default resolution
        camera = cls(rgb_path, cls.resolution)
        
        # Find the Color camera for RGB data and get its actual resolution
        stage = get_stage()
        rsd455_prim = stage_get_prim(stage, rgb_path)
        if rsd455_prim.IsValid():
            for child in rsd455_prim.GetChildren():
                if child.GetTypeName() == "Camera" and "Color" in str(child.GetPath()):
                    camera._prim_path = str(child.GetPath())
                    camera._prim = child
                    
                    # Update the XForm to use the Color camera path for transform
                    camera._xform_prim = XFormPrim(str(child.GetPath()))
                    
                    # Get the actual resolution from the camera prim
                    try:
                        # Try to get resolution from camera attributes
                        width_attr = child.GetAttribute("width")
                        height_attr = child.GetAttribute("height")
                        if width_attr.HasValue() and height_attr.HasValue():
                            actual_width = width_attr.Get()
                            actual_height = height_attr.Get()
                            camera._resolution = (actual_width, actual_height)
                        else:
                            # Use default resolution if attributes not found
                            pass
                    except:
                        # Fallback to default resolution
                        pass
                    break
        
        return camera

    def update_state(self):
        """Update the state of the RealSense camera."""
        # Just call the parent Camera's update_state - it has all the buffers we need
        super().update_state()
