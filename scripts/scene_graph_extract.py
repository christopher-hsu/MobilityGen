from pxr import Usd, Sdf, UsdGeom, UsdLux, UsdShade

def generate_usd_summary(usd_file_path, max_depth=5, include_attributes=False):
    """
    Generates a summary of the USD scene graph.

    Args:
        usd_file_path (str): Path to the .usda, .usdc, or .usdz file.
        max_depth (int): Maximum depth to traverse in the scene graph.
        include_attributes (bool): Whether to include some key attributes.

    Returns:
        str: A formatted string summarizing the scene.
    """
    stage = Usd.Stage.Open(usd_file_path)
    if not stage:
        return f"Error: Could not open USD stage at {usd_file_path}"

    summary_lines = []
    summary_lines.append(f"--- Scene Summary for: {usd_file_path} ---")
    summary_lines.append(f"Default Prim: {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else 'None'}")
    summary_lines.append(f"Up Axis: {UsdGeom.GetStageUpAxis(stage)}") # Corrected in previous iteration

    summary_lines.append("\nObjects in Scene (Path | Type | Details):")

    # Create XformCache for efficient world transform calculations
    xform_cache = UsdGeom.XformCache()

    # Use stage.Traverse() for efficient traversal
    for prim in stage.Traverse():
        prim_path = prim.GetPath()

        # CORRECTED LINE FOR DEPTH CALCULATION:
        # pathElementCount returns the number of path components.
        # For '/', it's 0. For '/World', it's 1. For '/World/Object', it's 2.
        # We want our indentation to be 0 for root, 1 for /World, etc.
        prim_depth = prim_path.pathElementCount

        if prim_depth >= max_depth: # Use >= because 0 is depth, so max_depth 3 means 0, 1, 2
            continue # Skip deeper levels if max_depth is set

        # Indent based on depth. Root prim (depth 0) gets 0 spaces.
        indent = "  " * prim_depth

        line = f"{indent}{prim_path} | Type: {prim.GetTypeName()}"

        details = []
        
        # Get world position information for prims with transforms
        try:
            # Get world transform using XformCache for efficient calculation
            world_transform = xform_cache.GetLocalToWorldTransform(prim)
            if world_transform:
                # Extract translation from the world transformation matrix
                translation = world_transform.ExtractTranslation()
                if translation:
                    details.append(f"World Pos: ({translation[0]:.2f}, {translation[1]:.2f}, {translation[2]:.2f})")
        except Exception:
            pass  # Skip if we can't get transform data
        
        # Add common schema details
        if prim.IsA(UsdGeom.Mesh):
            details.append("Mesh")
            if include_attributes:
                points_attr = UsdGeom.Mesh(prim).GetPointsAttr()
                if points_attr and points_attr.HasAuthoredValue():
                    try:
                        num_points = len(points_attr.Get())
                        details.append(f"Points: {num_points}")
                    except Exception:
                        pass
        elif prim.IsA(UsdGeom.Camera):
            details.append("Camera")
            if include_attributes:
                focal_length_attr = UsdGeom.Camera(prim).GetFocalLengthAttr()
                if focal_length_attr and focal_length_attr.HasAuthoredValue():
                    details.append(f"Focal Length: {focal_length_attr.Get()}mm")
        elif prim.GetTypeName().startswith('Light'):
            details.append("Light")
            if include_attributes:
                intensity_attr = prim.GetAttribute("inputs:intensity")
                if not intensity_attr or not intensity_attr.HasAuthoredValue():
                    intensity_attr = prim.GetAttribute("intensity")
                if intensity_attr and intensity_attr.HasAuthoredValue():
                    details.append(f"Intensity: {intensity_attr.Get()}")
        elif prim.IsA(UsdShade.Material):
            details.append("Material")

        if UsdShade.MaterialBindingAPI.CanApply(prim):
            material_binding_api = UsdShade.MaterialBindingAPI(prim)
            direct_binding = material_binding_api.GetDirectBinding()
            if direct_binding:
                material_prim = direct_binding.GetMaterial()
                if material_prim:
                    details.append(f"Material: {material_prim.GetPath()}")
            material_id_attr = prim.GetAttribute("material:binding")
            if material_id_attr and material_id_attr.HasAuthoredValue():
                 details.append(f"Material ID: {material_id_attr.Get()}")

        if details:
            line += f" ({', '.join(details)})"

        summary_lines.append(line)

    return "\n".join(summary_lines)

# # --- How to use the function ---
# # Replace with the actual path to your forklift USD file
# usda_file_path = "forkliftc_ackermanncontroller.usd"

# # Try running with max_depth and include_attributes
# scene_overview = generate_usd_summary(usda_file_path, max_depth=3, include_attributes=True)
# print(scene_overview)

# --- How to use the function ---
# Remember to replace 'path/to/your/large_file.usda' with the actual path
# If the summary is still too long, reduce max_depth or set include_attributes=False
# or add more specific filtering logic based on your needs.
# For example, only summarize Mesh and Camera prims.

# Example Usage:
# usda_file = "path/to/your/large_file.usda"
# scene_overview = generate_usd_summary(usda_file, max_depth=3, include_attributes=True)
# print(scene_overview)

# Then copy the output of print(scene_overview) and paste it as context to Gemini.


# finish the code
if __name__ == "__main__":
    # usda_file_path = '/data/scenes/full_warehouse.usda'
    # usda_file_path = '/data/scenes/hospital.usda'
    # usda_file_path = '/data/scenes/mobility_gen_warehouse.usda'
    usda_file_path = '/data/scenes/office.usda'
    scene_overview = generate_usd_summary(usda_file_path, max_depth=5, include_attributes=True)
    print(scene_overview)
    
    # Save to text file in the same directory as the source file
    import os
    source_dir = os.path.dirname(usda_file_path)
    source_filename = os.path.splitext(os.path.basename(usda_file_path))[0]
    output_file = os.path.join(source_dir, f"{source_filename}_scene_summary.txt")
    
    with open(output_file, 'w') as f:
        f.write(scene_overview)
    
    print(f"\nScene summary saved to: {output_file}")