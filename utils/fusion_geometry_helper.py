import adsk.core, adsk.fusion
import numpy as np
import os
import tempfile
import traceback
from utils import dxf_exporter

def create_fusion_spline(sketch, control_points, knots, degree, is_closed=False):
    """
    Creates a high-accuracy NURBS spline in a Fusion 360 sketch.
    Note: These are created as 'Fixed' (read-only) geometry.
    """
    # Use a native Python list of Point3D objects for the API
    points_list = []
    for cp in control_points:
        z = cp[2] if len(cp) > 2 else 0
        points_list.append(adsk.core.Point3D.create(cp[0], cp[1], z))
    
    try:
        # Step 1: Create the mathematical NURBS geometry
        # This handles the exact degree and knots from the B-spline fit
        nurbs_geom = adsk.core.NurbsCurve3D.createNonRational(
            points_list, 
            degree, 
            knots.tolist() if hasattr(knots, 'tolist') else knots, 
            is_closed
        )
        
        if not nurbs_geom:
            raise RuntimeError("NurbsCurve3D.createNonRational returned None")

        # Step 2: Add it to the sketch as a Fixed Spline
        spline = sketch.sketchCurves.sketchFixedSplines.addByNurbsCurve(nurbs_geom)
        
        # Show the control polygon for reference
        if hasattr(spline, 'isControlPolygonVisible'):
            spline.isControlPolygonVisible = True
            
        return spline
        
    except Exception as e:
        raise RuntimeError(f"NURBS geometry creation failed: {str(e)}")

def import_splines_via_dxf(sketch, upper_cp, upper_knots, upper_degree, 
                           lower_cp, lower_knots, lower_degree, is_sharp_te, sketch_name="Fitted Airfoil"):
    """
    Imports splines via a temporary DXF file to ensure they are editable control-point splines.
    
    Fusion's DXF import interprets X/Y coordinates relative to the sketch plane,
    but the mapping varies depending on the plane orientation. We use the sketch's transform
    to determine the correct axis mapping and apply a correction rotation if needed.
    """
    app = adsk.core.Application.get()
    import_manager = app.importManager
    
    # Create a temporary DXF file
    temp_dir = tempfile.gettempdir()
    dxf_path = os.path.join(temp_dir, "fusion_fitter_temp.dxf")
    
    try:
        # Determine if we need to rotate coordinates for DXF import
        # Fusion's DXF import maps DXF X/Y to the sketch plane's X/Y axes
        # but modelToSketchSpace may return coordinates in a different orientation
        # depending on which world plane the sketch is on.
        #
        # We detect this by checking the sketch's transform matrix to see how
        # the sketch's local axes map to world axes.
        
        sketch_transform = sketch.transform
        
        # Extract the sketch's X and Y axis directions in world space
        # The transform matrix columns 0 and 1 give us the X and Y axis directions
        sketch_x_world = adsk.core.Vector3D.create(
            sketch_transform.getCell(0, 0),
            sketch_transform.getCell(1, 0),
            sketch_transform.getCell(2, 0)
        )
        sketch_y_world = adsk.core.Vector3D.create(
            sketch_transform.getCell(0, 1),
            sketch_transform.getCell(1, 1),
            sketch_transform.getCell(2, 1)
        )
        sketch_z_world = adsk.core.Vector3D.create(
            sketch_transform.getCell(0, 2),
            sketch_transform.getCell(1, 2),
            sketch_transform.getCell(2, 2)
        )
        
        
        # Check if the sketch normal is aligned with world X-axis (YZ plane case)
        # YZ plane has normal pointing along X axis
        world_x = adsk.core.Vector3D.create(1, 0, 0)
        dot_normal_with_world_x = abs(sketch_z_world.dotProduct(world_x))
        
        # If sketch normal is aligned with world X (dot product close to 1), 
        # we're on the YZ plane and need rotation correction
        needs_rotation = dot_normal_with_world_x > 0.9
        
        if needs_rotation:
            # Apply 90-degree counter-clockwise rotation for YZ plane correction
            # This swaps coordinates: (x, y) -> (-y, x)
            rotation_matrix = np.array([
                [0, -1, 0],
                [1,  0, 0],
                [0,  0, 1]
            ])
            rotated_upper = np.dot(upper_cp, rotation_matrix)
            rotated_lower = np.dot(lower_cp, rotation_matrix)
        else:
            rotated_upper = upper_cp
            rotated_lower = lower_cp
        
        # Generate DXF content
        doc = dxf_exporter.export_transformed_bspline_to_dxf(
            rotated_upper, upper_knots, upper_degree,
            rotated_lower, lower_knots, lower_degree,
            is_sharp_te, lambda msg: None
        )
        
        if not doc:
            raise RuntimeError("Failed to create DXF document")
            
        doc.saveas(dxf_path)
        
        # Import to Fusion
        # Note: DXF2DImportOptions requires a Component as the target.
        # It will create a NEW sketch on the specified reference plane.
        dxf_options = import_manager.createDXF2DImportOptions(dxf_path, sketch.referencePlane)
        
        # Use isCreateControlPointSplines to ensure they are editable control point splines
        if hasattr(dxf_options, 'isCreateControlPointSplines'):
            dxf_options.isCreateControlPointSplines = True
        else:
            # Fallback for older API versions
            dxf_options.isFitPointsToSplines = False
        
        # Get the component and sketch count before import to identify the new sketch
        parent_comp = sketch.parentComponent
        sketches = parent_comp.sketches
        count_before = sketches.count

        # Import into the component
        import_manager.importToTarget(dxf_options, parent_comp)
        
        # Try to rename the newly created sketch and show control polygons
        new_sketch = None
        if sketches.count > count_before:
            new_sketch = sketches.item(sketches.count - 1)
            new_sketch.name = sketch_name
            
            # Show the control polygon for all imported splines
            # Splines are in sketchFittedSplines or sketchFixedSplines
            for spline in new_sketch.sketchCurves.sketchFittedSplines:
                try:
                    spline.isControlPolygonVisible = True
                except:
                    pass
            for spline in new_sketch.sketchCurves.sketchFixedSplines:
                try:
                    spline.isControlPolygonVisible = True
                except:
                    pass
        
        # Clean up temp file
        if os.path.exists(dxf_path):
            os.remove(dxf_path)
            
        return new_sketch
    except Exception as e:
        if os.path.exists(dxf_path):
            try: os.remove(dxf_path)
            except: pass
        raise RuntimeError(f"DXF import failed: {str(e)}\n{traceback.format_exc()}")
