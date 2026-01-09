import adsk.core, adsk.fusion
import numpy as np
import os
import tempfile
import traceback
import math
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
                           lower_cp, lower_knots, lower_degree, is_sharp_te, 
                           chord_start_world, chord_end_world, sketch_name="Fitted Airfoil"):
    """
    Imports splines via a temporary DXF file to ensure they are editable control-point splines.
    After import, aligns the airfoil with the chord line by moving the leading edge to the
    chord start point and rotating if needed to align the trailing edge with the chord end point.
    
    Args:
        sketch: The target sketch for import
        upper_cp, lower_cp: Control points for upper and lower surfaces
        upper_knots, lower_knots: Knot vectors
        upper_degree, lower_degree: Degrees of the splines
        is_sharp_te: Whether trailing edge is sharp
        chord_start_world: World coordinates of chord line start point (Point3D)
        chord_end_world: World coordinates of chord line end point (Point3D)
        sketch_name: Name for the new sketch
    """
    app = adsk.core.Application.get()
    import_manager = app.importManager
    
    # Create a temporary DXF file
    temp_dir = tempfile.gettempdir()
    dxf_path = os.path.join(temp_dir, "fusion_fitter_temp.dxf")
    
    try:
        # Generate DXF content without any pre-rotation
        doc = dxf_exporter.export_transformed_bspline_to_dxf(
            upper_cp, upper_knots, upper_degree,
            lower_cp, lower_knots, lower_degree,
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
            
            _align_airfoil_with_chord(new_sketch, chord_start_world, chord_end_world)
        else:
            # If no new sketch was created, log a warning
            app = adsk.core.Application.get()
            app.log(f"Warning: DXF import did not create a new sketch. Expected {count_before + 1}, got {sketches.count}")
        
        # Clean up temp file
        if os.path.exists(dxf_path):
            os.remove(dxf_path)
            
        return new_sketch
    except Exception as e:
        if os.path.exists(dxf_path):
            try: os.remove(dxf_path)
            except: pass
        raise RuntimeError(f"DXF import failed: {str(e)}\n{traceback.format_exc()}")


def _align_airfoil_with_chord(sketch, chord_start_world, chord_end_world):
    """
    Aligns the imported airfoil with the chord line by:
    1. Moving the leading edge to the chord start point
    2. Rotating if needed to align the trailing edge with the chord end point
    
    This function should always be called after DXF import, regardless of rotation state.
    
    Args:
        sketch: The sketch containing the imported airfoil splines
        chord_start_world: World coordinates of chord line start point (Point3D)
        chord_end_world: World coordinates of chord line end point (Point3D)
    """
    try:
        app = adsk.core.Application.get()
        app.log(f"Aligning airfoil with chord line. Chord start: {chord_start_world}, Chord end: {chord_end_world}")
        if not sketch:
            app.log("Warning: Cannot align airfoil - sketch is None")
            return
        
        # Convert chord points to sketch space
        # Note: modelToSketchSpace converts from world/model coordinates to sketch coordinates
        chord_start_sketch = sketch.modelToSketchSpace(chord_start_world)
        chord_end_sketch = sketch.modelToSketchSpace(chord_end_world)
        
        # Find all splines and lines in the sketch (for trailing edge connector, etc.)
        all_splines = []
        for i in range(sketch.sketchCurves.sketchControlPointSplines.count):
            try:
                spline = sketch.sketchCurves.sketchControlPointSplines.item(i)
                all_splines.append(spline)
            except Exception as e:
                app = adsk.core.Application.get()
                app.log(f"Error getting spline {i}: {e}")
            
        # Also get any lines (e.g., trailing edge connector for blunt TE)
        all_lines = []
        for line in sketch.sketchCurves.sketchLines:
            all_lines.append(line)
        
        if len(all_splines) < 2:
            return  # Need at least upper and lower splines
        
        # Get the leading edge point from the first spline's start point
        # The leading edge is where both upper and lower surfaces meet (start of upper spline)
        le_spline = all_splines[0]
        le_sketch_point = le_spline.startSketchPoint
        le_point_sketch = le_sketch_point.geometry
        
        # Find trailing edge - get end points from both splines
        # For sharp TE, both end at same point. For blunt TE, take midpoint
        te_points_sketch = []
        for spline in all_splines:
            te_sketch_point = spline.endSketchPoint
            te_points_sketch.append(te_sketch_point.geometry)
        
        # Use midpoint of trailing edge points
        if len(te_points_sketch) >= 2:
            te_point_sketch = adsk.core.Point3D.create(
                (te_points_sketch[0].x + te_points_sketch[1].x) / 2.0,
                (te_points_sketch[0].y + te_points_sketch[1].y) / 2.0,
                (te_points_sketch[0].z + te_points_sketch[1].z) / 2.0
            )
        else:
            te_point_sketch = te_points_sketch[0]

        le_vec = adsk.core.Vector3D.create(
            le_point_sketch.x - chord_start_sketch.x,
            le_point_sketch.y - chord_start_sketch.y,
            le_point_sketch.z - chord_start_sketch.z
        )
        if le_vec.length < 1e-10:
            return
        
        entities = adsk.core.ObjectCollection.create()
        for spline in all_splines:
            entities.add(spline)
        for line in all_lines:
            entities.add(line)
        try:                
            # Define the rotation transform (90 degrees counterclockwise around origin)
            # Create a matrix for 90-degree rotation in the XY plane
            transform = adsk.core.Matrix3D.create()
            origin = adsk.core.Point3D.create(0, 0, 0)
            zAxis = adsk.core.Vector3D.create(0, 0, 1)
            transform.setToRotation(-math.pi / 2, zAxis, origin)  # 90 degrees = π/2 radians
            sketch.move(entities, transform)
            
        except Exception as e:
            app = adsk.core.Application.get()
            app.log(f"Error moving spline {spline.name}: {e}")

        
    except Exception as e:
        # Log error but don't fail the import
        # Make sure we log the full error trace to help debug issues
        app = adsk.core.Application.get()
        error_msg = f"Error in _align_airfoil_with_chord: {str(e)}\n{traceback.format_exc()}"
        app.log(error_msg)
        # Re-raise to see if there's a pattern, but for now just log it
