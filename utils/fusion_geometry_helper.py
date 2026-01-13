import adsk.core, adsk.fusion
import numpy as np
import os
import tempfile
import traceback
import math
from utils import dxf_exporter

def create_fusion_spline(sketch, control_points, knots, degree, is_closed=False):
    """
    Creates a high-accuracy NURBS spline in a Fusion sketch.
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
        error_log = []
        def log_error(msg):
            error_log.append(msg)
            app.log(msg)
        
        doc = dxf_exporter.export_transformed_bspline_to_dxf(
            upper_cp, upper_knots, upper_degree,
            lower_cp, lower_knots, lower_degree,
            is_sharp_te, log_error
        )
        
        if not doc:
            error_msg = "Failed to create DXF document"
            if error_log:
                error_msg += f": {'; '.join(error_log)}"
            raise RuntimeError(error_msg)
            
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
                app.log(f"Error getting spline {i}: {e}")
            
        # Also get any lines (e.g., trailing edge connector for blunt TE)
        all_lines = []
        for line in sketch.sketchCurves.sketchLines:
            all_lines.append(line)
        
        if len(all_splines) < 2:
            return  # Need at least upper and lower splines

        # app.log(f"Chord start at ({chord_start_sketch.x:.2f}, {chord_start_sketch.y:.2f})")
        # app.log(f"Chord end at ({chord_end_sketch.x:.2f}, {chord_end_sketch.y:.2f})")

        # Determine which end is LE by checking control point 0 location
        # In airfoil space, CP[0] is always at the LE (x=0)
        # Check if the first control point is at the start or end of the spline
        first_spline = all_splines[0]

        # Access first control point - controlPoints is iterable
        cp0 = None
        for i, cp in enumerate(first_spline.controlPoints):
            if i == 0:
                cp0 = cp
                break

        if not cp0:
            app.log("Warning: Could not access control point 0")
            return

        cp0_geometry = cp0.geometry

        # Compare CP[0] with start and end sketch points
        dist_cp0_to_start = cp0_geometry.distanceTo(first_spline.startSketchPoint.geometry)
        dist_cp0_to_end = cp0_geometry.distanceTo(first_spline.endSketchPoint.geometry)

        # app.log(f"CP[0] at ({cp0_geometry.x:.2f}, {cp0_geometry.y:.2f})")
        # app.log(f"CP[0] distance to start: {dist_cp0_to_start:.4f}, to end: {dist_cp0_to_end:.4f}")

        # Get start and end points from all splines for midpoint calculation
        start_points = []
        end_points = []
        for spline in all_splines:
            start_points.append(spline.startSketchPoint.geometry)
            end_points.append(spline.endSketchPoint.geometry)

        start_midpoint = adsk.core.Point3D.create(
            sum(p.x for p in start_points) / len(start_points),
            sum(p.y for p in start_points) / len(start_points),
            sum(p.z for p in start_points) / len(start_points)
        )
        end_midpoint = adsk.core.Point3D.create(
            sum(p.x for p in end_points) / len(end_points),
            sum(p.y for p in end_points) / len(end_points),
            sum(p.z for p in end_points) / len(end_points)
        )

        # CP[0] is at the LE, so if it's closer to start, then start is LE
        if dist_cp0_to_start < dist_cp0_to_end:
            le_point_sketch = start_midpoint
            te_point_sketch = end_midpoint
            # app.log("CP[0] is at start -> Start is LE")
        else:
            le_point_sketch = end_midpoint
            te_point_sketch = start_midpoint
            # app.log("CP[0] is at end -> End is LE")

        # Calculate the imported airfoil's LE-to-TE vector (its current orientation)
        imported_airfoil_vec = adsk.core.Vector3D.create(
            te_point_sketch.x - le_point_sketch.x,
            te_point_sketch.y - le_point_sketch.y,
            te_point_sketch.z - le_point_sketch.z
        )

        # Calculate the desired chord vector (target orientation)
        chord_vec_sketch = adsk.core.Vector3D.create(
            chord_end_sketch.x - chord_start_sketch.x,
            chord_end_sketch.y - chord_start_sketch.y,
            chord_end_sketch.z - chord_start_sketch.z
        )

        # Check if vectors are valid
        if imported_airfoil_vec.length < 1e-10 or chord_vec_sketch.length < 1e-10:
            app.log("Warning: Cannot calculate rotation angle - invalid vectors")
            return

        # Calculate the angle between the imported airfoil vector and the chord vector
        # Use atan2 for proper quadrant handling
        imported_angle = math.atan2(imported_airfoil_vec.y, imported_airfoil_vec.x)
        chord_angle = math.atan2(chord_vec_sketch.y, chord_vec_sketch.x)
        rotation_angle = chord_angle - imported_angle

        # Normalize angle to [-π, π]
        while rotation_angle > math.pi:
            rotation_angle -= 2 * math.pi
        while rotation_angle < -math.pi:
            rotation_angle += 2 * math.pi

        # app.log(f"Imported airfoil angle: {math.degrees(imported_angle):.2f}°")
        # app.log(f"Target chord angle: {math.degrees(chord_angle):.2f}°")
        # app.log(f"Rotation needed: {math.degrees(rotation_angle):.2f}°")

        # Build transformation matrix directly
        # We need to map: le_point_sketch -> chord_start_sketch with the correct rotation

        transform = adsk.core.Matrix3D.create()

        # Calculate rotation components
        cos_angle = math.cos(rotation_angle)
        sin_angle = math.sin(rotation_angle)

        # Build the 2D rotation + translation matrix directly
        # For a point (x, y), the transformation is:
        # x' = cos(θ)(x - le_x) - sin(θ)(y - le_y) + chord_start_x
        # y' = sin(θ)(x - le_x) + cos(θ)(y - le_y) + chord_start_y

        # This can be written as a 4x4 homogeneous transformation matrix:
        # [ cos -sin  0  tx ]
        # [ sin  cos  0  ty ]
        # [  0    0   1  0  ]
        # [  0    0   0  1  ]
        # where tx and ty include both the rotation offset and final translation

        # Translation components that include rotation center offset
        tx = chord_start_sketch.x - (cos_angle * le_point_sketch.x - sin_angle * le_point_sketch.y)
        ty = chord_start_sketch.y - (sin_angle * le_point_sketch.x + cos_angle * le_point_sketch.y)

        # Set matrix components (row, column)
        transform.setCell(0, 0, cos_angle)
        transform.setCell(0, 1, -sin_angle)
        transform.setCell(0, 3, tx)

        transform.setCell(1, 0, sin_angle)
        transform.setCell(1, 1, cos_angle)
        transform.setCell(1, 3, ty)

        transform.setCell(2, 2, 1.0)  # Z remains unchanged
        transform.setCell(3, 3, 1.0)  # Homogeneous coordinate

        # app.log(f"Applying single transformation: rotate {math.degrees(rotation_angle):.2f}° + translate to chord start")

        # Apply the combined transformation in one operation
        entities = adsk.core.ObjectCollection.create()
        for spline in all_splines:
            entities.add(spline)
        for line in all_lines:
            entities.add(line)

        try:
            sketch.move(entities, transform)

            # Verification: Check if alignment is correct after rotation and translation
            # Re-collect endpoint positions after transformation
            start_points_after = []
            end_points_after = []
            for spline in all_splines:
                start_points_after.append(spline.startSketchPoint.geometry)
                end_points_after.append(spline.endSketchPoint.geometry)

            start_midpoint_after = adsk.core.Point3D.create(
                sum(p.x for p in start_points_after) / len(start_points_after),
                sum(p.y for p in start_points_after) / len(start_points_after),
                sum(p.z for p in start_points_after) / len(start_points_after)
            )
            end_midpoint_after = adsk.core.Point3D.create(
                sum(p.x for p in end_points_after) / len(end_points_after),
                sum(p.y for p in end_points_after) / len(end_points_after),
                sum(p.z for p in end_points_after) / len(end_points_after)
            )

            # app.log(f"After rotation+translation:")
            # app.log(f"  Start midpoint at ({start_midpoint_after.x:.2f}, {start_midpoint_after.y:.2f})")
            # app.log(f"  End midpoint at ({end_midpoint_after.x:.2f}, {end_midpoint_after.y:.2f})")
            # app.log(f"  Chord start at ({chord_start_sketch.x:.2f}, {chord_start_sketch.y:.2f})")
            # app.log(f"  Chord end at ({chord_end_sketch.x:.2f}, {chord_end_sketch.y:.2f})")

            # Re-identify which end is LE after rotation (should be closer to chord start)
            dist_start_to_chord_start = start_midpoint_after.distanceTo(chord_start_sketch)
            dist_end_to_chord_start = end_midpoint_after.distanceTo(chord_start_sketch)
            dist_start_to_chord_end = start_midpoint_after.distanceTo(chord_end_sketch)
            dist_end_to_chord_end = end_midpoint_after.distanceTo(chord_end_sketch)

            # app.log(f"  Start to chord start: {dist_start_to_chord_start:.4f}, to chord end: {dist_start_to_chord_end:.4f}")
            # app.log(f"  End to chord start: {dist_end_to_chord_start:.4f}, to chord end: {dist_end_to_chord_end:.4f}")

            # Determine which end is currently closer to chord start (that's the current LE position)
            if dist_start_to_chord_start < dist_end_to_chord_start:
                current_le = start_midpoint_after
                current_te = end_midpoint_after
                # app.log(f"  Current LE is at start midpoint")
            else:
                current_le = end_midpoint_after
                current_te = start_midpoint_after
                # app.log(f"  Current LE is at end midpoint")

            # Check if LE is closer to chord end than chord start - that means it's backwards
            dist_le_to_chord_start = current_le.distanceTo(chord_start_sketch)
            dist_le_to_chord_end = current_le.distanceTo(chord_end_sketch)

            # app.log(f"  LE to chord start: {dist_le_to_chord_start:.4f}, to chord end: {dist_le_to_chord_end:.4f}")

            # If LE is closer to chord end, airfoil is backwards
            if dist_le_to_chord_end < dist_le_to_chord_start:
                # app.log("Verification failed: Airfoil is backwards (LE closer to chord end), applying 180° correction")
                correction_transform = adsk.core.Matrix3D.create()
                zAxis = adsk.core.Vector3D.create(0, 0, 1)
                # Rotate 180 degrees around the chord midpoint
                chord_midpoint = adsk.core.Point3D.create(
                    (chord_start_sketch.x + chord_end_sketch.x) / 2.0,
                    (chord_start_sketch.y + chord_end_sketch.y) / 2.0,
                    (chord_start_sketch.z + chord_end_sketch.z) / 2.0
                )
                correction_transform.setToRotation(math.pi, zAxis, chord_midpoint)
                sketch.move(entities, correction_transform)

                # # Log final positions after correction
                # start_points_final = []
                # end_points_final = []
                # for spline in all_splines:
                #     start_points_final.append(spline.startSketchPoint.geometry)
                #     end_points_final.append(spline.endSketchPoint.geometry)
                # start_midpoint_final = adsk.core.Point3D.create(
                #     sum(p.x for p in start_points_final) / len(start_points_final),
                #     sum(p.y for p in start_points_final) / len(start_points_final),
                #     sum(p.z for p in start_points_final) / len(start_points_final)
                # )
                # end_midpoint_final = adsk.core.Point3D.create(
                #     sum(p.x for p in end_points_final) / len(end_points_final),
                #     sum(p.y for p in end_points_final) / len(end_points_final),
                #     sum(p.z for p in end_points_final) / len(end_points_final)
                # )
                # app.log(f"After 180° correction:")
                # app.log(f"  Start midpoint at ({start_midpoint_final.x:.2f}, {start_midpoint_final.y:.2f})")
                # app.log(f"  End midpoint at ({end_midpoint_final.x:.2f}, {end_midpoint_final.y:.2f})")
            # else:
                # app.log("Verification passed: Alignment correct")

        except Exception as e:
            app.log(f"Error moving spline: {e}")

        
    except Exception as e:
        # Log error but don't fail the import
        # Make sure we log the full error trace to help debug issues
        app = adsk.core.Application.get()
        error_msg = f"Error in _align_airfoil_with_chord: {str(e)}\n{traceback.format_exc()}"
        app.log(error_msg)
        # Re-raise to see if there's a pattern, but for now just log it
