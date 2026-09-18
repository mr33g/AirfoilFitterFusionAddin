import adsk.core, adsk.fusion
import os
import tempfile
import traceback
import math
from utils import dxf_exporter
from utils.sketch_plane_helper import with_sketch_support

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

def import_splines_via_dxf(sketch, target_plane, upper_cp, upper_knots, upper_degree, 
                           lower_cp, lower_knots, lower_degree, is_sharp_te, 
                           chord_start_world, chord_end_world, sketch_name="Fitted Airfoil",
                           source_sketch=None):
    """
    Imports splines via a temporary DXF file to ensure they are editable control-point splines.
    After import, aligns the airfoil with the chord line by moving the leading edge to the
    chord start point and rotating if needed to align the trailing edge with the chord end point.
    
    Args:
        sketch: Temporary sketch used for coordinate conversion and component lookup
        target_plane: Planar entity to create the imported sketch on
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
        def import_on(candidate):
            options = import_manager.createDXF2DImportOptions(dxf_path, candidate)
            if hasattr(options, 'isCreateControlPointSplines'):
                options.isCreateControlPointSplines = True
            else:
                options.isFitPointsToSplines = False
            if not import_manager.importToTarget(options, sketch.parentComponent):
                raise RuntimeError("Fusion rejected the DXF import.")
            return options

        count_before = sketch.parentComponent.sketches.count
        dxf_options, _ = with_sketch_support(
            source_sketch if source_sketch is not None else sketch,
            target_plane, sketch_name, import_on,
            can_retry=lambda: sketch.parentComponent.sketches.count == count_before)
        results = dxf_options.results
        if not results or results.count != 1:
            raise RuntimeError("DXF import must create exactly one airfoil sketch.")
        new_sketch = results.item(0)
        new_sketch.name = sketch_name
        curves = new_sketch.sketchCurves
        if (curves.sketchControlPointSplines.count != 2
                or curves.sketchFittedSplines.count or curves.sketchFixedSplines.count):
            raise RuntimeError("DXF import did not create exactly two adjustable splines.")
        for spline in curves.sketchControlPointSplines:
            spline.isControlPolygonVisible = True

        # Keep movement native; only coordinate conversion needs the occurrence.
        coordinate_sketch = new_sketch
        if sketch.assemblyContext:
            coordinate_sketch = new_sketch.createForAssemblyContext(sketch.assemblyContext)
            if not coordinate_sketch:
                raise RuntimeError("Cannot resolve the imported sketch's assembly context.")

        # Independently express the intended geometry in the actual output sketch.
        # This checks airfoil side/Flip as well as endpoints after placement.
        expected = []
        for surface in (upper_cp, lower_cp):
            expected.append([
                coordinate_sketch.modelToSketchSpace(sketch.sketchToModelSpace(
                    adsk.core.Point3D.create(*point))) for point in surface
            ])
        _align_airfoil_with_chord(
            new_sketch, chord_start_world, chord_end_world, coordinate_sketch, expected)

        # Clean up temp file
        if os.path.exists(dxf_path):
            os.remove(dxf_path)
            
        return new_sketch
    except Exception as e:
        if os.path.exists(dxf_path):
            try: os.remove(dxf_path)
            except: pass
        raise RuntimeError(f"DXF import failed: {str(e)}\n{traceback.format_exc()}")


def _align_airfoil_with_chord(sketch, chord_start_world, chord_end_world,
                             coordinate_sketch=None, expected_control_points=None):
    """Apply one rigid placement, then verify it; never guess a second correction.

    Our DXF exports both surfaces LE-to-TE. Import must preserve that direction.
    Surface collection order is irrelevant. Movement stays on the native sketch;
    the occurrence proxy is used only to convert the world-space chord endpoints.
    """
    if coordinate_sketch is None:
        coordinate_sketch = sketch
    start = coordinate_sketch.modelToSketchSpace(chord_start_world)
    end = coordinate_sketch.modelToSketchSpace(chord_end_world)
    splines = list(sketch.sketchCurves.sketchControlPointSplines)
    if len(splines) != 2 or any(len(s.controlPoints) < 2 for s in splines):
        raise RuntimeError("Alignment requires two editable control-point splines.")

    def midpoint(attribute):
        points = [getattr(spline, attribute).geometry for spline in splines]
        return adsk.core.Point3D.create(
            sum(p.x for p in points) / 2, sum(p.y for p in points) / 2,
            sum(p.z for p in points) / 2)

    leading = midpoint('startSketchPoint')
    trailing = midpoint('endSketchPoint')
    chord_length = start.distanceTo(end)
    # Fusion uses cm: absolute floor 10 nm, plus a relative allowance for size.
    tolerance = max(1e-6, chord_length * 1e-8)
    if chord_length <= tolerance or leading.distanceTo(trailing) <= tolerance:
        raise RuntimeError("Cannot align a zero-length airfoil or chord.")
    if any(abs(p.z) > tolerance for p in (start, end, leading, trailing)):
        raise RuntimeError("Airfoil and chord must lie in the imported sketch plane.")
    if abs(leading.distanceTo(trailing) - chord_length) > tolerance:
        raise RuntimeError("Imported airfoil length does not match the chord.")

    angle = (math.atan2(end.y - start.y, end.x - start.x)
             - math.atan2(trailing.y - leading.y, trailing.x - leading.x))
    c, s = math.cos(angle), math.sin(angle)
    transform = adsk.core.Matrix3D.create()
    transform.setCell(0, 0, c)
    transform.setCell(0, 1, -s)
    transform.setCell(1, 0, s)
    transform.setCell(1, 1, c)
    transform.setCell(0, 3, start.x - (c * leading.x - s * leading.y))
    transform.setCell(1, 3, start.y - (s * leading.x + c * leading.y))
    entities = adsk.core.ObjectCollection.create()
    for entity in splines + list(sketch.sketchCurves.sketchLines):
        entities.add(entity)
    if not sketch.move(entities, transform):
        raise RuntimeError("Fusion could not move the imported airfoil.")
    if (midpoint('startSketchPoint').distanceTo(start) > tolerance
            or midpoint('endSketchPoint').distanceTo(end) > tolerance):
        raise RuntimeError("Imported airfoil endpoints failed alignment verification.")

    trailing_points = [spline.endSketchPoint.geometry for spline in splines]
    # Fusion also exposes spline control-frame segments as sketch lines.
    # They belong to the splines, not to the exported trailing-edge connector.
    frame_lines = [line for spline in splines for line in spline.controlFrameLines]
    for line in sketch.sketchCurves.sketchLines:
        if line in frame_lines:
            continue
        endpoints = [line.startSketchPoint.geometry, line.endSketchPoint.geometry]
        error = min(max(endpoints[i].distanceTo(trailing_points[j])
                        for i, j in enumerate(order)) for order in ((0, 1), (1, 0)))
        if error > tolerance:
            raise RuntimeError(
                "Imported trailing-edge connector failed verification "
                f"(endpoint error={error:.6g} cm, tolerance={tolerance:.6g} cm, "
                f"control-frame lines excluded={len(frame_lines)}).")

    if expected_control_points is not None:
        actual = [[point.geometry for point in spline.controlPoints] for spline in splines]

        def matches(actual_surface, expected_surface):
            return len(actual_surface) == len(expected_surface) and all(
                p.distanceTo(q) <= tolerance for p, q in zip(actual_surface, expected_surface))

        # Fusion may enumerate the upper and lower surfaces in either order.
        if not any(all(matches(actual[i], expected_control_points[j])
                       for i, j in enumerate(order)) for order in ((0, 1), (1, 0))):
            raise RuntimeError("Imported airfoil shape or orientation failed verification.")
