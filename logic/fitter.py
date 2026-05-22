import adsk.core, adsk.fusion
import os
import math
import traceback
import numpy as np
from scipy import interpolate
from logic import state
from core import config
from core.airfoil_processor import AirfoilProcessor
from core.bspline_processor import BSplineProcessor
from utils.fusion_geometry_helper import create_fusion_spline, import_splines_via_dxf
from utils import bspline_helper
from logic.preview_renderer import render_preview
from utils.i18n import t


def run_fitter(inputs, is_preview):
    """Core logic for fitting and geometry generation."""
    app = adsk.core.Application.get()
    
    # Cleanup old preview graphics before recalculating
    if state.preview_graphics:
        try:
            state.preview_graphics.deleteMe()
        except:
            pass
        state.preview_graphics = None
    
    try:
        # 1. Get Selection and Environment
        line_select = inputs.itemById('chord_line')
        if line_select.selectionCount == 0:
            return False
        
        # Check if file is selected
        file_path_input = inputs.itemById('file_path')
        if not file_path_input or not file_path_input.value:
            return False
        
        selected_line = adsk.fusion.SketchLine.cast(line_select.selection(0).entity)
        if not selected_line:
            return False
            
        # Save original chord line endpoints for alignment (before any flip transformation)
        chord_start_world_original = selected_line.startSketchPoint.worldGeometry
        chord_end_world_original = selected_line.endSketchPoint.worldGeometry
        
        start_pt_world = selected_line.startSketchPoint.worldGeometry
        end_pt_world = selected_line.endSketchPoint.worldGeometry
        chord_vec_world = adsk.core.Vector3D.create(end_pt_world.x - start_pt_world.x, 
                                                   end_pt_world.y - start_pt_world.y, 
                                                   end_pt_world.z - start_pt_world.z)
        chord_length = chord_vec_world.length
        
        # Build 3D transformation matrix
        theta = state.rotation_state * (math.pi / 2.0)
        sketch = selected_line.parentSketch
        mat = sketch.transform
        if sketch.assemblyContext:
            mat.transformBy(sketch.assemblyContext.transform2)
            
        sketch_normal_world = adsk.core.Vector3D.create(mat.getCell(0, 2), mat.getCell(1, 2), mat.getCell(2, 2))
        sketch_normal_world.normalize()
        
        x_axis_world = chord_vec_world.copy()
        x_axis_world.normalize()
        
        # Calculate y_axis_in_plane before flipping (to preserve orientation)
        y_axis_in_plane = sketch_normal_world.crossProduct(x_axis_world)
        y_axis_in_plane.normalize()
        
        # Apply flip orientation: reverse direction along chord line (nose to tail)
        if state.flip_orientation:
            x_axis_world.scaleBy(-1.0)
            # When flipped, start from the end point instead
            start_pt_world = end_pt_world
            # Keep y_axis direction the same (don't let cross product flip it)
            # y_axis_in_plane stays as calculated above
        
        y_axis_world = y_axis_in_plane.copy()
        y_axis_world.scaleBy(math.cos(theta))
        z_part = sketch_normal_world.copy()
        z_part.scaleBy(math.sin(theta))
        y_axis_world.add(z_part)
        
        z_axis_world = sketch_normal_world.copy()
        z_axis_world.scaleBy(math.cos(theta))
        y_neg_part = y_axis_in_plane.copy()
        y_neg_part.scaleBy(-math.sin(theta))
        z_axis_world.add(y_neg_part)
        
        airfoil_to_world = adsk.core.Matrix3D.create()
        airfoil_to_world.setWithCoordinateSystem(start_pt_world, x_axis_world, y_axis_world, z_axis_world)

        # 2. Fitting Logic
        do_new_fit = (state.needs_refit or not state.fit_cache) if is_preview else True
        if is_preview:
            state.needs_refit = False

        # Save previous TE state before potential refit (for edge case: CP change after TE adjustment)
        prev_te_applied = state.fit_cache.get('te_applied', False) if state.fit_cache else False
        prev_te_value = state.fit_cache.get('te_value', 0.0) if state.fit_cache else 0.0

        def calc_max_err(curve, data, exponent):
            """Calculate maximum error against the displayed normalized input."""
            if not curve:
                return 0.0, np.array([0.0, 0.0])

            _, max_error, max_error_idx, _ = bspline_helper.calculate_bspline_fitting_error(
                curve, data, param_exponent=exponent, return_max_error=True
            )
            return max_error, data[max_error_idx].copy()

        def update_cache_errors_from_curves(upper_curve, lower_curve, cache):
            err_u, max_err_pt_u = calc_max_err(
                upper_curve,
                cache['raw_upper'],
                cache.get('param_exponent_upper', 0.5),
            )
            err_l, max_err_pt_l = calc_max_err(
                lower_curve,
                cache['raw_lower'],
                cache.get('param_exponent_lower', 0.5),
            )
            cache['err_u'] = err_u
            cache['err_l'] = err_l
            cache['max_err_pt_u'] = max_err_pt_u
            cache['max_err_pt_l'] = max_err_pt_l

        if do_new_fit:
            file_path = inputs.itemById('file_path').value
            if not file_path or not os.path.exists(file_path):
                return False

            # Get continuity level from dropdown
            continuity_dropdown = inputs.itemById('continuity_level')
            enforce_g2 = False
            enforce_g3 = False
            if continuity_dropdown:
                selected_item = continuity_dropdown.selectedItem
                if selected_item:
                    if selected_item.name == 'G2':
                        enforce_g2 = True
                        enforce_g3 = False
                    elif selected_item.name == 'G3':
                        enforce_g2 = True  # G3 requires G2
                        enforce_g3 = True
                    # G1: enforce_g2 = False, enforce_g3 = False (already set)
            
            smoothness = inputs.itemById('smoothness_input').valueOne
            
            processor = AirfoilProcessor(logger_func=lambda msg: None)
            if not processor.load_airfoil_data_and_initialize_model(file_path):
                app.userInterface.messageBox(t("failed_load_airfoil_data"))
                return False
            error_upper_data, error_lower_data = processor.error_reference_data()
            
            # Determine operation type based on state
            is_initial_fit = (state.current_cp_count_upper is None and state.current_cp_count_lower is None)

            # Set TE thickness input to match the newly loaded airfoil on initial load.
            # This must also write zero so a previous file's TE value cannot leak.
            if is_initial_fit:
                te_input = inputs.itemById('te_thickness')
                if te_input:
                    te_input.value = processor.get_te_thickness() * chord_length

            cp_count_upper = config.DEFAULT_CP_COUNT if is_initial_fit else state.current_cp_count_upper
            cp_count_lower = config.DEFAULT_CP_COUNT if is_initial_fit else state.current_cp_count_lower

            bspline = BSplineProcessor()
                
            if is_initial_fit:
                # Initial fit: use fit_bspline with actual UI values
                bspline.smoothing_weight = smoothness
                
                success = bspline.fit_bspline(
                    processor.upper_data, processor.lower_data,
                    num_control_points=(cp_count_upper, cp_count_lower),
                    is_thickened=processor.is_trailing_edge_thickened(),
                    upper_te_tangent_vector=processor.upper_te_tangent_vector,
                    lower_te_tangent_vector=processor.lower_te_tangent_vector,
                    enforce_g2=enforce_g2, enforce_g3=enforce_g3,
                    single_span=True
                )
                if not success: 
                    app.userInterface.messageBox(t("failed_fit_airfoil"))
                    return False
                
                # Store processor and CP count in state
                state.current_cp_count_upper = cp_count_upper
                state.current_cp_count_lower = cp_count_lower
                
            else:
                # Refinement: add or remove control points
                bspline.smoothing_weight = smoothness
                bspline.enforce_g2 = enforce_g2
                bspline.enforce_g3 = enforce_g3 if enforce_g2 else False
                
                current_cp_upper = state.current_cp_count_upper
                current_cp_lower = state.current_cp_count_lower
                cp_diff_upper = cp_count_upper - current_cp_upper
                cp_diff_lower = cp_count_lower - current_cp_lower
                
                # Save current state to fit_cache before refinement (so we can restore when removing)
                # This preserves the state before we modify it
                if bspline.is_fitted():
                    state.fit_cache['upper_cp_raw'] = bspline.upper_control_points.copy()
                    state.fit_cache['lower_cp_raw'] = bspline.lower_control_points.copy()
                    state.fit_cache['upper_knots'] = bspline.upper_knot_vector.copy() if bspline.upper_knot_vector is not None else None
                    state.fit_cache['lower_knots'] = bspline.lower_knot_vector.copy() if bspline.lower_knot_vector is not None else None
                    state.fit_cache['degree_u'] = bspline.degree_upper
                    state.fit_cache['degree_l'] = bspline.degree_lower
                    state.fit_cache['is_sharp'] = bspline.is_sharp_te
                
                def add_control_points(cp_diff, surface):
                    # Adding control points: insert knots at max error locations
                    for i in range(cp_diff):
                        success = bspline.insert_knot_at_max_error(surface, single_span=True)
                        if not success:
                            app.userInterface.messageBox(t("failed_insert_knot", surface=surface))
                            return False
                
                def remove_control_points(cp_diff, surface):
                    # Removing control points: re-fit with new desired count for the changed surface
                    # Keep the other surface's current count unchanged
                    if surface == 'upper':
                        target_upper = cp_count_upper  # New desired count
                        target_lower = current_cp_lower  # Keep current
                    else:  # lower
                        target_upper = current_cp_upper  # Keep current
                        target_lower = cp_count_lower  # New desired count
                    
                    success = bspline.fit_bspline(
                        processor.upper_data, processor.lower_data,
                        num_control_points=(target_upper, target_lower),
                        is_thickened=processor.is_trailing_edge_thickened(),
                        upper_te_tangent_vector=processor.upper_te_tangent_vector,
                        lower_te_tangent_vector=processor.lower_te_tangent_vector,
                        enforce_g2=enforce_g2, enforce_g3=enforce_g3,
                        single_span=True
                    )
                    if not success:
                        app.userInterface.messageBox(t("failed_refit_surface", surface=surface))
                        return False
                    # Update state for the changed surface
                    if surface == 'upper':
                        state.current_cp_count_upper = cp_count_upper
                    else:
                        state.current_cp_count_lower = cp_count_lower
                    return True
                
                # Handle upper surface changes
                if cp_diff_upper > 0:
                    add_control_points(cp_diff_upper, 'upper')
                    state.current_cp_count_upper = cp_count_upper
                elif cp_diff_upper < 0:
                    remove_control_points(cp_diff_upper, 'upper')
                
                # Handle lower surface changes
                if cp_diff_lower > 0:
                    add_control_points(cp_diff_lower, 'lower')
                    state.current_cp_count_lower = cp_count_lower
                elif cp_diff_lower < 0:
                    remove_control_points(cp_diff_lower, 'lower')
                
                # If both counts are unchanged but other parameters changed, re-fit
                if cp_diff_upper == 0 and cp_diff_lower == 0:
                    bspline.fit_bspline(
                        processor.upper_data, processor.lower_data,
                        num_control_points=(cp_count_upper, cp_count_lower),
                        is_thickened=processor.is_trailing_edge_thickened(),
                        upper_te_tangent_vector=processor.upper_te_tangent_vector,
                        lower_te_tangent_vector=processor.lower_te_tangent_vector,
                        enforce_g2=enforce_g2, enforce_g3=enforce_g3,
                        single_span=True
                    )
                
                # Update state (only if not already updated in remove_control_points)
                if cp_diff_upper >= 0:
                    state.current_cp_count_upper = cp_count_upper
                if cp_diff_lower >= 0:
                    state.current_cp_count_lower = cp_count_lower
                
            err_u, max_err_pt_u = calc_max_err(
                bspline.upper_curve,
                error_upper_data,
                bspline.param_exponent_upper,
            )
            err_l, max_err_pt_l = calc_max_err(
                bspline.lower_curve,
                error_lower_data,
                bspline.param_exponent_lower,
            )
            
            state.fit_cache = {
                'upper_cp_raw': bspline.upper_control_points.copy(),
                'lower_cp_raw': bspline.lower_control_points.copy(),
                'upper_knots': bspline.upper_knot_vector, 'lower_knots': bspline.lower_knot_vector,
                'degree_u': bspline.degree_upper, 'degree_l': bspline.degree_lower,
                'is_sharp': bspline.is_sharp_te,
                'err_u': err_u, 'err_l': err_l,
                'max_err_pt_u': max_err_pt_u, 'max_err_pt_l': max_err_pt_l,  # Store coordinates of max deviation points
                'raw_upper': error_upper_data.copy(), 'raw_lower': error_lower_data.copy(),
                'fit_upper': processor.upper_data.copy(), 'fit_lower': processor.lower_data.copy(),
                'param_exponent_upper': bspline.param_exponent_upper,
                'param_exponent_lower': bspline.param_exponent_lower,
                'te_applied': False,
                'te_value': processor.get_te_thickness(),  # Original TE thickness (normalized)
                'enforce_g2': enforce_g2,
                'enforce_g3': enforce_g3
            }

            # Edge case: Reapply TE thickness after CP count change
            if prev_te_applied:
                bspline.te_thickness_normalized = prev_te_value
                bspline.upper_original_data = state.fit_cache['fit_upper'].copy()
                bspline.lower_original_data = state.fit_cache['fit_lower'].copy()
                if bspline.apply_te_thickening_parametric():
                    state.fit_cache['upper_cp_raw'] = bspline.upper_control_points.copy()
                    state.fit_cache['lower_cp_raw'] = bspline.lower_control_points.copy()
                    state.fit_cache['upper_knots'] = bspline.upper_knot_vector
                    state.fit_cache['lower_knots'] = bspline.lower_knot_vector
                    state.fit_cache['degree_u'] = bspline.degree_upper
                    state.fit_cache['degree_l'] = bspline.degree_lower
                    state.fit_cache['is_sharp'] = bspline.is_sharp_te
                    state.fit_cache['te_applied'] = True
                    state.fit_cache['te_value'] = prev_te_value
                    state.fit_cache['param_exponent_upper'] = bspline.param_exponent_upper
                    state.fit_cache['param_exponent_lower'] = bspline.param_exponent_lower
                    update_cache_errors_from_curves(
                        bspline.upper_curve,
                        bspline.lower_curve,
                        state.fit_cache,
                    )

        # 3. Post-Processing - Apply TE thickening parametrically
        te_thickness = inputs.itemById('te_thickness').value
        te_thickness_normalized = te_thickness / chord_length

        cached_te_value = state.fit_cache.get('te_value')
        te_changed = (
            cached_te_value is None or
            abs(cached_te_value - te_thickness_normalized) > 1e-9
        )

        if te_changed and not do_new_fit:
            # Reconstruct processor state for TE thickening
            bspline_te = BSplineProcessor()
            fit_upper = state.fit_cache.get('fit_upper', state.fit_cache['raw_upper'])
            fit_lower = state.fit_cache.get('fit_lower', state.fit_cache['raw_lower'])
            bspline_te.upper_original_data = fit_upper.copy()
            bspline_te.lower_original_data = fit_lower.copy()
            bspline_te.num_cp_upper = state.current_cp_count_upper
            bspline_te.num_cp_lower = state.current_cp_count_lower
            bspline_te.enforce_g2 = state.fit_cache.get('enforce_g2', False)
            bspline_te.enforce_g3 = state.fit_cache.get('enforce_g3', False)
            bspline_te.smoothing_weight = inputs.itemById('smoothness_input').valueOne
            bspline_te.fitted = True

            # Apply TE thickness
            bspline_te.te_thickness_normalized = te_thickness_normalized
            if bspline_te.apply_te_thickening_parametric():
                # Update cache
                state.fit_cache['upper_cp_raw'] = bspline_te.upper_control_points.copy()
                state.fit_cache['lower_cp_raw'] = bspline_te.lower_control_points.copy()
                state.fit_cache['upper_knots'] = bspline_te.upper_knot_vector
                state.fit_cache['lower_knots'] = bspline_te.lower_knot_vector
                state.fit_cache['degree_u'] = bspline_te.degree_upper
                state.fit_cache['degree_l'] = bspline_te.degree_lower
                state.fit_cache['is_sharp'] = bspline_te.is_sharp_te
                state.fit_cache['te_applied'] = True
                state.fit_cache['te_value'] = te_thickness_normalized
                state.fit_cache['param_exponent_upper'] = bspline_te.param_exponent_upper
                state.fit_cache['param_exponent_lower'] = bspline_te.param_exponent_lower
                update_cache_errors_from_curves(
                    bspline_te.upper_curve,
                    bspline_te.lower_curve,
                    state.fit_cache,
                )

        upper_cp = state.fit_cache['upper_cp_raw'].copy()
        lower_cp = state.fit_cache['lower_cp_raw'].copy()
        is_sharp = state.fit_cache['is_sharp']

        # 4. Update UI Status
        design = adsk.fusion.Design.cast(app.activeProduct)
        units_mgr = design.unitsManager
        def_units = units_mgr.defaultLengthUnits
        decimals = 2 if 'in' not in def_units else 4
       
        # 5. Render Geometry
        def transform_pts(pts, target):
            is_sketch = hasattr(target, 'modelToSketchSpace')
            transformed = []
            for pt in pts:
                p_world = adsk.core.Point3D.create(pt[0] * chord_length, pt[1] * chord_length, 0)
                p_world.transformBy(airfoil_to_world)
                if is_sketch:
                    p_local = target.modelToSketchSpace(p_world)
                else:
                    # Handle ConstructionPlane using its transform property
                    plane_transform = target.transform.copy()
                    plane_transform.invert()
                    p_world.transformBy(plane_transform)
                    p_local = p_world
                transformed.append([p_local.x, p_local.y, p_local.z])
            return np.array(transformed)

        is_editable = inputs.itemById('editable_splines').value
        
        if is_preview:
            target_sketch = selected_line.parentSketch
            target_sketch.is3D = True
            create_fusion_spline(target_sketch, transform_pts(upper_cp, target_sketch), state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
            create_fusion_spline(target_sketch, transform_pts(lower_cp, target_sketch), state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
            
            # Render all preview graphics
            render_preview(
                target_sketch, upper_cp, lower_cp, state.fit_cache,
                chord_length, airfoil_to_world, y_axis_world,
                transform_pts, inputs, is_sharp
            )
        else:
            file_path = inputs.itemById('file_path').value
            sketch_name = os.path.splitext(os.path.basename(file_path))[0] if file_path else "Fitted Airfoil"
            parent_comp = selected_line.parentSketch.parentComponent
            
            def add_sketch_on_plane(target_plane_obj):
                sketches = parent_comp.sketches
                previous_occurrence = None
                try:
                    if hasattr(sketches, 'occurrenceForCreation'):
                        previous_occurrence = sketches.occurrenceForCreation
                        sketches.occurrenceForCreation = selected_line.parentSketch.assemblyContext
                    return sketches.add(target_plane_obj)
                finally:
                    if hasattr(sketches, 'occurrenceForCreation'):
                        sketches.occurrenceForCreation = previous_occurrence

            def warn_missing_reference_plane():
                app.log(
                    "Warning: Selected sketch has lost its reference plane. "
                    "Please repair the timeline or redefine the sketch plane."
                )
                app.userInterface.messageBox(t("missing_sketch_reference_plane"))
                return False

            def get_reference_plane_or_warn(sketch_obj):
                try:
                    reference_plane = sketch_obj.referencePlane
                except RuntimeError as ex:
                    app.log(
                        "Warning: Failed to resolve sketch reference plane. "
                        f"Fusion reported: {ex}"
                    )
                    return None
                except Exception as ex:
                    app.log(
                        "Warning: Unexpected error while resolving sketch reference plane. "
                        f"{ex}"
                    )
                    return None

                return reference_plane if reference_plane else None

            fallback_to_source_sketch = False
            target_plane = None

            if state.rotation_state == 0:
                if is_editable:
                    target_plane = get_reference_plane_or_warn(selected_line.parentSketch)
                    if not target_plane:
                        return warn_missing_reference_plane()
                else:
                    fallback_to_source_sketch = True
            else:
                root_comp = design.rootComponent
                tol = 1e-6
                standard_plane = None
                point_on_plane = start_pt_world

                # A rotated plane only coincides with an origin plane if both its normal
                # and its offset from the origin match that plane.
                if abs(abs(z_axis_world.z) - 1.0) < tol and abs(point_on_plane.z) < tol:
                    standard_plane = root_comp.xYConstructionPlane
                elif abs(abs(z_axis_world.y) - 1.0) < tol and abs(point_on_plane.y) < tol:
                    standard_plane = root_comp.xZConstructionPlane
                elif abs(abs(z_axis_world.x) - 1.0) < tol and abs(point_on_plane.x) < tol:
                    standard_plane = root_comp.yZConstructionPlane

                if standard_plane:
                    target_plane = standard_plane
                else:
                    reference_plane = get_reference_plane_or_warn(selected_line.parentSketch)
                    if not reference_plane:
                        return warn_missing_reference_plane()

                    plane_input = parent_comp.constructionPlanes.createInput()
                    if selected_line.assemblyContext:
                        plane_input.occurrenceForCreation = selected_line.assemblyContext
                    plane_input.setByAngle(selected_line, adsk.core.ValueInput.createByReal(-theta), reference_plane)
                    target_plane = parent_comp.constructionPlanes.add(plane_input)
                    try:
                        target_plane.isLightBulbOn = False
                    except Exception:
                        pass
                    target_plane.name = sketch_name
                                                                               
            if fallback_to_source_sketch:
                target_sketch = selected_line.parentSketch
                target_sketch.is3D = True
                u_final = transform_pts(upper_cp, target_sketch)
                l_final = transform_pts(lower_cp, target_sketch)
                create_fusion_spline(target_sketch, u_final, state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
                create_fusion_spline(target_sketch, l_final, state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
                if not is_sharp:
                    target_sketch.sketchCurves.sketchLines.addByTwoPoints(
                        adsk.core.Point3D.create(u_final[-1,0], u_final[-1,1], 0),
                        adsk.core.Point3D.create(l_final[-1,0], l_final[-1,1], 0)
                    )
            elif is_editable:
                # Create a temporary sketch on the target plane to use modelToSketchSpace for accurate transformation
                temp_sketch = add_sketch_on_plane(target_plane)
                u_dxf = transform_pts(upper_cp, temp_sketch); l_dxf = transform_pts(lower_cp, temp_sketch)
                # If airfoil is flipped, swap the chord start and end points
                if state.flip_orientation:
                    chord_start_aligned = chord_end_world_original
                    chord_end_aligned = chord_start_world_original
                else:
                    chord_start_aligned = chord_start_world_original
                    chord_end_aligned = chord_end_world_original
                target_sketch = import_splines_via_dxf(
                    temp_sketch, target_plane, u_dxf, state.fit_cache['upper_knots'], state.fit_cache['degree_u'], 
                    l_dxf, state.fit_cache['lower_knots'], state.fit_cache['degree_l'], is_sharp,
                    chord_start_aligned, chord_end_aligned, sketch_name=sketch_name
                )
                if temp_sketch != target_sketch: temp_sketch.deleteMe()
            else:
                target_sketch = add_sketch_on_plane(target_plane); target_sketch.name = sketch_name
                u_final = transform_pts(upper_cp, target_sketch); l_final = transform_pts(lower_cp, target_sketch)
                create_fusion_spline(target_sketch, u_final, state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
                create_fusion_spline(target_sketch, l_final, state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
                if not is_sharp:
                    target_sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(u_final[-1,0], u_final[-1,1], 0), adsk.core.Point3D.create(l_final[-1,0], l_final[-1,1], 0))
        
                    
        return True
    except:
        app.log(f"Error: {traceback.format_exc()}")
        app.userInterface.messageBox(t("generic_error"))
        return False
