import adsk.core, adsk.fusion
import os
import math
import traceback
import numpy as np
from logic import state
from core.airfoil_processor import AirfoilProcessor
from core.bspline_processor import BSplineProcessor
from utils.fusion_geometry_helper import create_fusion_spline, import_splines_via_dxf
from utils import bspline_helper
from logic.preview_renderer import render_preview


def run_fitter(inputs, is_preview):
    """Core logic for fitting and geometry generation."""
    if is_preview:
        preview_item = inputs.itemById('do_preview')
        if not preview_item or not preview_item.value:
            return True
    
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
        selected_line = adsk.fusion.SketchLine.cast(line_select.selection(0).entity)
        if not selected_line:
            return False
            
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
            mat.transformBy(sketch.assemblyContext.worldTransform)
            
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
            state.needs_preview = False
            state.needs_refit = False

        if do_new_fit:
            file_path = inputs.itemById('file_path').value
            if not file_path or not os.path.exists(file_path):
                return False
                
            cp_count = inputs.itemById('cp_count').value
            
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
            
            enforce_te_tangent = inputs.itemById('enforce_te_tangency').value
            smoothness = inputs.itemById('smoothness_input').valueOne
            
            processor = AirfoilProcessor(logger_func=lambda msg: None)
            if not processor.load_airfoil_data_and_initialize_model(file_path):
                app.userInterface.messageBox('Failed to load airfoil data, please check the file path and try again.')
                return False
                
            bspline = BSplineProcessor()
            bspline.smoothing_weight = smoothness
            
            success = bspline.fit_bspline(
                processor.upper_data, processor.lower_data,
                num_control_points=cp_count,
                is_thickened=processor.is_trailing_edge_thickened(),
                upper_te_tangent_vector=processor.upper_te_tangent_vector,
                lower_te_tangent_vector=processor.lower_te_tangent_vector,
                enforce_g2=enforce_g2, enforce_g3=enforce_g3,
                enforce_te_tangency=enforce_te_tangent, single_span=True
            )
            if not success: 
                app.userInterface.messageBox('Failed to fit airfoil, please check the input parameters and try again.')
                return False
                
            def calc_max_err(curve, data, exponent):
                if not curve: return 0.0, np.array([0.0, 0.0])
                u_params = bspline_helper.create_parameter_from_x_coords(data, exponent)
                diffs = np.linalg.norm(data - curve(u_params), axis=1)
                idx = np.argmax(diffs)
                # Return error value and the data point with max deviation
                # The spline intersection point will be calculated later when drawing
                return diffs[idx], data[idx].copy()  # Return full (x, y) coordinates of data point

            err_u, max_err_pt_u = calc_max_err(bspline.upper_curve, processor.upper_data, bspline.param_exponent_upper)
            err_l, max_err_pt_l = calc_max_err(bspline.lower_curve, processor.lower_data, bspline.param_exponent_lower)
            
            state.fit_cache = {
                'upper_cp_raw': bspline.upper_control_points.copy(),
                'lower_cp_raw': bspline.lower_control_points.copy(),
                'upper_knots': bspline.upper_knot_vector, 'lower_knots': bspline.lower_knot_vector,
                'degree_u': bspline.degree_upper, 'degree_l': bspline.degree_lower,
                'is_sharp': bspline.is_sharp_te,
                'err_u': err_u, 'err_l': err_l,
                'max_err_pt_u': max_err_pt_u, 'max_err_pt_l': max_err_pt_l,  # Store coordinates of max deviation points
                'raw_upper': processor.upper_data, 'raw_lower': processor.lower_data
            }

        # 3. Post-Processing
        te_thickness = inputs.itemById('te_thickness').value
        upper_cp = state.fit_cache['upper_cp_raw'].copy()
        lower_cp = state.fit_cache['lower_cp_raw'].copy()
        is_sharp = state.fit_cache['is_sharp']
        
        if te_thickness > 0:
            half_thick = 0.5 * (te_thickness / chord_length)
            upper_cp[:, 1] += half_thick * bspline_helper.smoothstep_quintic(upper_cp[:, 0])
            lower_cp[:, 1] -= half_thick * bspline_helper.smoothstep_quintic(lower_cp[:, 0])
            is_sharp = False

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
            
            if state.rotation_state == 0:
                target_plane = selected_line.parentSketch.referencePlane
            else:
                plane_input = parent_comp.constructionPlanes.createInput()
                plane_input.setByAngle(selected_line, adsk.core.ValueInput.createByReal(-theta), selected_line.parentSketch.referencePlane)
                target_plane = parent_comp.constructionPlanes.add(plane_input)
                target_plane.name = sketch_name

            if is_editable:
                # Create a temporary sketch on the target plane to use modelToSketchSpace for accurate transformation
                temp_sketch = parent_comp.sketches.add(target_plane)
                u_dxf = transform_pts(upper_cp, temp_sketch); l_dxf = transform_pts(lower_cp, temp_sketch)
                target_sketch = import_splines_via_dxf(temp_sketch, u_dxf, state.fit_cache['upper_knots'], state.fit_cache['degree_u'], l_dxf, state.fit_cache['lower_knots'], state.fit_cache['degree_l'], is_sharp, sketch_name=sketch_name)
                if temp_sketch != target_sketch: temp_sketch.deleteMe()
            else:
                target_sketch = parent_comp.sketches.add(target_plane); target_sketch.name = sketch_name
                u_final = transform_pts(upper_cp, target_sketch); l_final = transform_pts(lower_cp, target_sketch)
                create_fusion_spline(target_sketch, u_final, state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
                create_fusion_spline(target_sketch, l_final, state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
                if not is_sharp:
                    target_sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(u_final[-1,0], u_final[-1,1], 0), adsk.core.Point3D.create(l_final[-1,0], l_final[-1,1], 0))
        
        if target_sketch and inputs.itemById('import_raw').value and state.fit_cache.get('raw_upper') is not None:
            r_u = transform_pts(state.fit_cache['raw_upper'], target_sketch); r_l = transform_pts(state.fit_cache['raw_lower'], target_sketch)
            for pt in r_u: target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
            for pt in r_l: target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
                    
        return True
    except:
        app.log(f"Error: {traceback.format_exc()}")
        app.userInterface.messageBox('An error occurred, please check the log for more details.')
        return False