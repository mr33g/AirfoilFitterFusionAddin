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
            enforce_g2 = inputs.itemById('enforce_g2').value
            enforce_g3 = inputs.itemById('enforce_g3').value if enforce_g2 else False
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
                if not curve: return 0.0, 0.0
                u_params = bspline_helper.create_parameter_from_x_coords(data, exponent)
                diffs = np.linalg.norm(data - curve(u_params), axis=1)
                idx = np.argmax(diffs)
                return diffs[idx], data[idx, 0]

            err_u, _ = calc_max_err(bspline.upper_curve, processor.upper_data, bspline.param_exponent_upper)
            err_l, _ = calc_max_err(bspline.lower_curve, processor.lower_data, bspline.param_exponent_lower)
            
            state.fit_cache = {
                'upper_cp_raw': bspline.upper_control_points.copy(),
                'lower_cp_raw': bspline.lower_control_points.copy(),
                'upper_knots': bspline.upper_knot_vector, 'lower_knots': bspline.lower_knot_vector,
                'degree_u': bspline.degree_upper, 'degree_l': bspline.degree_lower,
                'is_sharp': bspline.is_sharp_te,
                'err_u': err_u, 'err_l': err_l,
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
        err_u_val = units_mgr.convert(state.fit_cache['err_u'] * chord_length, 'cm', def_units)
        err_l_val = units_mgr.convert(state.fit_cache['err_l'] * chord_length, 'cm', def_units)
        inputs.itemById('status_box').text = f"Upper Max Err: {err_u_val:.{decimals}f} {def_units}\nLower Max Err: {err_l_val:.{decimals}f} {def_units}"

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
            u_cp_trans = transform_pts(upper_cp, target_sketch)
            l_cp_trans = transform_pts(lower_cp, target_sketch)
            create_fusion_spline(target_sketch, u_cp_trans, state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
            create_fusion_spline(target_sketch, l_cp_trans, state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
            
            # Create custom graphics for control polygon (selection-immune)
            root = design.rootComponent
            state.preview_graphics = root.customGraphicsGroups.add()
            
            # Convert control points to world coordinates for custom graphics
            def get_world_pts(cp_list):
                """Convert control points to world coordinates as flat list [x, y, z, x, y, z, ...]"""
                coords = []
                for pt in cp_list:
                    p_world = adsk.core.Point3D.create(pt[0] * chord_length, pt[1] * chord_length, 0)
                    p_world.transformBy(airfoil_to_world)
                    coords.extend([p_world.x, p_world.y, p_world.z])
                return coords
            
            # Build coordinate arrays for upper and lower control polygons
            upper_coords = get_world_pts(upper_cp)
            lower_coords = get_world_pts(lower_cp)
            
            # Helper function to create connectivity indices for sequential lines
            # Try using sequential indices with isLineStrip=True for a connected polyline
            def create_line_indices(num_points):
                """Create sequential indices for a polyline: [0, 1, 2, 3, ...]"""
                if num_points < 2:
                    return []
                # Return sequential indices for use with isLineStrip=True
                return [int(i) for i in range(num_points)]
            
            # Draw control polygon lines using custom graphics
            # Upper polygon lines - use isLineStrip=True with sequential indices
            num_upper_points = len(upper_cp)
            if num_upper_points >= 2 and len(upper_coords) >= 6:  # At least 2 points (6 coords)
                try:
                    cg_coords_upper = adsk.fusion.CustomGraphicsCoordinates.create(upper_coords)
                    upper_indices = create_line_indices(num_upper_points)
                    # Use isLineStrip=True to connect points sequentially
                    cg_lines_upper = state.preview_graphics.addLines(cg_coords_upper, upper_indices, True)
                    if cg_lines_upper:
                        # Set color to orange (RGB: 255, 165, 0)
                        cg_lines_upper.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
                        # Set line style to dashed
                        cg_lines_upper.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern
                        # cg_lines_upper.weight = 2
                        cg_lines_upper.lineStyleScale = 0.15
                        cg_lines_upper.isScreenSpaceLineStyle = False

                except Exception as e:
                    app.log(f"Error drawing upper control polygon: {e}")
            
            # Lower polygon lines - use isLineStrip=True with sequential indices
            num_lower_points = len(lower_cp)
            if num_lower_points >= 2 and len(lower_coords) >= 6:  # At least 2 points (6 coords)
                try:
                    cg_coords_lower = adsk.fusion.CustomGraphicsCoordinates.create(lower_coords)
                    lower_indices = create_line_indices(num_lower_points)
                    # Use isLineStrip=True to connect points sequentially
                    cg_lines_lower = state.preview_graphics.addLines(cg_coords_lower, lower_indices, True)
                    if cg_lines_lower:
                        # Set color to orange (RGB: 255, 165, 0)
                        cg_lines_lower.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
                        # Set line style to dashed
                        cg_lines_lower.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern
                        # cg_lines_lower.weight = 2
                        cg_lines_lower.lineStyleScale = .15
                        cg_lines_lower.isScreenSpaceLineStyle = False

                except Exception as e:
                    app.log(f"Error drawing lower control polygon: {e}")
            
            # Draw control polygon points (handles) using sketch geometry
            # Sketch points don't cause selection issues, so we can use them safely
            try:
                # Add points for upper control polygon
                for pt in u_cp_trans:
                    target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
                # Add points for lower control polygon
                for pt in l_cp_trans:
                    target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
            except Exception as e:
                app.log(f"Error drawing control polygon points: {e}")
            
            # Draw trailing edge line if not sharp
            if not is_sharp and num_upper_points > 0 and num_lower_points > 0:
                # The TE line connects the last point of upper to the last point of lower
                # Create coordinates array with just the two TE points
                te_coords = [
                    upper_coords[-3], upper_coords[-2], upper_coords[-1],  # Last upper point
                    lower_coords[-3], lower_coords[-2], lower_coords[-1]   # Last lower point
                ]
                te_indices = [0, 1]  # Connect point 0 to point 1
                cg_coords_te = adsk.fusion.CustomGraphicsCoordinates.create(te_coords)
                cg_lines_te = state.preview_graphics.addLines(cg_coords_te, te_indices, False)
                if cg_lines_te:
                    # Set color to orange (RGB: 255, 165, 0) to match control polygon lines
                    cg_lines_te.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
                    # Set line style to dashed
                    cg_lines_te.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern
            
            # Draw curvature comb if enabled
            curvature_comb_item = inputs.itemById('curvature_comb')
            if curvature_comb_item and curvature_comb_item.value and state.fit_cache:
                try:
                    from scipy import interpolate
                    
                    # Recreate BSpline curves from current control points (includes TE thickening if applied)
                    upper_curve = interpolate.BSpline(
                        state.fit_cache['upper_knots'],
                        upper_cp,
                        state.fit_cache['degree_u']
                    )
                    lower_curve = interpolate.BSpline(
                        state.fit_cache['lower_knots'],
                        lower_cp,
                        state.fit_cache['degree_l']
                    )
                    
                    # Get comb settings
                    comb_scale_item = inputs.itemById('comb_scale')
                    comb_density_item = inputs.itemById('comb_density')
                    comb_scale = comb_scale_item.valueOne if comb_scale_item else 0.05
                    # IntegerSliderCommandInput uses .value, not .valueOne
                    comb_density = int(comb_density_item.valueOne) if comb_density_item else 200
                    
                    # Calculate comb data
                    comb_data = bspline_helper.calculate_curvature_comb_data(
                        upper_curve, lower_curve,
                        num_points_per_segment=comb_density,
                        scale_factor=comb_scale
                    )
                    
                    if comb_data and len(comb_data) == 2:
                        
                        # Draw comb for upper and lower curves
                        for curve_idx, curve_comb_hairs in enumerate(comb_data):
                            if not curve_comb_hairs:
                                continue
                            
                            # Collect all coordinates and indices for comb hairs and outer tips
                            all_coords = []
                            hair_indices = []
                            outer_tips_coords = []  # Coordinates for the outer tips (end points of hairs)
                            
                            coord_idx = 0
                            for hair_segment in curve_comb_hairs:
                                # Transform hair segment from airfoil space to world space
                                start_pt_airfoil = hair_segment[0]
                                end_pt_airfoil = hair_segment[1]
                                
                                # Transform to world coordinates
                                start_world = adsk.core.Point3D.create(
                                    start_pt_airfoil[0] * chord_length,
                                    start_pt_airfoil[1] * chord_length,
                                    0
                                )
                                end_world = adsk.core.Point3D.create(
                                    end_pt_airfoil[0] * chord_length,
                                    end_pt_airfoil[1] * chord_length,
                                    0
                                )
                                start_world.transformBy(airfoil_to_world)
                                end_world.transformBy(airfoil_to_world)
                                
                                # Add to coordinates for hairs
                                all_coords.extend([start_world.x, start_world.y, start_world.z])
                                all_coords.extend([end_world.x, end_world.y, end_world.z])
                                
                                # Hair line indices (connect start to end)
                                hair_indices.extend([coord_idx, coord_idx + 1])
                                
                                # Collect outer tips (end points of hairs) for the outline
                                outer_tips_coords.extend([end_world.x, end_world.y, end_world.z])
                                
                                coord_idx += 2
                            
                            # Draw comb hairs (light blue)
                            if all_coords and hair_indices:
                                cg_coords_hairs = adsk.fusion.CustomGraphicsCoordinates.create(all_coords)
                                cg_hairs = state.preview_graphics.addLines(cg_coords_hairs, hair_indices, False)
                                if cg_hairs:
                                    # Light blue color (RGB: 40, 169, 212)
                                    cg_hairs.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(40, 169, 212, 255))

                            # Draw red outline connecting the outer tips of the hairs
                            if len(outer_tips_coords) >= 6:  # At least 2 points
                                outer_tips_indices = [int(i) for i in range(len(outer_tips_coords) // 3)]
                                cg_coords_outline = adsk.fusion.CustomGraphicsCoordinates.create(outer_tips_coords)
                                cg_outline = state.preview_graphics.addLines(cg_coords_outline, outer_tips_indices, True)
                                if cg_outline:
                                    # Red color for outline
                                    cg_outline.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 0, 0, 255))
                
                except Exception as e:
                    app.log(f"Error drawing curvature comb: {e}")
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
