import adsk.core, adsk.fusion
import os
import traceback
import numpy as np
from logic import state
import airfoil_fitter_settings as config
from logic.airfoil_frame import chord_frame
from logic import fit_service
from utils.fusion_geometry_helper import create_fusion_spline, import_splines_via_dxf
from utils.sketch_plane_helper import AirfoilPlaneError, resolve_airfoil_plane, add_airfoil_sketch
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
        
        airfoil_to_world, chord_length = chord_frame(
            selected_line, state.rotation_state, state.flip_orientation)
        start_pt_world = (selected_line.endSketchPoint.worldGeometry
                          if state.flip_orientation else chord_start_world_original)
        y_axis_world = adsk.core.Vector3D.create(
            airfoil_to_world.getCell(0, 1), airfoil_to_world.getCell(1, 1),
            airfoil_to_world.getCell(2, 1))
        z_axis_world = adsk.core.Vector3D.create(
            airfoil_to_world.getCell(0, 2), airfoil_to_world.getCell(1, 2),
            airfoil_to_world.getCell(2, 2))

        # 2. Fitting Logic
        do_new_fit = (state.needs_refit or not state.fit_cache) if is_preview else True

        def selected_initial_cp_count():
            try:
                initial_cp_count = inputs.itemById('initial_cp_count')
                if initial_cp_count:
                    if hasattr(initial_cp_count, 'value'):
                        return max(config.MIN_CP_COUNT, min(config.MAX_CP_COUNT, int(initial_cp_count.value)))
                    if initial_cp_count.selectedItem:
                        return max(config.MIN_CP_COUNT, min(config.MAX_CP_COUNT, int(initial_cp_count.selectedItem.name)))
            except Exception:
                pass
            return config.DEFAULT_CP_COUNT

        if do_new_fit:
            file_path = inputs.itemById('file_path').value
            if not file_path or not os.path.exists(file_path):
                return False

            source = fit_service.read_source(file_path)
            is_initial_fit = (not state.fit_cache
                              or state.current_cp_count_upper is None
                              or state.current_cp_count_lower is None)
            if is_initial_fit:
                inputs.itemById('te_thickness').value = (
                    fit_service.source_te_thickness(source) * chord_length)

            continuity_dropdown = inputs.itemById('continuity_level')
            continuity = (int(continuity_dropdown.selectedItem.name[1:])
                          if continuity_dropdown and continuity_dropdown.selectedItem
                          else config.DEFAULT_CONTINUITY)
            initial_cp_count = selected_initial_cp_count()
            values = dict(
                te=inputs.itemById('te_thickness').value,
                smoothness=inputs.itemById('smoothness_input').valueOne,
                upper_count=(state.current_cp_count_upper
                             if state.current_cp_count_upper is not None
                             else initial_cp_count),
                lower_count=(state.current_cp_count_lower
                             if state.current_cp_count_lower is not None
                             else initial_cp_count),
                continuity=continuity)
            requested_smoothing = values['smoothness']
            requested_continuity = values['continuity']
            fitted = fit_service.fit_data(source, values, chord_length)
            if fitted['fallback_used']:
                app.log(
                    f'AirfoilFitter fit fallback: requested G{requested_continuity} '
                    f'smoothing={requested_smoothing:g}; using G{values["continuity"]} '
                    f'smoothing={values["smoothness"]:g}.')
            state.fit_cache = fitted
            state.current_cp_count_upper = int(values['upper_count'])
            state.current_cp_count_lower = int(values['lower_count'])
            state.needs_refit = False

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
            
            # Render all preview graphics
            render_preview(
                target_sketch, upper_cp, lower_cp, state.fit_cache,
                chord_length, airfoil_to_world, y_axis_world,
                transform_pts, inputs, is_sharp
            )
        else:
            file_path = inputs.itemById('file_path').value
            sketch_name = os.path.splitext(os.path.basename(file_path))[0] if file_path else "Fitted Airfoil"
            source_sketch = selected_line.parentSketch
            target_plane = resolve_airfoil_plane(
                selected_line, state.rotation_state, start_pt_world, z_axis_world,
                design.rootComponent, sketch_name,
            )

            if is_editable:
                # Create a temporary sketch on the target plane to use modelToSketchSpace for accurate transformation
                temp_sketch, target_plane = add_airfoil_sketch(
                    source_sketch, target_plane, sketch_name, return_support=True)
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
                    chord_start_aligned, chord_end_aligned, sketch_name=sketch_name,
                    source_sketch=source_sketch
                )
                if temp_sketch != target_sketch: temp_sketch.deleteMe()
            else:
                target_sketch = add_airfoil_sketch(source_sketch, target_plane, sketch_name)
                u_final = transform_pts(upper_cp, target_sketch); l_final = transform_pts(lower_cp, target_sketch)
                create_fusion_spline(target_sketch, u_final, state.fit_cache['upper_knots'], state.fit_cache['degree_u'])
                create_fusion_spline(target_sketch, l_final, state.fit_cache['lower_knots'], state.fit_cache['degree_l'])
                if not is_sharp:
                    target_sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(u_final[-1,0], u_final[-1,1], 0), adsk.core.Point3D.create(l_final[-1,0], l_final[-1,1], 0))
        
                    
        return True
    except fit_service.AirfoilFitError as exc:
        state.needs_refit = True
        app.log(str(exc))
        if not is_preview:
            app.userInterface.messageBox(str(exc))
        return False
    except AirfoilPlaneError as exc:
        app.log(f"AirfoilFitter plane creation failed: {traceback.format_exc()}")
        app.userInterface.messageBox(t("failed_create_airfoil_plane", error=str(exc)))
        return False
    except:
        app.log(f"Error: {traceback.format_exc()}")
        app.userInterface.messageBox(t("generic_error"))
        return False
