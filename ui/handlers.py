import adsk.core, adsk.fusion
import traceback
import math
import os
from ui.dialog import create_ui_inputs
from logic import state
from logic.fitter import run_fitter

class FusionFitterCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandCreatedEventArgs.cast(args)
            cmd = event_args.command
            
            on_execute = FusionFitterCommandExecuteHandler()
            cmd.execute.add(on_execute)
            state.handlers.append(on_execute)
            
            on_input_changed = FusionFitterCommandInputChangedHandler()
            cmd.inputChanged.add(on_input_changed)
            state.handlers.append(on_input_changed)
            
            on_execute_preview = FusionFitterCommandExecutePreviewHandler()
            cmd.executePreview.add(on_execute_preview)
            state.handlers.append(on_execute_preview)
            
            on_destroy = FusionFitterCommandDestroyedHandler()
            cmd.destroy.add(on_destroy)
            state.handlers.append(on_destroy)

            create_ui_inputs(cmd.commandInputs)

        except Exception as e:
            app = adsk.core.Application.get()
            app.userInterface.messageBox('Command Created Failed:\n{}'.format(traceback.format_exc()))

class FusionFitterCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandEventArgs.cast(args)
            event_args.isExecuted = run_fitter(event_args.command.commandInputs, False)
        except Exception as e:
            app = adsk.core.Application.get()
            app.userInterface.messageBox('Execution Error:\n{}'.format(traceback.format_exc()))

class FusionFitterCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.InputChangedEventArgs.cast(args)
            inputs = event_args.inputs
            # Get the root command inputs
            app = adsk.core.Application.get()
            try:
                cmd = event_args.firingEvent.sender
                if cmd:
                    inputs = cmd.commandInputs
            except:
                pass
         
            changed_id = event_args.input.id
            
            if changed_id == 'select_file':
                ui = adsk.core.Application.get().userInterface
                dlg = ui.createFileDialog()
                dlg.filter = 'Airfoil (*.dat);;All Files (*.*)'
                if dlg.showOpen() == adsk.core.DialogResults.DialogOK:
                    file_input = inputs.itemById('file_path')
                    file_input.value = dlg.filename
                    file_input.isVisible = True
                    # If preview is active, trigger immediate update with new file
                    preview_item = inputs.itemById('do_preview')
                    if preview_item and preview_item.value:
                        state.needs_refit = True
                        state.needs_preview = True
            
            elif changed_id in ['continuity_level', 'smoothness_input', 'cp_count', 'enforce_te_tangency']:
                #trigger refit
                state.needs_refit = True
                state.needs_preview = True
            
            elif changed_id == 'rotate_airfoil':
                state.rotation_state = (state.rotation_state + 1) % 4
            
            elif changed_id == 'flip_airfoil':
                state.flip_orientation = not state.flip_orientation
            
            elif changed_id == 'curvature_comb':
                # Show/hide comb settings based on checkbox
                comb_checked = inputs.itemById('curvature_comb').value
                comb_scale_item = inputs.itemById('comb_scale')
                comb_density_item = inputs.itemById('comb_density')
                if comb_scale_item:
                    comb_scale_item.isVisible = comb_checked
                if comb_density_item:
                    comb_density_item.isVisible = comb_checked
            
            chord_line_input = inputs.itemById('chord_line')
            file_path_input = inputs.itemById('file_path')
            has_selection = chord_line_input.selectionCount > 0 and file_path_input.value != ""
            
            toggle_ids = ['cp_count', 'te_thickness', 'smoothness_input', 'continuity_level',
                          'enforce_te_tangency', 'import_raw', 
                          'rotate_airfoil', 'flip_airfoil', 'do_preview', 'curvature_comb', 
                          'comb_scale', 'comb_density', 'editable_splines', 'fitter_settings', 'import_settings']
            
            for input_id in toggle_ids:
                item = inputs.itemById(input_id)
                if item:
                    item.isVisible = has_selection
            
            # Handle comb settings visibility based on checkbox state
            if has_selection:
                comb_checked = inputs.itemById('curvature_comb').value if inputs.itemById('curvature_comb') else False
                comb_scale_item = inputs.itemById('comb_scale')
                comb_density_item = inputs.itemById('comb_density')
                if comb_scale_item:
                    comb_scale_item.isVisible = comb_checked and has_selection
                if comb_density_item:
                    comb_density_item.isVisible = comb_checked and has_selection

            if changed_id in ['chord_line', 'select_file', 'rotate_airfoil', 'flip_airfoil']:
                import_settings_group = inputs.itemById('import_settings')
                if import_settings_group and has_selection:
                    import_settings_group.isVisible = True
                
                te_input = adsk.core.DistanceValueCommandInput.cast(inputs.itemById('te_thickness'))
                chord_line_input = inputs.itemById('chord_line')
                if te_input and chord_line_input.selectionCount > 0:
                    selected_line = adsk.fusion.SketchLine.cast(chord_line_input.selection(0).entity)
                    if selected_line:
                        start_pt = selected_line.startSketchPoint.worldGeometry
                        end_pt = selected_line.endSketchPoint.worldGeometry
                        chord_vec = adsk.core.Vector3D.create(end_pt.x - start_pt.x, 
                                                             end_pt.y - start_pt.y, 
                                                             end_pt.z - start_pt.z)
                        sketch = selected_line.parentSketch
                        mat = sketch.transform
                        if sketch.assemblyContext:
                            mat.transformBy(sketch.assemblyContext.worldTransform)
                        
                        sketch_normal_world = adsk.core.Vector3D.create(mat.getCell(0, 2),
                                                                       mat.getCell(1, 2),
                                                                       mat.getCell(2, 2))
                        sketch_normal_world.normalize()
                        theta = state.rotation_state * (math.pi / 2.0)
                        x_axis = chord_vec.copy()
                        x_axis.normalize()
                        y_axis_in_plane = sketch_normal_world.crossProduct(x_axis)
                        y_axis_in_plane.normalize()
                        mani_dir = y_axis_in_plane.copy()
                        mani_dir.scaleBy(math.cos(theta))
                        z_part = sketch_normal_world.copy()
                        z_part.scaleBy(math.sin(theta))
                        mani_dir.add(z_part)
                        mani_dir.normalize()
                        
                        # Trailing edge position depends on flip orientation
                        # When not flipped: TE is at end_pt (normal orientation)
                        # When flipped: TE is at start_pt (reversed orientation)
                        te_position = end_pt if not state.flip_orientation else start_pt
                        te_input.setManipulator(te_position, mani_dir)

            preview_item = inputs.itemById('do_preview')
            preview_checked = preview_item.value if preview_item else False
            if preview_checked:
                refit_ids = ['cp_count', 'smoothness_input', 'continuity_level',
                             'enforce_te_tangency', 'file_path']
                update_ids = ['te_thickness', 'import_raw', 'rotate_airfoil', 'flip_airfoil', 'chord_line',
                             'curvature_comb', 'comb_scale', 'comb_density']
                if changed_id in refit_ids:
                    state.needs_refit = True
                    state.needs_preview = True
                elif changed_id in update_ids:
                    state.needs_preview = True
            
            if changed_id == 'do_preview' and preview_checked:
                state.needs_preview = True
                state.needs_refit = True

        except Exception as e:
            pass

class FusionFitterCommandExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandEventArgs.cast(args)
            run_fitter(event_args.command.commandInputs, True)
        except Exception as e:
            pass

class FusionFitterCommandDestroyedHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        """Clean up when command is destroyed (finished or cancelled)."""
        try:
            # Clean up temporary DXF file if it exists
            import tempfile
            temp_dir = tempfile.gettempdir()
            dxf_path = os.path.join(temp_dir, "fusion_fitter_temp.dxf")
            if os.path.exists(dxf_path):
                try:
                    os.remove(dxf_path)
                except:
                    pass
            
            # Reset all state to default values (includes preview graphics cleanup)
            state.reset_state()
        except Exception as e:
            pass
