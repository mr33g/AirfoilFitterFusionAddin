import adsk.core, adsk.fusion
import traceback
import math
import os
from ui.dialog import create_ui_inputs
from logic import state
from logic.fitter import run_fitter
from core import config

def update_cp_count_labels(inputs):
    """Update the labels for CP count controls with current values."""
    try:
        cp_count_upper = inputs.itemById('cp_count_upper')
        if cp_count_upper:
            current_count = state.current_cp_count_upper if state.current_cp_count_upper is not None else config.DEFAULT_CP_COUNT
            cp_count_upper.text = f'  {current_count}'
        
        cp_count_lower = inputs.itemById('cp_count_lower')
        if cp_count_lower:
            current_count = state.current_cp_count_lower if state.current_cp_count_lower is not None else config.DEFAULT_CP_COUNT
            cp_count_lower.text = f'  {current_count}'
    except Exception as e:
        pass

def reset_fitter_settings_to_defaults(inputs, resetAll=False):
    """Reset all fitter settings to their default values. Preserves import settings."""
    try:
        # Reset control point counts in state (these are stored in state, not in the UI controls)
        state.fit_cache = {}
        state.preview_graphics = None
        state.current_cp_count_upper = None
        state.current_cp_count_lower = None
        
        # Update labels to show default values
        update_cp_count_labels(inputs)  
        
        if resetAll:
            # Reset soothness penalty
            smoothness = inputs.itemById('smoothness_input')
            if smoothness:
                smoothness.valueOne = config.DEFAULT_SMOOTHNESS_PENALTY
            
            # Reset continuity level to G1 (first item)
            continuity_dropdown = inputs.itemById('continuity_level')
            if continuity_dropdown:
                for i in range(continuity_dropdown.listItems.count):
                    continuity_dropdown.listItems.item(i).isSelected = (i == 0)
            
            # Reset TE tangency enforcement
            enforce_te_tangent = inputs.itemById('enforce_te_tangency')
            if enforce_te_tangent:
                enforce_te_tangent.value = False
    except Exception as e:
        pass

class AirfoilFitterCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandCreatedEventArgs.cast(args)
            cmd = event_args.command
            
            on_execute = AirfoilFitterCommandExecuteHandler()
            cmd.execute.add(on_execute)
            state.handlers.append(on_execute)
            
            on_input_changed = AirfoilFitterCommandInputChangedHandler()
            cmd.inputChanged.add(on_input_changed)
            state.handlers.append(on_input_changed)
            
            on_execute_preview = AirfoilFitterCommandExecutePreviewHandler()
            cmd.executePreview.add(on_execute_preview)
            state.handlers.append(on_execute_preview)
            
            on_destroy = AirfoilFitterCommandDestroyedHandler()
            cmd.destroy.add(on_destroy)
            state.handlers.append(on_destroy)

            create_ui_inputs(cmd.commandInputs)

        except Exception as e:
            app = adsk.core.Application.get()
            app.userInterface.messageBox('Command Created Failed:\n{}'.format(traceback.format_exc()))

class AirfoilFitterCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandEventArgs.cast(args)
            event_args.isExecuted = run_fitter(event_args.command.commandInputs, False)
        except Exception as e:
            app = adsk.core.Application.get()
            app.userInterface.messageBox('Execution Error:\n{}'.format(traceback.format_exc()))

class AirfoilFitterCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
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
                    # file_input.isVisible = True
                    
                    # Reset fitter settings to defaults when a new file is selected
                    reset_fitter_settings_to_defaults(inputs)
                    
                    # Reset state variables related to fitting
                    state.fit_cache = {}
                    state.current_cp_count_upper = None
                    state.current_cp_count_lower = None
                    
                    # Trigger preview update when file is selected (if line is also selected)
                    line_select = inputs.itemById('chord_line')
                    if line_select and line_select.selectionCount > 0:
                        state.needs_refit = True
            
            elif changed_id in ['continuity_level', 'smoothness_input', 'cp_count_upper', 'cp_count_lower', 'enforce_te_tangency']:
                if changed_id in ['continuity_level']:
                    state.fit_cache = {}
                
                # Correct CP count values before triggering refit
                if changed_id =='cp_count_upper' and state.current_cp_count_upper is not None:
                    state.current_cp_count_upper = state.current_cp_count_upper + 1
                    update_cp_count_labels(inputs)
                elif changed_id =='cp_count_lower' and state.current_cp_count_lower is not None:
                    state.current_cp_count_lower = state.current_cp_count_lower + 1
                    update_cp_count_labels(inputs)

                state.needs_refit = True
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
            
            elif changed_id == 'reset_button':
                reset_fitter_settings_to_defaults(inputs, False)
                state.needs_refit = True
            
            chord_line_input = inputs.itemById('chord_line')
            file_path_input = inputs.itemById('file_path')
            has_selection = chord_line_input.selectionCount > 0 and file_path_input.value != ""
            
            toggle_ids = ['cp_count_upper', 'cp_count_lower', 'te_thickness', 'smoothness_input', 'continuity_level',
                          'enforce_te_tangency', 'import_raw', 
                          'rotate_airfoil', 'flip_airfoil', 'curvature_comb', 
                          'comb_scale', 'comb_density', 'editable_splines', 'fitter_settings', 'import_settings', 'reset_button']
            
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
                            mat.transformBy(sketch.assemblyContext.transform2)
                        
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

            # Always trigger preview when parameters change (if line and file are selected)
            if has_selection:
                refit_ids = ['cp_count_upper', 'cp_count_lower', 'smoothness_input', 'continuity_level',
                             'enforce_te_tangency', 'file_path', 'chord_line']
                update_ids = ['te_thickness', 'import_raw', 'rotate_airfoil', 'flip_airfoil',
                             'curvature_comb', 'comb_scale', 'comb_density']
                if changed_id in refit_ids:
                    state.needs_refit = True
                elif changed_id in update_ids:
                    # For update-only changes, just mark that preview needs refresh
                    pass  # Preview will be triggered automatically via executePreview

        except Exception as e:
            pass

class AirfoilFitterCommandExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            event_args = adsk.core.CommandEventArgs.cast(args)
            run_fitter(event_args.command.commandInputs, True)
            # Update labels after fit completes (counts may have changed)
            update_cp_count_labels(event_args.command.commandInputs)
        except Exception as e:
            pass

class AirfoilFitterCommandDestroyedHandler(adsk.core.CommandEventHandler):
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
