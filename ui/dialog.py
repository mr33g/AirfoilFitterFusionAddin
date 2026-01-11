import adsk.core, adsk.fusion
import traceback
from logic import state
from core import config

def create_ui_inputs(inputs):
    """Defines all the UI controls for the Fusion Fitter command."""
    try:
        # 1. Line Selection (Chord Line)
        line_select = inputs.addSelectionInput('chord_line', 'Select Chord Line', 'Select a construction line')
        line_select.addSelectionFilter('SketchLines')
        line_select.setSelectionLimits(1, 1)
        
        # 2. File Selection
        inputs.addBoolValueInput('select_file', 'Select Airfoil...', False, '', True)
        file_path_input = inputs.addStringValueInput('file_path', 'Selected File', '')
        file_path_input.isReadOnly = True
        file_path_input.isVisible = False
        

        # 3. Fitting Parameters (Initially Hidden)
        
        groupFitterSettings = inputs.addGroupCommandInput('fitter_settings', 'Fitter Settings')
        groupFitterSettings.isExpanded = True
        groupFitterSettings.isEnabledCheckBoxDisplayed = False
        groupFitterSettings.isVisible = False
        groupFitterSettingsChildInputs = groupFitterSettings.children
        
        # Control Point Count (Integer Slider/Spinner)
        cp_count_upper = groupFitterSettingsChildInputs.addBoolValueInput('cp_count_upper', f'CP Count Upper', False, 'resources/FusionFitterCommand/add', False)
        cp_count_upper.text = f'  {state.current_cp_count_upper if state.current_cp_count_upper is not None else config.DEFAULT_CP_COUNT}'
        cp_count_upper.isVisible = False
        cp_count_lower = groupFitterSettingsChildInputs.addBoolValueInput('cp_count_lower', f'CP Count Lower', False, 'resources/FusionFitterCommand/add', False)
        cp_count_lower.text = f'  {state.current_cp_count_lower if state.current_cp_count_lower is not None else config.DEFAULT_CP_COUNT}'
        cp_count_lower.isVisible = False

        # Reset Button

        reset_button = groupFitterSettingsChildInputs.addBoolValueInput('reset_button', 'Reset', False, 'resources/FusionFitterCommand/reset', False)
        reset_button.isVisible = False

        # Smoothness penalty
        smoothness = groupFitterSettingsChildInputs.addFloatSliderCommandInput('smoothness_input', 'Smoothness Penalty', "", 0, 0.1, False)
        smoothness.valueOne = 0.01
        smoothness.isVisible = False

        # 4. Options (Initially Hidden)
        # Continuity dropdown (G1, G2, G3)
        continuity_dropdown = groupFitterSettingsChildInputs.addDropDownCommandInput('continuity_level', 'LE Continuity', adsk.core.DropDownStyles.TextListDropDownStyle)
        continuity_dropdown.listItems.add('G1', False)
        continuity_dropdown.listItems.add('G2', False)
        continuity_dropdown.listItems.add('G3', False)
        continuity_dropdown.listItems[0].isSelected = True  # G1 selected by default
        continuity_dropdown.isVisible = False
        
        te_tan = groupFitterSettingsChildInputs.addBoolValueInput('enforce_te_tangency', 'Enforce TE Tangency', True, '', False)
        te_tan.isVisible = False
        
        groupImportSettings = inputs.addGroupCommandInput('import_settings', 'Import Settings')
        groupImportSettings.isExpanded = True
        groupImportSettings.isEnabledCheckBoxDisplayed = False
        groupImportSettings.isVisible = False
        groupImportSettingsChildInputs = groupImportSettings.children

        # Rotation Button (Cycles 0, 90, 180, 270) - use rotate.svg icon
        rotate = groupImportSettingsChildInputs.addBoolValueInput('rotate_airfoil', 'Turn 90°', False, 'resources/FusionFitterCommand/rotate', False)
        rotate.isVisible = False
        
        # Flip Button (Reverses nose to tail orientation) - use flip.svg icon
        flip = groupImportSettingsChildInputs.addBoolValueInput('flip_airfoil', 'Flip', False, 'resources/FusionFitterCommand/flip', False)
        flip.isVisible = False

        # TE Thickness (Distance Manipulator)
        te_thickness = groupImportSettingsChildInputs.addDistanceValueCommandInput('te_thickness', 'TE Thickness', adsk.core.ValueInput.createByReal(0))
        te_thickness.isVisible = False
        te_thickness.minimumValue = 0
        te_thickness.isMinimumValueInclusive = True
        te_thickness.maximumValue = 10
        te_thickness.isMaximumValueInclusive = True

        # Editable Results
        editable = groupImportSettingsChildInputs.addBoolValueInput('editable_splines', 'Keep adjustable', True, '', False)
        editable.isVisible = False

        # 5. Curvature Comb (Initially Hidden)
        curvature_comb = inputs.addBoolValueInput('curvature_comb', 'Curvature comb', True, 'resources/FusionFitterCommand/comb', False)
        curvature_comb.isVisible = False
        # Curvature Comb Settings (Initially Hidden)
        comb_scale = inputs.addFloatSliderCommandInput('comb_scale', 'Comb Scale', "", 0.0001, 0.05, False)
        comb_scale.valueOne = 0.005
        comb_scale.isVisible = False

        comb_density = inputs.addIntegerSliderCommandInput('comb_density', 'Comb Density', 10, 500, False)
        comb_density.valueOne = 200
        comb_density.isVisible = False

        raw = inputs.addBoolValueInput('import_raw', 'Show Input Data', True, '', False)
        raw.isVisible = False

    except Exception as e:
        app = adsk.core.Application.get()
        app.userInterface.messageBox('UI Creation Failed:\n{}'.format(traceback.format_exc()))

