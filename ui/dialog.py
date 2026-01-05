import adsk.core, adsk.fusion
import traceback

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
        # Control Point Count (Integer Slider/Spinner)
        cp_count = inputs.addIntegerSpinnerCommandInput('cp_count', 'Control Point Count', 4, 19, 1, 10)
        cp_count.isVisible = False
        
        # TE Thickness (Distance Manipulator)
        te_thickness = inputs.addDistanceValueCommandInput('te_thickness', 'TE Thickness', adsk.core.ValueInput.createByReal(0))
        te_thickness.isVisible = False
        te_thickness.minimumValue = 0
        te_thickness.isMinimumValueInclusive = True
        te_thickness.maximumValue = 10
        te_thickness.isMaximumValueInclusive = True
        
        # Smoothness penalty
        smoothness = inputs.addFloatSliderCommandInput('smoothness_input', 'Smoothness Penalty', "", 0, 0.1, False)
        smoothness.valueOne = 0.01
        smoothness.isVisible = False

        # 4. Options (Initially Hidden)
        g2 = inputs.addBoolValueInput('enforce_g2', 'G2', True, '', False)
        g2.isVisible = False
        g3 = inputs.addBoolValueInput('enforce_g3', 'G3', True, '', False)
        g3.isVisible = False
        te_tan = inputs.addBoolValueInput('enforce_te_tangency', 'Enforce TE Tangency', True, '', False)
        te_tan.isVisible = False
        raw = inputs.addBoolValueInput('import_raw', 'Show Input Data', True, '', False)
        raw.isVisible = False
        
        # Rotation Button (Cycles 0, 90, 180, 270) - use rotate.svg icon
        rotate = inputs.addBoolValueInput('rotate_airfoil', 'Rotate 90°', False, 'resources/FusionFitterCommand/rotate', False)
        rotate.isVisible = False
        
        # Flip Button (Reverses nose to tail orientation) - use flip.svg icon
        flip = inputs.addBoolValueInput('flip_airfoil', 'Flip', False, 'resources/FusionFitterCommand/flip', False)
        flip.isVisible = False
        
        # 5. Preview (Initially Hidden)
        preview = inputs.addBoolValueInput('do_preview', 'Preview', True, '', True)
        preview.isVisible = False
        
        # 6. Curvature Comb (Initially Hidden)
        curvature_comb = inputs.addBoolValueInput('curvature_comb', 'Curvature comb', True, 'resources/FusionFitterCommand/comb', False)
        curvature_comb.isVisible = False
        
        # Curvature Comb Settings (Initially Hidden)
        comb_scale = inputs.addFloatSliderCommandInput('comb_scale', 'Comb Scale', "", 0.0001, 0.05, False)
        comb_scale.valueOne = 0.005
        comb_scale.isVisible = False
        
        comb_density = inputs.addIntegerSliderCommandInput('comb_density', 'Comb Density', 10, 500, False)
        comb_density.valueOne = 200
        comb_density.isVisible = False
        
        # 7. Editable (Initially Hidden)
        editable = inputs.addBoolValueInput('editable_splines', 'Editable', True, '', False)
        editable.isVisible = False

        # 8. Status / Deviation info (Initially Hidden)
        status_box = inputs.addTextBoxCommandInput('status_box', '', '', 4, True)
        status_box.isFullWidth = True
        status_box.isVisible = False

    except Exception as e:
        app = adsk.core.Application.get()
        app.userInterface.messageBox('UI Creation Failed:\n{}'.format(traceback.format_exc()))

