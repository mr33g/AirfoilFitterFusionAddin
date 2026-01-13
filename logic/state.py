# Global state for the AirfoilFitter add-in
handlers = []
needs_refit = True
fit_cache = {}
rotation_state = 0
flip_orientation = False  # Flip airfoil nose to tail along chord line
preview_graphics = None  # Custom graphics group for control polygon preview
current_cp_count_upper = None  # Track current control point count to detect changes
current_cp_count_lower = None  # Track current control point count to detect changes
bspline_processor = None  # Store the processor instance for refinement operations

def reset_state():
    """Reset all state variables to their default values."""
    global needs_refit, fit_cache, rotation_state, flip_orientation, preview_graphics, current_cp_count, bspline_processor
    needs_refit = True
    fit_cache = {}
    rotation_state = 0
    flip_orientation = False
    current_cp_count_upper = None
    current_cp_count_lower = None
    bspline_processor = None
    if preview_graphics:
        try:
            preview_graphics.deleteMe()
        except:
            pass
    preview_graphics = None