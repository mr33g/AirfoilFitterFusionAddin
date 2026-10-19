# Global state for the AirfoilFitter add-in
handlers = []
needs_refit = True
fit_cache = {}
rotation_state = 0
flip_orientation = False  # Flip airfoil nose to tail along chord line
preview_graphics = None  # Custom graphics group for control polygon preview
graphics_world_to_local = None  # Transform for graphics when sketch is in sub-component
current_cp_count_upper = None  # Track current control point count to detect changes
current_cp_count_lower = None  # Track current control point count to detect changes

def reset_state():
    """Reset all state variables to their default values."""
    global needs_refit, fit_cache, rotation_state, flip_orientation
    global preview_graphics, graphics_world_to_local
    global current_cp_count_upper, current_cp_count_lower
    needs_refit = True
    fit_cache = {}
    rotation_state = 0
    flip_orientation = False
    graphics_world_to_local = None
    current_cp_count_upper = None
    current_cp_count_lower = None
    if preview_graphics:
        try:
            preview_graphics.deleteMe()
        except:
            pass
    preview_graphics = None
