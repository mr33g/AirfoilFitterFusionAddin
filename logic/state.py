# Global state for the FusionFitter add-in
handlers = []
needs_preview = False
needs_refit = True
fit_cache = {}
rotation_state = 0
flip_orientation = False  # Flip airfoil nose to tail along chord line
preview_graphics = None  # Custom graphics group for control polygon preview

def reset_state():
    """Reset all state variables to their default values."""
    global needs_preview, needs_refit, fit_cache, rotation_state, flip_orientation, preview_graphics
    needs_preview = False
    needs_refit = True
    fit_cache = {}
    rotation_state = 0
    flip_orientation = False
    if preview_graphics:
        try:
            preview_graphics.deleteMe()
        except:
            pass
    preview_graphics = None