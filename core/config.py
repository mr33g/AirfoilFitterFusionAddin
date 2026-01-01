"""Central project configuration constants.

This module gathers default numeric parameters and tunable hyper-parameters
used across the airfoil processing library so they live in one place.
Import these values instead of hard-coding magic numbers inside
algorithms or UI widgets.
"""
from __future__ import annotations

# B-spline settings
DEFAULT_BSPLINE_DEGREE: int = 5  # Degree of B-spline curves (3-7 recommended for airfoils)
DEFAULT_SMOOTHNESS_PENALTY: float = 0.01  # Weight for control point smoothing penalty (higher = smoother, lower = more accurate)

# Sampling settings (used for TE thickening)
PLOT_POINTS_PER_SURFACE: int = 500

# Number of points used for trailing edge vector calculations
# Higher numbers provide more robust tangent estimates but may be less sensitive to local geometry
DEFAULT_TE_VECTOR_POINTS: int = 2
