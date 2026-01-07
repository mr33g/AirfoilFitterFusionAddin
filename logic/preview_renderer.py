"""
Preview rendering for airfoil fitting visualization.

This module handles all preview graphics rendering including control polygons,
curvature combs, error markers, and text labels.
"""

import adsk.core, adsk.fusion
import os
import numpy as np
from scipy import interpolate
from logic import state
from utils import bspline_helper
from logic.fusion_graphics import draw_error_labels

_addin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _get_world_pts(cp_list, chord_length, airfoil_to_world):
    """Convert control points to world coordinates as flat list [x, y, z, x, y, z, ...]"""
    coords = []
    for pt in cp_list:
        p_world = adsk.core.Point3D.create(pt[0] * chord_length, pt[1] * chord_length, 0)
        p_world.transformBy(airfoil_to_world)
        coords.extend([p_world.x, p_world.y, p_world.z])
    return coords


def _create_line_indices(num_points):
    """Create sequential indices for a polyline: [0, 1, 2, 3, ...]"""
    if num_points < 2:
        return []
    return [int(i) for i in range(num_points)]


def draw_control_polygon(graphics_group, upper_cp, lower_cp, chord_length, 
                         airfoil_to_world, target_sketch, u_cp_trans, l_cp_trans):
    """
    Draw control polygon lines and points.
    
    Args:
        graphics_group: CustomGraphicsGroup to add graphics to
        upper_cp: Upper control points
        lower_cp: Lower control points
        chord_length: Chord length for scaling
        airfoil_to_world: Transformation matrix
        target_sketch: Target sketch for control points
        u_cp_trans: Transformed upper control points
        l_cp_trans: Transformed lower control points
    """
    app = adsk.core.Application.get()
    
    # Convert control points to world coordinates
    upper_coords = _get_world_pts(upper_cp, chord_length, airfoil_to_world)
    lower_coords = _get_world_pts(lower_cp, chord_length, airfoil_to_world)
    
    # Draw control polygon lines
    num_upper_points = len(upper_cp)
    if num_upper_points >= 2 and len(upper_coords) >= 6:
        try:
            cg_coords_upper = adsk.fusion.CustomGraphicsCoordinates.create(upper_coords)
            upper_indices = _create_line_indices(num_upper_points)
            cg_lines_upper = graphics_group.addLines(cg_coords_upper, upper_indices, True)
            if cg_lines_upper:
                cg_lines_upper.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
                cg_lines_upper.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern
                cg_lines_upper.lineStyleScale = 0.15
                cg_lines_upper.isScreenSpaceLineStyle = False
        except Exception as e:
            app.log(f"Error drawing upper control polygon: {e}")
    
    num_lower_points = len(lower_cp)
    if num_lower_points >= 2 and len(lower_coords) >= 6:
        try:
            cg_coords_lower = adsk.fusion.CustomGraphicsCoordinates.create(lower_coords)
            lower_indices = _create_line_indices(num_lower_points)
            cg_lines_lower = graphics_group.addLines(cg_coords_lower, lower_indices, True)
            if cg_lines_lower:
                cg_lines_lower.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
                cg_lines_lower.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern
                cg_lines_lower.lineStyleScale = 0.15
                cg_lines_lower.isScreenSpaceLineStyle = False
        except Exception as e:
            app.log(f"Error drawing lower control polygon: {e}")
    
    # Draw control polygon points (handles) using sketch geometry
    try:
        for pt in u_cp_trans:
            target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
        for pt in l_cp_trans:
            target_sketch.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2]))
    except Exception as e:
        app.log(f"Error drawing control polygon points: {e}")
    
    return upper_coords, lower_coords


def draw_trailing_edge_line(graphics_group, upper_coords, lower_coords, is_sharp):
    """
    Draw trailing edge line if not sharp.
    
    Args:
        graphics_group: CustomGraphicsGroup to add graphics to
        upper_coords: Upper control point coordinates in world space
        lower_coords: Lower control point coordinates in world space
        is_sharp: Whether trailing edge is sharp
    """
    if is_sharp or len(upper_coords) < 6 or len(lower_coords) < 6:
        return
    
    # The TE line connects the last point of upper to the last point of lower
    te_coords = [
        upper_coords[-3], upper_coords[-2], upper_coords[-1],  # Last upper point
        lower_coords[-3], lower_coords[-2], lower_coords[-1]   # Last lower point
    ]
    te_indices = [0, 1]  # Connect point 0 to point 1
    cg_coords_te = adsk.fusion.CustomGraphicsCoordinates.create(te_coords)
    cg_lines_te = graphics_group.addLines(cg_coords_te, te_indices, False)
    if cg_lines_te:
        cg_lines_te.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 165, 0, 255))
        cg_lines_te.lineStylePattern = adsk.fusion.LineStylePatterns.dashedLineStylePattern


def draw_curvature_comb(graphics_group, upper_cp, lower_cp, fit_cache, 
                       chord_length, airfoil_to_world, inputs):
    """
    Draw curvature comb visualization if enabled.
    
    Args:
        graphics_group: CustomGraphicsGroup to add graphics to
        upper_cp: Upper control points
        lower_cp: Lower control points
        fit_cache: Fit cache dictionary
        chord_length: Chord length for scaling
        airfoil_to_world: Transformation matrix
        inputs: Command inputs
    """
    app = adsk.core.Application.get()
    curvature_comb_item = inputs.itemById('curvature_comb')
    if not curvature_comb_item or not curvature_comb_item.value or not state.fit_cache:
        return
    
    try:
        # Recreate BSpline curves from current control points (includes TE thickening if applied)
        upper_curve = interpolate.BSpline(
            fit_cache['upper_knots'],
            upper_cp,
            fit_cache['degree_u']
        )
        lower_curve = interpolate.BSpline(
            fit_cache['lower_knots'],
            lower_cp,
            fit_cache['degree_l']
        )
        
        # Get comb settings
        comb_scale_item = inputs.itemById('comb_scale')
        comb_density_item = inputs.itemById('comb_density')
        comb_scale = comb_scale_item.valueOne if comb_scale_item else 0.05
        comb_density = int(comb_density_item.valueOne) if comb_density_item else 200
        
        # Calculate comb data
        comb_data = bspline_helper.calculate_curvature_comb_data(
            upper_curve, lower_curve,
            num_points_per_segment=comb_density,
            scale_factor=comb_scale
        )
        
        if not comb_data or len(comb_data) != 2:
            return
        
        # Draw comb for upper and lower curves
        for curve_idx, curve_comb_hairs in enumerate(comb_data):
            if not curve_comb_hairs:
                continue
            
            # Collect all coordinates and indices for comb hairs and outer tips
            all_coords = []
            hair_indices = []
            outer_tips_coords = []
            
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
                cg_hairs = graphics_group.addLines(cg_coords_hairs, hair_indices, False)
                if cg_hairs:
                    cg_hairs.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(40, 169, 212, 255))
            
            # Draw red outline connecting the outer tips of the hairs
            if len(outer_tips_coords) >= 6:  # At least 2 points
                outer_tips_indices = [int(i) for i in range(len(outer_tips_coords) // 3)]
                cg_coords_outline = adsk.fusion.CustomGraphicsCoordinates.create(outer_tips_coords)
                cg_outline = graphics_group.addLines(cg_coords_outline, outer_tips_indices, True)
                if cg_outline:
                    cg_outline.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(255, 0, 0, 255))
    
    except Exception as e:
        app.log(f"Error drawing curvature comb: {e}")


def draw_error_markers(graphics_group, upper_cp, lower_cp, fit_cache,
                       chord_length, airfoil_to_world, y_axis_world):
    """
    Draw error location markers and labels.
    
    Args:
        graphics_group: CustomGraphicsGroup to add graphics to
        upper_cp: Upper control points
        lower_cp: Lower control points
        fit_cache: Fit cache dictionary
        chord_length: Chord length for scaling
        airfoil_to_world: Transformation matrix
        y_axis_world: Y-axis vector in world coordinates (for fallback)
    """
    if fit_cache.get('max_err_pt_u') is None or fit_cache.get('max_err_pt_l') is None:
        return
    
    error_points_coords = []
    
    # Recreate splines from current control points (which may include TE thickening)
    upper_curve_current = interpolate.BSpline(
        fit_cache['upper_knots'],
        upper_cp,
        fit_cache['degree_u']
    )
    lower_curve_current = interpolate.BSpline(
        fit_cache['lower_knots'],
        lower_cp,
        fit_cache['degree_l']
    )
    
    # Get the max error data points
    max_err_data_pt_u = fit_cache['max_err_pt_u']
    max_err_data_pt_l = fit_cache['max_err_pt_l']
    
    # Find the points on the spline closest to the max error data points
    spline_pt_u, u_param_u = bspline_helper.find_closest_point_on_spline(
        upper_curve_current, max_err_data_pt_u
    )
    spline_pt_l, u_param_l = bspline_helper.find_closest_point_on_spline(
        lower_curve_current, max_err_data_pt_l
    )
    
    # Upper surface max error point (on spline)
    p_world_u = adsk.core.Point3D.create(
        spline_pt_u[0] * chord_length,
        spline_pt_u[1] * chord_length,
        0
    )
    p_world_u.transformBy(airfoil_to_world)
    error_points_coords.extend([p_world_u.x, p_world_u.y, p_world_u.z])
    
    # Lower surface max error point (on spline)
    p_world_l = adsk.core.Point3D.create(
        spline_pt_l[0] * chord_length,
        spline_pt_l[1] * chord_length,
        0
    )
    p_world_l.transformBy(airfoil_to_world)
    error_points_coords.extend([p_world_l.x, p_world_l.y, p_world_l.z])
    
    # Create CustomGraphicsPointSet
    cg_coords_points = adsk.fusion.CustomGraphicsCoordinates.create(error_points_coords)
    point_indices = [0, 1]
    error_image_path = os.path.join(_addin_dir, 'resources', 'FusionFitterCommand', 'error', '12x12.png')
    point_type = adsk.fusion.CustomGraphicsPointTypes.PointCloudCustomGraphicsPointType
    cg_point_set = graphics_group.addPointSet(
        cg_coords_points, 
        point_indices,
        point_type,
        error_image_path
    )
    
    # Add billboarding text labels for error values
    draw_error_labels(
        graphics_group,
        upper_curve_current, lower_curve_current,
        u_param_u, u_param_l,
        max_err_data_pt_u, max_err_data_pt_l,
        p_world_u, p_world_l,
        airfoil_to_world, chord_length, y_axis_world,
        fit_cache
    )


def render_preview(target_sketch, upper_cp, lower_cp, fit_cache, chord_length,
                   airfoil_to_world, y_axis_world, transform_pts, inputs, is_sharp):
    """
    Render all preview graphics.
    
    Args:
        target_sketch: Target sketch for splines and control points
        upper_cp: Upper control points
        lower_cp: Lower control points
        fit_cache: Fit cache dictionary
        chord_length: Chord length for scaling
        airfoil_to_world: Transformation matrix
        y_axis_world: Y-axis vector in world coordinates
        transform_pts: Function to transform points
        inputs: Command inputs
        is_sharp: Whether trailing edge is sharp
    """
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    
    # Transform control points for sketch
    u_cp_trans = transform_pts(upper_cp, target_sketch)
    l_cp_trans = transform_pts(lower_cp, target_sketch)
    
    # Create custom graphics group
    root = design.rootComponent
    state.preview_graphics = root.customGraphicsGroups.add()
    
    # Draw control polygon
    upper_coords, lower_coords = draw_control_polygon(
        state.preview_graphics, upper_cp, lower_cp, chord_length,
        airfoil_to_world, target_sketch, u_cp_trans, l_cp_trans
    )
    
    # Draw trailing edge line
    draw_trailing_edge_line(state.preview_graphics, upper_coords, lower_coords, is_sharp)
    
    # Draw curvature comb if enabled
    draw_curvature_comb(
        state.preview_graphics, upper_cp, lower_cp, fit_cache,
        chord_length, airfoil_to_world, inputs
    )
    
    # Draw error markers
    draw_error_markers(
        state.preview_graphics, upper_cp, lower_cp, fit_cache,
        chord_length, airfoil_to_world, y_axis_world
    )

