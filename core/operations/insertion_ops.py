from __future__ import annotations

import adsk.core
import numpy as np

from utils import bspline_helper


def apply_knot_insertions(proc, new_knots: list[float], surface: str | None = None) -> bool:
    """Apply knot insertion to selected surfaces without re-fitting/finalizing.

    Returns True when degree changed due to single-span mode preservation.
    """
    if not new_knots:
        return False

    sorted_new_knots = sorted(new_knots)
    was_single_span_upper = (proc.degree_upper == proc.num_cp_upper - 1) if proc.num_cp_upper > 0 else False
    was_single_span_lower = (proc.degree_lower == proc.num_cp_lower - 1) if proc.num_cp_lower > 0 else False
    degree_changed = False

    if surface is None or surface == "upper":
        current_upper_cps = proc.upper_control_points.copy()
        current_upper_knots = proc.upper_knot_vector.copy()
        for knot in sorted_new_knots:
            current_upper_cps, current_upper_knots = bspline_helper.insert_knot(
                current_upper_cps, current_upper_knots, proc.degree_upper, knot
            )
        proc.upper_control_points = current_upper_cps
        proc.upper_knot_vector = current_upper_knots
        proc.num_cp_upper = len(proc.upper_control_points)
        if was_single_span_upper:
            proc.degree_upper = proc.num_cp_upper - 1
            degree_changed = True

    if surface is None or surface == "lower":
        current_lower_cps = proc.lower_control_points.copy()
        current_lower_knots = proc.lower_knot_vector.copy()
        for knot in sorted_new_knots:
            current_lower_cps, current_lower_knots = bspline_helper.insert_knot(
                current_lower_cps, current_lower_knots, proc.degree_lower, knot
            )
        proc.lower_control_points = current_lower_cps
        proc.lower_knot_vector = current_lower_knots
        proc.num_cp_lower = len(proc.lower_control_points)
        if was_single_span_lower:
            proc.degree_lower = proc.num_cp_lower - 1
            degree_changed = True

    return degree_changed


def refit_after_knot_insertion(proc, *, use_existing_knot_vectors: bool) -> bool:
    """Re-fit with current CP counts and selected knot-vector strategy."""
    if proc.upper_original_data is None or proc.lower_original_data is None:
        return True

    cp_counts = (proc.num_cp_upper, proc.num_cp_lower)
    if proc.enforce_g2:
        success = proc._fit_with_g2_optimization(
            proc.upper_original_data,
            proc.lower_original_data,
            cp_counts,
            upper_te_dir=None,
            lower_te_dir=None,
            enforce_te_tangency=False,
            use_existing_knot_vectors=use_existing_knot_vectors,
        )
        if not success:
            proc._fit_g1_independent(
                proc.upper_original_data,
                proc.lower_original_data,
                cp_counts,
                upper_te_dir=None,
                lower_te_dir=None,
                enforce_te_tangency=False,
                use_existing_knot_vectors=use_existing_knot_vectors,
            )
    else:
        proc._fit_g1_independent(
            proc.upper_original_data,
            proc.lower_original_data,
            cp_counts,
            upper_te_dir=None,
            lower_te_dir=None,
            enforce_te_tangency=False,
            use_existing_knot_vectors=use_existing_knot_vectors,
        )
    return True


def refine_curve_with_knots(proc, new_knots: list[float], surface: str | None = None) -> bool:
    if not proc.fitted or proc.upper_curve is None or proc.lower_curve is None:
        return False
    if not new_knots:
        return True
    try:
        degree_changed = apply_knot_insertions(proc, new_knots, surface=surface)
        refit_after_knot_insertion(proc, use_existing_knot_vectors=not degree_changed)
        proc._finalize_curves()
        proc._validate_continuity()
        return True
    except Exception as exc:
        try:
            app = adsk.core.Application.get()
            app.log(f"Error in refine_curve_with_knots: {exc}")
        except Exception:
            pass
        return False


def refine_curves_with_surface_knots(
    proc,
    *,
    upper_knots: list[float] | None = None,
    lower_knots: list[float] | None = None,
) -> bool:
    if not proc.fitted or proc.upper_curve is None or proc.lower_curve is None:
        return False

    upper_knots = upper_knots or []
    lower_knots = lower_knots or []
    if not upper_knots and not lower_knots:
        return True

    try:
        degree_changed = False
        if upper_knots:
            degree_changed = apply_knot_insertions(proc, upper_knots, surface="upper") or degree_changed
        if lower_knots:
            degree_changed = apply_knot_insertions(proc, lower_knots, surface="lower") or degree_changed

        refit_after_knot_insertion(proc, use_existing_knot_vectors=not degree_changed)
        proc._finalize_curves()
        proc._validate_continuity()
        return True
    except Exception as exc:
        try:
            app = adsk.core.Application.get()
            app.log(f"Error in refine_curves_with_surface_knots: {exc}")
        except Exception:
            pass
        return False
