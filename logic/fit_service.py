"""Headless adapter from Airfoil Fitter settings to the shared numerical core."""
from copy import deepcopy
from functools import lru_cache
import math
from pathlib import Path

import airfoil_fitter_settings as config
from airfoil_splines_core import bspline_helper
from airfoil_splines_core.airfoil_processor import AirfoilProcessor
from airfoil_splines_core.bspline_helper import apply_te_thickness_to_reference
from airfoil_splines_core.fit_recovery import fit_with_recovery


class AirfoilFitError(RuntimeError):
    """The requested fit and all permitted fallback attempts failed."""


def read_source(path):
    return Path(path).read_text()


def validate(values, chord_length):
    if not math.isfinite(chord_length) or chord_length <= 1e-9:
        raise ValueError('The chord must have a positive length.')
    keys = ('te', 'smoothness', 'upper_count', 'lower_count', 'continuity')
    if not all(math.isfinite(values[key]) for key in keys):
        raise ValueError('Airfoil parameters must be finite.')
    if values['te'] < 0 or values['smoothness'] < 0:
        raise ValueError('TE thickness and smoothness must not be negative.')
    for key in ('upper_count', 'lower_count'):
        count = values[key]
        if count != int(count) or not config.MIN_CP_COUNT <= count <= config.MAX_CP_COUNT:
            raise ValueError(
                f'Control point counts must be whole numbers from '
                f'{config.MIN_CP_COUNT} to {config.MAX_CP_COUNT}.')
    if values['continuity'] not in (1, 2, 3):
        raise ValueError('LE continuity must be 1, 2 or 3.')


@lru_cache(maxsize=16)
def _load_source(text):
    processor = AirfoilProcessor(logger_func=lambda _message: None)
    if not processor.load_text(text):
        raise ValueError('Cannot load the airfoil data.')
    return processor


def source_te_thickness(text):
    return _load_source(text).get_te_thickness()


def fit_data(text, values, chord_length):
    """Fit legacy single-span Béziers through the shared core package."""
    validate(values, chord_length)
    data = deepcopy(_fit_cached(
        text, values['te'] / chord_length, values['smoothness'],
        int(values['upper_count']), int(values['lower_count']),
        int(values['continuity'])))
    values['smoothness'] = data['effective_smoothness']
    values['continuity'] = data['effective_continuity']
    return data


@lru_cache(maxsize=64)
def _fit_cached(text, te, smoothness, upper_count, lower_count, continuity):
    processor = _load_source(text)
    upper, lower = apply_te_thickness_to_reference(
        processor.upper_data, processor.lower_data, te)

    # Degree count-1 retains Airfoil Fitter's one-Bézier-per-surface contract.
    degrees = (upper_count - 1, lower_count - 1)
    recovery = fit_with_recovery(
        degrees, upper, lower, (upper_count, lower_count),
        smoothing_weight=smoothness, continuity=continuity,
        upper_te_tangent_vector=processor.upper_te_tangent_vector,
        lower_te_tangent_vector=processor.lower_te_tangent_vector)
    if not recovery:
        raise AirfoilFitError(recovery.failure_message)

    bspline = recovery.processor
    raw_upper, raw_lower = processor.error_reference_data()
    error_upper, error_lower = apply_te_thickness_to_reference(raw_upper, raw_lower, te)

    def error(curve, reference, exponent):
        _, maximum, index, _ = bspline_helper.calculate_bspline_fitting_error(
            curve, reference, param_exponent=exponent, return_max_error=True)
        return maximum, reference[index].copy()

    err_u, point_u = error(
        bspline.upper_curve, error_upper, bspline.param_exponent_upper)
    err_l, point_l = error(
        bspline.lower_curve, error_lower, bspline.param_exponent_lower)
    return dict(
        upper_cp_raw=bspline.upper_control_points,
        lower_cp_raw=bspline.lower_control_points,
        upper_knots=bspline.upper_knot_vector,
        lower_knots=bspline.lower_knot_vector,
        degree_u=bspline.degree_upper,
        degree_l=bspline.degree_lower,
        is_sharp=bspline.is_sharp_te,
        err_u=err_u,
        err_l=err_l,
        max_err_pt_u=point_u,
        max_err_pt_l=point_l,
        raw_upper=raw_upper,
        raw_lower=raw_lower,
        error_upper=error_upper,
        error_lower=error_lower,
        fit_upper=upper,
        fit_lower=lower,
        param_exponent_upper=bspline.param_exponent_upper,
        param_exponent_lower=bspline.param_exponent_lower,
        te_value=te,
        requested_smoothness=float(smoothness),
        requested_continuity=int(continuity),
        effective_smoothness=recovery.effective_smoothing,
        effective_continuity=recovery.effective_continuity,
        fallback_used=recovery.fallback_used,
        enforce_g2=recovery.effective_continuity >= 2,
        enforce_g3=recovery.effective_continuity == 3,
        fit_result=recovery.result)

