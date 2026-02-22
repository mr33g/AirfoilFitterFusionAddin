from __future__ import annotations

import numpy as np
from scipy import interpolate
import adsk.core

from core import config
from core.operations import (
    apply_knot_insertions,
    finalize_curves,
    fit_bspline as fit_bspline_op,
    fit_g1_independent,
    fit_single_surface_g1,
    fit_with_g2_optimization as fit_with_g2_optimization_op,
    refine_curve_with_knots as refine_curve_with_knots_op,
    refine_curves_with_surface_knots as refine_curves_with_surface_knots_op,
    refit_after_knot_insertion,
    validate_continuity,
    validate_trailing_edge_tangents,
)
from core.optimization import (
    vars_to_control_points,
)
from utils import bspline_helper

class BSplineProcessor:
    """
    B-spline processor with true G2 constraint that maintains G1 continuity.
    Works with arbitrary degree B-splines.
    """

    def __init__(self, degree: int = config.DEFAULT_CP_COUNT-1):
        self.upper_control_points: np.ndarray | None = None
        self.lower_control_points: np.ndarray | None = None
        self.upper_knot_vector: np.ndarray | None = None
        self.lower_knot_vector: np.ndarray | None = None
        self.upper_curve: interpolate.BSpline | None = None
        self.lower_curve: interpolate.BSpline | None = None
        self.degree: int = int(degree)
        self.degree_upper: int = int(degree)
        self.degree_lower: int = int(degree)
        self.fitted_degree: int | None = None  # Stores the actual degree used during fitting (may differ from self.degree if single span mode was used)
        self.num_cp_upper: int = 10  # Current number of control points for upper surface
        self.num_cp_lower: int = 10  # Current number of control points for lower surface
        self.param_exponent_upper: float = 0.5 # Mapping x -> u (0.5 is sqrt)
        self.param_exponent_lower: float = 0.5
        self.fitted: bool = False
        self.is_sharp_te: bool = False
        self.enforce_g2: bool = True
        self.enforce_g3: bool = False
        self.g2_weight: float = 100.0  # Weight for G2 constraint in optimization
        self.smoothing_weight: float = config.DEFAULT_SMOOTHNESS_PENALTY  # Weight for control point smoothing penalty
        # Tight insertion-mode SLSQP settings from tuned jacobian implementation.
        self.insertion_solver_ftol: float = 1e-10
        self.insertion_solver_maxiter_factor: float = 50.0
        self.insertion_solver_min_maxiter: int = 480
        self.last_optimizer_info: dict | None = None
        
        self.upper_original_data: np.ndarray | None = None
        self.lower_original_data: np.ndarray | None = None
        self.te_thickness_normalized: float = 0.0  # Normalized to chord (0-1)


    def fit_bspline(
        self,
        upper_data: np.ndarray,
        lower_data: np.ndarray,
        num_control_points: int | tuple[int, int],
        is_thickened: bool = False,
        upper_te_tangent_vector: np.ndarray | None = None,
        lower_te_tangent_vector: np.ndarray | None = None,
        enforce_g2: bool = False,
        enforce_g3: bool = False,
        enforce_te_tangency: bool = True,
        single_span: bool = False,
    ) -> bool:
        return fit_bspline_op(
            self,
            upper_data,
            lower_data,
            num_control_points,
            is_thickened=is_thickened,
            upper_te_tangent_vector=upper_te_tangent_vector,
            lower_te_tangent_vector=lower_te_tangent_vector,
            enforce_g2=enforce_g2,
            enforce_g3=enforce_g3,
            enforce_te_tangency=enforce_te_tangency,
            single_span=single_span,
        )

    def _fit_with_g2_optimization(
        self,
        upper_data: np.ndarray,
        lower_data: np.ndarray,
        num_control_points: int | tuple[int, int],
        upper_te_dir: np.ndarray | None,
        lower_te_dir: np.ndarray | None,
        enforce_te_tangency: bool = True,
        use_existing_knot_vectors: bool = False,
    ) -> bool:
        return fit_with_g2_optimization_op(
            self,
            upper_data,
            lower_data,
            num_control_points,
            upper_te_dir,
            lower_te_dir,
            enforce_te_tangency=enforce_te_tangency,
            use_existing_knot_vectors=use_existing_knot_vectors,
        )

    def _vars_to_control_points(self, vars: np.ndarray, num_cp_upper: int, num_cp_lower: int) -> tuple[np.ndarray, np.ndarray]:
        """Convert optimization variables to control points maintaining G1 constraints."""
        return vars_to_control_points(vars, num_cp_upper, num_cp_lower)

    def _fit_g1_independent(
        self,
        upper_data: np.ndarray,
        lower_data: np.ndarray,
        num_control_points: int | tuple[int, int],
        upper_te_dir: np.ndarray | None,
        lower_te_dir: np.ndarray | None,
        enforce_te_tangency: bool = True,
        use_existing_knot_vectors: bool = False,
    ):
        return fit_g1_independent(
            self,
            upper_data,
            lower_data,
            num_control_points,
            upper_te_dir,
            lower_te_dir,
            enforce_te_tangency=enforce_te_tangency,
            use_existing_knot_vectors=use_existing_knot_vectors,
        )

    def _fit_single_surface_g1(
        self,
        basis_matrix: np.ndarray,
        surface_data: np.ndarray,
        num_control_points: int,
        is_upper: bool,
        te_tangent_vector: np.ndarray | None = None,
        te_point: np.ndarray | None = None
    ) -> np.ndarray:
        return fit_single_surface_g1(
            self,
            basis_matrix,
            surface_data,
            num_control_points,
            is_upper,
            te_tangent_vector=te_tangent_vector,
            te_point=te_point,
        )

    def _finalize_curves(self):
        return finalize_curves(self)

    def _validate_trailing_edge_tangents(self, upper_te_dir: np.ndarray | None, lower_te_dir: np.ndarray | None) -> None:
        return validate_trailing_edge_tangents(self, upper_te_dir, lower_te_dir)

    def _validate_continuity(self):
        return validate_continuity(self)

    def is_fitted(self) -> bool:
        """Check if B-splines have been successfully fitted."""
        return self.fitted and self.upper_curve is not None and self.lower_curve is not None

    def calculate_curvature_comb_data(self, num_points_per_segment=200, scale_factor=0.050):
        """Calculate curvature comb visualization data."""
        if not self.is_fitted():
            return None
        
        return bspline_helper.calculate_curvature_comb_data(
            self.upper_curve, self.lower_curve, num_points_per_segment, scale_factor
        )

    def apply_te_thickening_parametric(self) -> bool:
        """
        Apply TE thickness to original data, then refit.
        Uses te_thickness_normalized parameter to calculate delta from original TE thickness.
        Always operates on original data to prevent error accumulation.

        Returns:
            bool: True if thickening was applied successfully, False otherwise
        """
        if not self.fitted or self.upper_original_data is None or self.lower_original_data is None:
            return False

        try:
            modified_upper = self.upper_original_data.copy()
            modified_lower = self.lower_original_data.copy()

            # Calculate delta from original TE thickness
            original_te = abs(self.upper_original_data[-1, 1] - self.lower_original_data[-1, 1])
            te_delta = self.te_thickness_normalized - original_te

            # Apply smoothstep blend if delta is non-zero
            if abs(te_delta) > 1e-9:
                half_delta = 0.5 * te_delta
                upper_x = np.clip(modified_upper[:, 0], 0.0, 1.0)
                lower_x = np.clip(modified_lower[:, 0], 0.0, 1.0)

                modified_upper[:, 1] += half_delta * bspline_helper.smoothstep_quintic(upper_x)
                modified_lower[:, 1] -= half_delta * bspline_helper.smoothstep_quintic(lower_x)

            # Refit with modified data
            if self.enforce_g2:
                success = self._fit_with_g2_optimization(
                    modified_upper, modified_lower,
                    (self.num_cp_upper, self.num_cp_lower),
                    upper_te_dir=None, lower_te_dir=None,
                    enforce_te_tangency=False,
                    use_existing_knot_vectors=False
                )
                if not success:
                    # Fallback to G1
                    self._fit_g1_independent(
                        modified_upper, modified_lower,
                        (self.num_cp_upper, self.num_cp_lower),
                        upper_te_dir=None, lower_te_dir=None,
                        enforce_te_tangency=False,
                        use_existing_knot_vectors=False
                    )
            else:
                self._fit_g1_independent(
                    modified_upper, modified_lower,
                    (self.num_cp_upper, self.num_cp_lower),
                    upper_te_dir=None, lower_te_dir=None,
                    enforce_te_tangency=False,
                    use_existing_knot_vectors=False
                )

            # Update sharp TE status based on final thickness
            self.is_sharp_te = (abs(self.te_thickness_normalized) < 1e-9)
            self._finalize_curves()

            return True

        except Exception as e:
            try:
                app = adsk.core.Application.get()
                app.log(f"Error in apply_te_thickening_parametric: {str(e)}")
            except:
                pass
            return False

    def insert_knot_at_max_error(self, surface: str, single_span: bool = False) -> bool:
        """
        Insert a knot at the location of maximum deviation on the specified surface.
        
        Args:
            surface: Which surface to insert knot into ('upper' or 'lower')
            single_span: If True, update degree to maintain single span mode (degree = CP - 1)
            
        Returns:
            bool: True if knot was inserted successfully, False otherwise
        """
        if not self.fitted or self.upper_curve is None or self.lower_curve is None:
            return False
        
        if self.upper_original_data is None or self.lower_original_data is None:
            return False
        
        try:
            # Determine target surface
            target_curve = self.upper_curve if surface == 'upper' else self.lower_curve
            target_data = self.upper_original_data if surface == 'upper' else self.lower_original_data
            target_knot_vector = self.upper_knot_vector if surface == 'upper' else self.lower_knot_vector
            current_param_exponent = self.param_exponent_upper if surface == 'upper' else self.param_exponent_lower
            
            # Use max-error-midspan policy for knot location.
            new_knot, adjusted_param_exponent = bspline_helper.prepare_knot_insertion_with_parameter_adjustment(
                target_curve,
                target_data,
                target_knot_vector,
                current_param_exponent,
                adjust_parameter_exponent=False,
            )
            
            # Update parameter exponent
            if surface == 'upper':
                self.param_exponent_upper = adjusted_param_exponent
            else:
                self.param_exponent_lower = adjusted_param_exponent
            
            # Insert the knot using the existing refine method
            # refine_curve_with_knots will automatically detect and maintain single span mode
            success = self.refine_curve_with_knots([new_knot], surface=surface)
            
            return success
            
        except Exception as e:
            try:
                app = adsk.core.Application.get()
                app.log(f"Error in insert_knot_at_max_error: {str(e)}")
            except:
                pass
            return False

    def _apply_knot_insertions(self, new_knots: list[float], surface: str | None = None) -> bool:
        return apply_knot_insertions(self, new_knots, surface=surface)

    def _refit_after_knot_insertion(self, *, use_existing_knot_vectors: bool) -> bool:
        return refit_after_knot_insertion(self, use_existing_knot_vectors=use_existing_knot_vectors)

    def refine_curve_with_knots(self, new_knots: list[float], surface: str | None = None) -> bool:
        """
        Refine the B-spline curves by inserting new knots.
        This operation preserves the curve's shape while increasing the number of control points.

        Args:
            new_knots: A list of parameter values (knots) to insert.
            surface: Which surface(s) to insert knots into. 
                     None (default) = both surfaces, 'upper' = upper only, 'lower' = lower only.

        Returns:
            bool: True if knots were inserted successfully, False otherwise.
        """
        return refine_curve_with_knots_op(self, new_knots, surface=surface)

    def refine_curves_with_surface_knots(
        self,
        *,
        upper_knots: list[float] | None = None,
        lower_knots: list[float] | None = None,
    ) -> bool:
        return refine_curves_with_surface_knots_op(self, upper_knots=upper_knots, lower_knots=lower_knots)
