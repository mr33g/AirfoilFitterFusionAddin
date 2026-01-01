from __future__ import annotations

import numpy as np
from scipy import interpolate, optimize

from core import config
from utils import bspline_helper

class BSplineProcessor:
    """
    B-spline processor with true G2 constraint that maintains G1 continuity.
    Works with arbitrary degree B-splines.
    """

    def __init__(self, degree: int = config.DEFAULT_BSPLINE_DEGREE):
        self.upper_control_points: np.ndarray | None = None
        self.lower_control_points: np.ndarray | None = None
        self.upper_knot_vector: np.ndarray | None = None
        self.lower_knot_vector: np.ndarray | None = None
        self.upper_curve: interpolate.BSpline | None = None
        self.lower_curve: interpolate.BSpline | None = None
        self.degree: int = int(degree)
        self.degree_upper: int = int(degree)
        self.degree_lower: int = int(degree)
        self.fitted_degree: int | None = None  
        self.num_cp_upper: int = 10  
        self.num_cp_lower: int = 10  
        self.param_exponent_upper: float = 0.5 
        self.param_exponent_lower: float = 0.5
        self.fitted: bool = False
        self.is_sharp_te: bool = False
        self.enforce_g2: bool = True
        self.enforce_g3: bool = False
        self.g2_weight: float = 100.0  
        self.smoothing_weight: float = config.DEFAULT_SMOOTHNESS_PENALTY  
        
        self.upper_original_data: np.ndarray | None = None
        self.lower_original_data: np.ndarray | None = None
        self._backup_upper_control_points: np.ndarray | None = None
        self._backup_lower_control_points: np.ndarray | None = None
        self._backup_upper_knot_vector: np.ndarray | None = None
        self._backup_lower_knot_vector: np.ndarray | None = None


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
        """Fit B-splines with G1 and optional G2 constraints at leading edge."""
        try:
            if isinstance(num_control_points, tuple):
                num_cp_upper, num_cp_lower = num_control_points
            else:
                num_cp_upper = num_cp_lower = num_control_points
            
            self.num_cp_upper = num_cp_upper
            self.num_cp_lower = num_cp_lower
            self.enforce_g2 = enforce_g2
            self.enforce_g3 = enforce_g3 if enforce_g2 else False

            if single_span:
                self.degree_upper = num_cp_upper - 1
                self.degree_lower = num_cp_lower - 1
            else:
                self.degree_upper = self.degree
                self.degree_lower = self.degree

            self.is_sharp_te = not is_thickened
            le_point = (upper_data[0] + lower_data[0]) / 2
            upper_data_corrected = upper_data.copy()
            lower_data_corrected = lower_data.copy()
            upper_data_corrected[0] = le_point
            lower_data_corrected[0] = le_point

            self.upper_original_data = upper_data_corrected.copy()
            self.lower_original_data = lower_data_corrected.copy()
            
            if self.is_sharp_te:
                te_point = np.array([1.0, 0.0])
                upper_data_corrected[-1] = te_point
                lower_data_corrected[-1] = te_point
            
            upper_te_dir = bspline_helper.normalize_vector(upper_te_tangent_vector)
            lower_te_dir = bspline_helper.normalize_vector(lower_te_tangent_vector)

            if self.enforce_g2:
                success = self._fit_with_g2_optimization(
                    upper_data_corrected, lower_data_corrected,
                    (self.num_cp_upper, self.num_cp_lower), upper_te_dir, lower_te_dir, enforce_te_tangency
                )
                if not success:
                    self.enforce_g2 = False
                    self.enforce_g3 = False
            
            if not self.enforce_g2:
                self._fit_g1_independent(
                    upper_data_corrected, lower_data_corrected,
                    (self.num_cp_upper, self.num_cp_lower), upper_te_dir, lower_te_dir, enforce_te_tangency
                )
            
            self._finalize_curves()
            self.fitted_degree = (self.degree_upper, self.degree_lower)
            self.fitted = True
            return True
            
        except Exception:
            self.fitted = False
            return False

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
        te_point_upper = upper_data[-1]
        te_point_lower = lower_data[-1]
        
        u_params_upper = bspline_helper.create_parameter_from_x_coords(upper_data, self.param_exponent_upper)
        u_params_lower = bspline_helper.create_parameter_from_x_coords(lower_data, self.param_exponent_lower)
        
        if not use_existing_knot_vectors:
            self.upper_knot_vector = bspline_helper.create_knot_vector(self.num_cp_upper, self.degree_upper)
            self.lower_knot_vector = bspline_helper.create_knot_vector(self.num_cp_lower, self.degree_lower)

        if self.upper_knot_vector is None or self.lower_knot_vector is None:
             raise ValueError("Knot vectors are None")

        num_cp_upper = len(self.upper_knot_vector) - self.degree_upper - 1
        num_cp_lower = len(self.lower_knot_vector) - self.degree_lower - 1

        self._fit_g1_independent(upper_data, lower_data, (num_cp_upper, num_cp_lower), upper_te_dir, lower_te_dir, enforce_te_tangency, use_existing_knot_vectors)
        
        initial_vars = []
        initial_vars.append(self.upper_control_points[1, 1])
        initial_vars.append(self.lower_control_points[1, 1])
        initial_vars.append(self.upper_control_points[2, 0])
        initial_vars.append(self.upper_control_points[2, 1])
        initial_vars.append(self.lower_control_points[2, 0])
        initial_vars.append(self.lower_control_points[2, 1])
        
        for i in range(3, num_cp_upper):
            initial_vars.extend([self.upper_control_points[i, 0], self.upper_control_points[i, 1]])
        for i in range(3, num_cp_lower):
            initial_vars.extend([self.lower_control_points[i, 0], self.lower_control_points[i, 1]])
        
        initial_vars = np.array(initial_vars)
        
        def objective(vars):
            cp_upper, cp_lower = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
            curve_upper = interpolate.BSpline(self.upper_knot_vector, cp_upper, self.degree_upper)
            curve_lower = interpolate.BSpline(self.lower_knot_vector, cp_lower, self.degree_lower)
            fitted_upper = np.array([curve_upper(u) for u in u_params_upper])
            fitted_lower = np.array([curve_lower(u) for u in u_params_lower])
            error_upper = np.sum((upper_data - fitted_upper)**2)
            error_lower = np.sum((lower_data - fitted_lower)**2)
            smoothing_penalty = 0.0
            for i in range(num_cp_upper - 2):
                gradient = 0.5 + 1.5 * (i / (num_cp_upper - 3)) if num_cp_upper > 3 else 1.0
                diff_upper = cp_upper[i+2] - 2 * cp_upper[i+1] + cp_upper[i]
                smoothing_penalty += np.sum(diff_upper**2) * (gradient**2)
            for i in range(num_cp_lower - 2):
                gradient = 0.5 + 1.5 * (i / (num_cp_lower - 3)) if num_cp_lower > 3 else 1.0
                diff_lower = cp_lower[i+2] - 2 * cp_lower[i+1] + cp_lower[i]
                smoothing_penalty += np.sum(diff_lower**2) * (gradient**2)
            return error_upper + error_lower + self.smoothing_weight * smoothing_penalty
        
        def curvature_constraint(vars):
            cp_upper, cp_lower = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
            kappa_upper = bspline_helper.compute_curvature_at_zero(cp_upper, self.upper_knot_vector, self.degree_upper)
            kappa_lower = bspline_helper.compute_curvature_at_zero(cp_lower, self.lower_knot_vector, self.degree_lower)
            return kappa_upper - kappa_lower

        def curvature_derivative_constraint(vars):
            cp_upper, cp_lower = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
            dk_upper = bspline_helper.compute_curvature_derivative_at_zero(cp_upper, self.upper_knot_vector, self.degree_upper)
            dk_lower = bspline_helper.compute_curvature_derivative_at_zero(cp_lower, self.lower_knot_vector, self.degree_lower)
            return dk_upper - dk_lower
        
        constraints = [{'type': 'eq', 'fun': curvature_constraint}]
        if self.enforce_g3:
            constraints.append({'type': 'eq', 'fun': curvature_derivative_constraint})
        
        def te_constraint_upper(vars):
            cp_upper, _ = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
            return cp_upper[-1] - te_point_upper
        
        def te_constraint_lower(vars):
            _, cp_lower = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
            return cp_lower[-1] - te_point_lower
        
        constraints.extend([
            {'type': 'eq', 'fun': te_constraint_upper},
            {'type': 'eq', 'fun': te_constraint_lower}
        ])
        
        if upper_te_dir is not None and lower_te_dir is not None and enforce_te_tangency:
            def te_tangent_constraint_upper(vars):
                cp_upper, _ = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
                computed_tangent = bspline_helper.compute_tangent_at_trailing_edge(cp_upper, self.upper_knot_vector, self.degree_upper)
                return computed_tangent - upper_te_dir
            def te_tangent_constraint_lower(vars):
                _, cp_lower = self._vars_to_control_points(vars, num_cp_upper, num_cp_lower)
                computed_tangent = bspline_helper.compute_tangent_at_trailing_edge(cp_lower, self.lower_knot_vector, self.degree_lower)
                return computed_tangent - lower_te_dir
            constraints.extend([
                {'type': 'eq', 'fun': te_tangent_constraint_upper},
                {'type': 'eq', 'fun': te_tangent_constraint_lower}
            ])
        
        bounds = []
        bounds.append((0.001, 0.1))   # P1.y_upper
        bounds.append((-0.1, -0.001)) # P1.y_lower
        bounds.append((0.001, 0.5))   # P2.x_upper
        bounds.append((0.001, 0.3))   # P2.y_upper
        bounds.append((0.001, 0.5))   # P2.x_lower
        bounds.append((-0.3, -0.001)) # P2.y_lower
        
        for _ in range(num_cp_upper - 3):
            bounds.extend([(None, None), (None, None)])
        for _ in range(num_cp_lower - 3):
            bounds.extend([(None, None), (None, None)])
        
        max_deg = max(self.degree_upper, self.degree_lower)
        max_iter = max(200, len(initial_vars) * 20)
        if max_deg > 10:
            max_iter += (max_deg - 10) * 100
            
        result = optimize.minimize(
            objective, initial_vars,
            method='SLSQP',
            constraints=constraints,
            bounds=bounds,
            options={'ftol': 1e-7, 'maxiter': max_iter, 'disp': False}
        )
        
        if result.success or result.status == 0:
            self.upper_control_points, self.lower_control_points = \
                self._vars_to_control_points(result.x, num_cp_upper, num_cp_lower)
            return True
        return False

    def _vars_to_control_points(self, vars: np.ndarray, num_cp_upper: int, num_cp_lower: int) -> tuple[np.ndarray, np.ndarray]:
        cp_upper = np.zeros((num_cp_upper, 2))
        cp_lower = np.zeros((num_cp_lower, 2))
        cp_upper[0] = [0.0, 0.0]
        cp_lower[0] = [0.0, 0.0]
        cp_upper[1] = [0.0, vars[0]]
        cp_lower[1] = [0.0, vars[1]]
        cp_upper[2] = [vars[2], vars[3]]
        cp_lower[2] = [vars[4], vars[5]]
        idx = 6
        for i in range(3, num_cp_upper):
            cp_upper[i] = vars[idx:idx+2]
            idx += 2
        for i in range(3, num_cp_lower):
            cp_lower[i] = vars[idx:idx+2]
            idx += 2
        return cp_upper, cp_lower

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
        te_point_upper = upper_data[-1]
        te_point_lower = lower_data[-1]
        
        u_params_upper = bspline_helper.create_parameter_from_x_coords(upper_data, self.param_exponent_upper)
        u_params_lower = bspline_helper.create_parameter_from_x_coords(lower_data, self.param_exponent_lower)
        
        if not use_existing_knot_vectors:
            self.upper_knot_vector = bspline_helper.create_knot_vector(self.num_cp_upper, self.degree_upper)
            self.lower_knot_vector = bspline_helper.create_knot_vector(self.num_cp_lower, self.degree_lower)

        basis_upper = bspline_helper.build_basis_matrix(u_params_upper, self.upper_knot_vector, self.degree_upper)
        basis_lower = bspline_helper.build_basis_matrix(u_params_lower, self.lower_knot_vector, self.degree_lower)
        
        num_control_points_upper = len(self.upper_knot_vector) - self.degree_upper - 1
        num_control_points_lower = len(self.lower_knot_vector) - self.degree_lower - 1
        
        self.upper_control_points = self._fit_single_surface_g1(
            basis_upper, upper_data, num_control_points_upper, is_upper=True, 
            te_tangent_vector=upper_te_dir if enforce_te_tangency else None, te_point=te_point_upper
        )
        self.lower_control_points = self._fit_single_surface_g1(
            basis_lower, lower_data, num_control_points_lower, is_upper=False, 
            te_tangent_vector=lower_te_dir if enforce_te_tangency else None, te_point=te_point_lower
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
        A_data = np.zeros((2 * len(surface_data), 2 * num_control_points))
        b_data = np.zeros(2 * len(surface_data))
        A_data[:len(surface_data), :num_control_points] = basis_matrix
        b_data[:len(surface_data)] = surface_data[:, 0]
        A_data[len(surface_data):, num_control_points:] = basis_matrix
        b_data[len(surface_data):] = surface_data[:, 1]
        
        constraints = []
        constraint_rhs = []
        row = np.zeros(2 * num_control_points)
        row[0] = 1.0; constraints.append(row); constraint_rhs.append(0.0)
        row = np.zeros(2 * num_control_points)
        row[num_control_points] = 1.0; constraints.append(row); constraint_rhs.append(0.0)
        row = np.zeros(2 * num_control_points)
        row[1] = 1.0; constraints.append(row); constraint_rhs.append(0.0)
        
        if te_tangent_vector is not None:
            row = np.zeros(2 * num_control_points)
            row[num_control_points - 1] = -te_tangent_vector[1]
            row[2 * num_control_points - 1] = te_tangent_vector[0]
            row[num_control_points - 2] = te_tangent_vector[1]
            row[2 * num_control_points - 2] = -te_tangent_vector[0]
            constraints.append(row); constraint_rhs.append(0.0)
        
        if te_point is not None:
            row_x = np.zeros(2 * num_control_points)
            row_x[num_control_points - 1] = 1.0; constraints.append(row_x); constraint_rhs.append(te_point[0])
            row_y = np.zeros(2 * num_control_points)
            row_y[2 * num_control_points - 1] = 1.0; constraints.append(row_y); constraint_rhs.append(te_point[1])
        
        constraint_weight = 1000.0
        A_constraints = np.array(constraints) * constraint_weight
        b_constraints = np.array(constraint_rhs) * constraint_weight

        A_smoothing = np.zeros(((num_control_points - 2) * 2, 2 * num_control_points))
        b_smoothing = np.zeros((num_control_points - 2) * 2)
        for i in range(num_control_points - 2):
            gradient = 0.5 + 1.5 * (i / (num_control_points - 3)) if num_control_points > 3 else 1.0
            current_weight = self.smoothing_weight * gradient
            A_smoothing[i, i] = current_weight
            A_smoothing[i, i+1] = -2 * current_weight
            A_smoothing[i, i+2] = current_weight
            A_smoothing[i + (num_control_points - 2), num_control_points + i] = current_weight
            A_smoothing[i + (num_control_points - 2), num_control_points + i+1] = -2 * current_weight
            A_smoothing[i + (num_control_points - 2), num_control_points + i+2] = current_weight

        A_all = np.vstack([A_data, A_constraints, A_smoothing])
        b_all = np.hstack([b_data, b_constraints, b_smoothing])
        solution = np.linalg.lstsq(A_all, b_all, rcond=None)[0]
        control_points = np.zeros((num_control_points, 2))
        control_points[:, 0] = solution[:num_control_points]
        control_points[:, 1] = solution[num_control_points:]
        control_points[0] = [0.0, 0.0]
        control_points[1, 0] = 0.0
        if is_upper and control_points[1, 1] < 0:
            control_points[1, 1] = abs(control_points[1, 1])
        elif not is_upper and control_points[1, 1] > 0:
            control_points[1, 1] = -abs(control_points[1, 1])
        return control_points

    def _finalize_curves(self):
        if self.upper_control_points is not None and self.lower_control_points is not None:
            shared_p0 = (self.upper_control_points[0] + self.lower_control_points[0]) / 2
            self.upper_control_points[0] = shared_p0
            self.lower_control_points[0] = shared_p0
            self.upper_control_points[0] = [0.0, 0.0]
            self.lower_control_points[0] = [0.0, 0.0]
            self.upper_control_points[1, 0] = 0.0
            self.lower_control_points[1, 0] = 0.0
            if self.is_sharp_te:
                te_point = np.array([1.0, 0.0])
                self.upper_control_points[-1] = te_point
                self.lower_control_points[-1] = te_point
        
        if self.upper_control_points is not None and self.upper_knot_vector is not None:
            self.upper_curve = interpolate.BSpline(self.upper_knot_vector, self.upper_control_points, self.degree_upper)
        if self.lower_control_points is not None and self.lower_knot_vector is not None:
            self.lower_curve = interpolate.BSpline(self.lower_knot_vector, self.lower_control_points, self.degree_lower)

    def is_fitted(self) -> bool:
        return self.fitted and self.upper_curve is not None and self.lower_curve is not None

    def apply_te_thickening(self, te_thickness: float) -> bool:
        if not self.fitted or self.upper_control_points is None or self.lower_control_points is None:
            return False
        if te_thickness < 0.0:
            return False
        try:
            self._backup_upper_control_points = self.upper_control_points.copy()
            self._backup_lower_control_points = self.lower_control_points.copy()
            self._backup_upper_knot_vector = None if self.upper_knot_vector is None else self.upper_knot_vector.copy()
            self._backup_lower_knot_vector = None if self.lower_knot_vector is None else self.lower_knot_vector.copy()
            if self.upper_curve is None or self.lower_curve is None:
                return False
            num_samples = max(200, int(config.PLOT_POINTS_PER_SURFACE))
            _, upper_pts = bspline_helper.sample_curve(self.upper_curve, num_samples)
            _, lower_pts = bspline_helper.sample_curve(self.lower_curve, num_samples)
            f_upper = bspline_helper.smoothstep_quintic(np.clip(upper_pts[:, 0], 0.0, 1.0))
            f_lower = bspline_helper.smoothstep_quintic(np.clip(lower_pts[:, 0], 0.0, 1.0))
            half_thickness = 0.5 * te_thickness
            thick_upper = upper_pts.copy()
            thick_lower = lower_pts.copy()
            thick_upper[:, 1] += half_thickness * f_upper
            thick_lower[:, 1] -= half_thickness * f_lower
            self._fit_g1_independent(thick_upper, thick_lower, (self.num_cp_upper, self.num_cp_lower), None, None, False)
            self.is_sharp_te = False
            self._finalize_curves()
            self.fitted = True
            return True
        except Exception:
            return False

    def remove_te_thickening(self) -> bool:
        if not self.fitted or self.upper_control_points is None or self.lower_control_points is None:
            return False
        try:
            if self._backup_upper_control_points is not None and self._backup_lower_control_points is not None:
                self.upper_control_points = self._backup_upper_control_points
                self.lower_control_points = self._backup_lower_control_points
                if self._backup_upper_knot_vector is not None:
                    self.upper_knot_vector = self._backup_upper_knot_vector
                if self._backup_lower_knot_vector is not None:
                    self.lower_knot_vector = self._backup_lower_knot_vector
                if self.upper_knot_vector is not None:
                    self.upper_curve = interpolate.BSpline(self.upper_knot_vector, self.upper_control_points, self.degree_upper)
                if self.lower_knot_vector is not None:
                    self.lower_curve = interpolate.BSpline(self.lower_knot_vector, self.lower_control_points, self.degree_lower)
                self._backup_upper_control_points = None
                self._backup_lower_control_points = None
                self._backup_upper_knot_vector = None
                self._backup_lower_knot_vector = None
                self.is_sharp_te = bool(np.allclose(self.upper_control_points[-1], self.lower_control_points[-1], atol=1e-12))
                return True
            return False
        except Exception:
            return False

    def refine_curve_with_knots(self, new_knots: list[float], surface: str | None = None) -> bool:
        if not self.fitted or self.upper_curve is None or self.lower_curve is None:
            return False
        if not new_knots:
            return True
        try:
            sorted_new_knots = sorted(new_knots)
            if surface is None or surface == 'upper':
                current_upper_cps = self.upper_control_points.copy()
                current_upper_knots = self.upper_knot_vector.copy()
                for knot in sorted_new_knots:
                    current_upper_cps, current_upper_knots = bspline_helper.insert_knot(current_upper_cps, current_upper_knots, self.degree_upper, knot)
                self.upper_control_points = current_upper_cps
                self.upper_knot_vector = current_upper_knots
                self.num_cp_upper = len(self.upper_control_points)
            if surface is None or surface == 'lower':
                current_lower_cps = self.lower_control_points.copy()
                current_lower_knots = self.lower_knot_vector.copy()
                for knot in sorted_new_knots:
                    current_lower_cps, current_lower_knots = bspline_helper.insert_knot(current_lower_cps, current_lower_knots, self.degree_lower, knot)
                self.lower_control_points = current_lower_cps
                self.lower_knot_vector = current_lower_knots
                self.num_cp_lower = len(self.lower_control_points)
            if self.upper_original_data is not None and self.lower_original_data is not None:
                cp_counts = (self.num_cp_upper, self.num_cp_lower)
                if self.enforce_g2:
                    success = self._fit_with_g2_optimization(self.upper_original_data, self.lower_original_data, cp_counts, None, None, False, True)
                    if not success:
                        self._fit_g1_independent(self.upper_original_data, self.lower_original_data, cp_counts, None, None, False, True)
                else:
                    self._fit_g1_independent(self.upper_original_data, self.lower_original_data, cp_counts, None, None, False, True)
            self._finalize_curves()
            return True
        except Exception:
            return False
