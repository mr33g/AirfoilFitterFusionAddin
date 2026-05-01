from __future__ import annotations

import numpy as np
from scipy import interpolate

from utils import bspline_helper


def build_basis_derivative_matrix(
    t_values: np.ndarray,
    knot_vector: np.ndarray,
    degree: int,
) -> np.ndarray:
    if degree <= 0:
        num_basis = max(0, len(knot_vector) - degree - 1)
        return np.zeros((len(t_values), num_basis), dtype=float)

    lower_basis = bspline_helper.build_basis_matrix(t_values, knot_vector, degree - 1)
    num_basis = len(knot_vector) - degree - 1
    deriv = np.zeros((len(t_values), num_basis), dtype=float)
    p = float(degree)
    for i in range(num_basis):
        left = 0.0
        right = 0.0
        left_denom = float(knot_vector[i + degree] - knot_vector[i])
        if abs(left_denom) > 1e-15:
            left = p / left_denom
        right_denom = float(knot_vector[i + degree + 1] - knot_vector[i + 1])
        if abs(right_denom) > 1e-15:
            right = p / right_denom
        deriv[:, i] = left * lower_basis[:, i] - right * lower_basis[:, i + 1]
    return deriv


def solve_u_for_x_targets(
    x_targets: np.ndarray,
    control_points: np.ndarray,
    knot_vector: np.ndarray,
    degree: int,
    *,
    initial_u: np.ndarray | None = None,
    exponent_guess: float = 0.5,
    newton_iters: int = 8,
    bracket_samples: int = 65,
    bisect_iters: int = 24,
    x_tol: float = 1e-11,
) -> np.ndarray:
    x_targets_arr = np.clip(np.asarray(x_targets, dtype=float), 0.0, 1.0)
    if x_targets_arr.size == 0:
        return np.zeros((0,), dtype=float)

    curve = interpolate.BSpline(knot_vector, control_points, degree)
    deriv_curve = curve.derivative(1)
    u_min = float(knot_vector[degree])
    u_max = float(knot_vector[-(degree + 1)])
    u_eps = max(1e-12, 1e-12 * max(1.0, abs(u_max - u_min)))
    u_hi = max(u_min, u_max - u_eps)

    if initial_u is None or len(initial_u) != len(x_targets_arr):
        u = np.power(np.clip(x_targets_arr, 0.0, 1.0 - 1e-12), exponent_guess)
    else:
        u = np.asarray(initial_u, dtype=float).copy()
    u = np.clip(u, u_min, u_hi)

    for _ in range(max(1, int(newton_iters))):
        curve_points = curve(u)
        f = curve_points[:, 0] - x_targets_arr
        if np.all(np.abs(f) <= x_tol):
            return np.clip(u, u_min, u_hi)
        dx_du = deriv_curve(u)[:, 0]
        good = np.abs(dx_du) > 1e-12
        if not np.any(good):
            break
        u_next = u.copy()
        u_next[good] = np.clip(u[good] - (f[good] / dx_du[good]), u_min, u_hi)
        u = u_next

    sample_u = np.linspace(u_min, u_hi, max(3, int(bracket_samples)))
    sample_x = curve(sample_u)[:, 0]
    sort_idx = np.argsort(sample_x)
    sample_x_sorted = sample_x[sort_idx]
    sample_u_sorted = sample_u[sort_idx]

    for idx_point, x_target in enumerate(x_targets_arr):
        x_at_u = float(curve(u[idx_point])[0])
        if abs(x_at_u - x_target) <= x_tol:
            continue

        insert_idx = int(np.searchsorted(sample_x_sorted, x_target))
        left_idx = max(0, min(insert_idx - 1, len(sample_x_sorted) - 1))
        right_idx = max(0, min(insert_idx, len(sample_x_sorted) - 1))
        if left_idx == right_idx:
            if right_idx < len(sample_x_sorted) - 1:
                right_idx += 1
            elif left_idx > 0:
                left_idx -= 1

        ua = float(min(sample_u_sorted[left_idx], sample_u_sorted[right_idx]))
        ub = float(max(sample_u_sorted[left_idx], sample_u_sorted[right_idx]))
        fa = float(curve(ua)[0] - x_target)
        fb = float(curve(ub)[0] - x_target)

        if abs(fa) <= x_tol:
            u[idx_point] = ua
            continue
        if abs(fb) <= x_tol:
            u[idx_point] = ub
            continue
        if fa * fb > 0.0:
            u[idx_point] = ua if abs(fa) <= abs(fb) else ub
            continue

        left_u = ua
        right_u = ub
        left_f = fa
        for _ in range(max(1, int(bisect_iters))):
            mid_u = 0.5 * (left_u + right_u)
            mid_f = float(curve(mid_u)[0] - x_target)
            if abs(mid_f) <= x_tol or abs(right_u - left_u) <= u_eps:
                left_u = mid_u
                right_u = mid_u
                break
            if left_f * mid_f <= 0.0:
                right_u = mid_u
            else:
                left_u = mid_u
                left_f = mid_f
        u[idx_point] = 0.5 * (left_u + right_u)

    return np.clip(u, u_min, u_hi)


def vertical_distance_and_grad(
    data_points: np.ndarray,
    control_points: np.ndarray,
    knot_vector: np.ndarray,
    degree: int,
    *,
    initial_u: np.ndarray | None = None,
    exponent_guess: float = 0.5,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    num_cp = int(control_points.shape[0])
    if data_points.size == 0 or control_points.size == 0:
        return (
            0.0,
            np.zeros((num_cp, 2), dtype=float),
            np.zeros((0,), dtype=float),
            np.zeros((0,), dtype=float),
        )

    solved_u = solve_u_for_x_targets(
        data_points[:, 0],
        control_points,
        knot_vector,
        degree,
        initial_u=initial_u,
        exponent_guess=exponent_guess,
    )
    basis = bspline_helper.build_basis_matrix(solved_u, knot_vector, degree)
    basis_d1 = build_basis_derivative_matrix(solved_u, knot_vector, degree)

    y_curve = basis @ control_points[:, 1]
    y_residual = y_curve - data_points[:, 1]
    error = float(np.sum(y_residual * y_residual))

    grad_cp = np.zeros((num_cp, 2), dtype=float)
    grad_cp[:, 1] = 2.0 * (basis.T @ y_residual)

    dx_du = basis_d1 @ control_points[:, 0]
    dy_du = basis_d1 @ control_points[:, 1]
    safe_dx_du = np.where(
        np.abs(dx_du) > 1e-12,
        dx_du,
        np.where(dx_du >= 0.0, 1e-12, -1e-12),
    )
    x_weight = 2.0 * y_residual * (-dy_du / safe_dx_du)
    grad_cp[:, 0] = basis.T @ x_weight

    return error, grad_cp, solved_u, y_residual
