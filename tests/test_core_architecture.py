"""Contract tests for the adapter between Fusion AF and the shared core."""
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from logic import fit_service, state


class SharedCoreAdapterTests(unittest.TestCase):
    def setUp(self):
        fit_service._fit_cached.cache_clear()
        fit_service._load_source.cache_clear()

    def test_legacy_bezier_degrees_are_derived_from_each_surface_count(self):
        surface = np.array([[0.0, 0.0], [0.5, 0.1], [1.0, 0.0]])
        processor = NS(
            upper_data=surface,
            lower_data=surface * np.array([1.0, -1.0]),
            upper_te_tangent_vector=None,
            lower_te_tangent_vector=None,
            error_reference_data=lambda: (surface.copy(), surface.copy()),
        )
        model = NS(
            upper_control_points=np.zeros((7, 2)),
            lower_control_points=np.zeros((9, 2)),
            upper_knot_vector=np.arange(14, dtype=float),
            lower_knot_vector=np.arange(18, dtype=float),
            degree_upper=6,
            degree_lower=8,
            upper_curve=object(),
            lower_curve=object(),
            param_exponent_upper=0.5,
            param_exponent_lower=0.5,
            is_sharp_te=True,
        )
        recovery = NS(
            processor=model,
            effective_smoothing=0.001,
            effective_continuity=2,
            fallback_used=False,
            result=NS(success=True),
            __bool__=lambda self: True,
        )
        values = dict(te=0.0, smoothness=0.001, upper_count=7,
                      lower_count=9, continuity=2)

        with patch.object(fit_service, '_load_source', return_value=processor), \
             patch.object(fit_service, 'fit_with_recovery', return_value=recovery) as fit, \
             patch.object(fit_service.bspline_helper,
                          'calculate_bspline_fitting_error',
                          return_value=(0.0, 0.2, 1, None)):
            result = fit_service.fit_data('airfoil data', values, 10.0)

        self.assertEqual(fit.call_args.args[0], (6, 8))
        self.assertEqual(fit.call_args.args[3], (7, 9))
        self.assertEqual((result['degree_u'], result['degree_l']), (6, 8))

    def test_adapter_rejects_counts_outside_the_ui_contract(self):
        values = dict(te=0.0, smoothness=0.001, upper_count=3,
                      lower_count=10, continuity=2)
        with self.assertRaisesRegex(ValueError, '4 to 19'):
            fit_service.fit_data('unused', values, 10.0)

    def test_command_state_reset_clears_surface_counts(self):
        state.current_cp_count_upper = 12
        state.current_cp_count_lower = 14
        state.reset_state()
        self.assertIsNone(state.current_cp_count_upper)
        self.assertIsNone(state.current_cp_count_lower)


if __name__ == '__main__':
    unittest.main()
