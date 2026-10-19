"""Numerical migration regressions adapted to the current shared core."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '_vendor'))
from logic.fit_service import fit_data, AirfoilFitError, _load_source

def fit_loaded_airfoil(processor, te, smoothness, upper, lower, continuity):
    return fit_data(source_text(), dict(te=te, smoothness=smoothness,
        upper_count=upper, lower_count=lower, continuity=continuity), 1.0)


def source_text():
    x = (1 - np.cos(np.linspace(0, np.pi, 61))) / 2
    y = .6 * (.2969*np.sqrt(x)-.126*x-.3516*x*x+.2843*x**3-.1036*x**4)
    points = list(zip(x[::-1], y[::-1])) + list(zip(x[1:], -y[1:]))
    text = 'foil\n' + '\n'.join(f'{a:.12f} {b:.12f}' for a, b in points)
    return text

def source():
    return _load_source(source_text())


class FittingTests(unittest.TestCase):
    def test_all_continuity_levels_and_unequal_single_spans(self):
        processor = source()
        for continuity in (1, 2, 3):
            with self.subTest(continuity=continuity):
                data = fit_loaded_airfoil(processor, .02, .001, 8, 9, continuity)
                self.assertEqual(data['fit_result'].achieved_continuity, continuity)
                for side, short, count in [('upper', 'u', 8), ('lower', 'l', 9)]:
                    self.assertEqual(data[f'degree_{short}'], count - 1)
                    self.assertEqual(len(data[f'{side}_cp_raw']), count)
                    np.testing.assert_array_equal(data[f'{side}_knots'], [0]*count+[1]*count)
                    self.assertTrue(np.isfinite(data[f'err_{short}']))

    def test_thickness_is_preserved_without_mutating_loaded_data(self):
        processor = source()
        original = processor.upper_data.copy()
        for te in (.02, 0):
            data = fit_loaded_airfoil(processor, te, .001, 8, 8, 2)
            self.assertAlmostEqual(data['upper_cp_raw'][-1, 1]-data['lower_cp_raw'][-1, 1], te)
            self.assertEqual(data['is_sharp'], te == 0)
            self.assertAlmostEqual(data['error_upper'][-1, 1]-data['error_lower'][-1, 1], te)
        np.testing.assert_array_equal(processor.upper_data, original)

    def test_failed_fit_does_not_produce_preview_data(self):
        from logic import fit_service
        fit_service._fit_cached.cache_clear()
        class Failed:
            failure_message = 'test failure'
            def __bool__(self):
                return False
        with patch.object(fit_service, 'fit_with_recovery', return_value=Failed()):
            with self.assertRaisesRegex(AirfoilFitError, 'test failure'):
                fit_loaded_airfoil(source(), .02, .001, 8, 8, 3)

    def test_invalid_requests_are_rejected(self):
        processor = source()
        for te, smooth, upper, lower, mode in [(float('nan'), .001, 8, 8, 2),
                (-1, .001, 8, 8, 2), (0, -1, 8, 8, 2), (0, .001, 20, 8, 2),
                (0, .001, 8, 3, 2), (0, .001, 8, 8, 4)]:
            with self.subTest(values=(te, smooth, upper, lower, mode)):
                with self.assertRaises(ValueError):
                    fit_loaded_airfoil(processor, te, smooth, upper, lower, mode)


