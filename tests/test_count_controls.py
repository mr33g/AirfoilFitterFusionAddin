"""Preserved local migration regressions for Fusion count controls."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
from logic import state
ROOT = Path(__file__).resolve().parents[1]

class CountControlTests(unittest.TestCase):
    def setUp(self):
        state.reset_state()
        self.addCleanup(state.reset_state)
        state.current_cp_count_upper, state.current_cp_count_lower = 8, 10
        state.fit_cache = {'previous': True}
        state.needs_refit = False
        adsk = ModuleType('adsk')
        adsk.core, adsk.fusion = ModuleType('adsk.core'), ModuleType('adsk.fusion')
        for name in ('CommandCreatedEventHandler', 'CommandEventHandler', 'InputChangedEventHandler'):
            setattr(adsk.core, name, type(name, (), {}))
        adsk.core.Application = NS(get=lambda: NS(userInterface=Mock()))
        adsk.core.InputChangedEventArgs = NS(cast=lambda obj: obj)
        self.controls = {'initial_cp_count': NS(value=6), 'chord_line': NS(selectionCount=1),
                         'file_path': NS(value='foil.dat')}
        self.inputs = NS(itemById=lambda key: self.controls.get(key))
        spec = importlib.util.spec_from_file_location('legacy_handlers_test', ROOT/'ui/handlers.py')
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'adsk': adsk, 'adsk.core': adsk.core, 'adsk.fusion': adsk.fusion,
                'ui.dialog': NS(create_ui_inputs=Mock()), 'logic.fitter': NS(run_fitter=Mock()),
                'utils.i18n': NS(t=lambda key, **kwargs: key)}):
            spec.loader.exec_module(self.module)
        self.handler = self.module.AirfoilFitterCommandInputChangedHandler()
        owner = self
        class Button:
            def __init__(self, name, count):
                self.name, self._text, self.writes = name, f'  {count}', 0
            @property
            def text(self):
                return self._text
            @text.setter
            def text(self, value):
                self._text = value
                self.writes += 1
                owner.change(self.name)
        self.controls['cp_count_upper'] = Button('cp_count_upper', 8)
        self.controls['cp_count_lower'] = Button('cp_count_lower', 10)

    def change(self, name):
        self.handler.notify(NS(inputs=self.inputs, input=NS(id=name)))

    def test_initial_count_applies_only_on_reset(self):
        previous = state.fit_cache
        self.change('initial_cp_count')
        self.assertEqual((state.current_cp_count_upper, state.current_cp_count_lower), (8, 10))
        self.assertIs(state.fit_cache, previous)
        self.assertFalse(state.needs_refit)
        self.change('reset_button')
        self.assertEqual((state.current_cp_count_upper, state.current_cp_count_lower), (6, 6))
        self.assertEqual(state.fit_cache, {})
        self.assertTrue(state.needs_refit)

    def test_reentrant_label_events_do_not_add_points_to_both_surfaces(self):
        self.change('cp_count_upper')
        self.assertEqual((state.current_cp_count_upper, state.current_cp_count_lower), (9, 10))
        self.change('cp_count_lower')
        self.assertEqual((state.current_cp_count_upper, state.current_cp_count_lower), (9, 11))

    def test_counts_stop_at_supported_limit(self):
        state.current_cp_count_upper = 19
        self.change('cp_count_upper')
        self.assertEqual((state.current_cp_count_upper, state.current_cp_count_lower), (19, 10))

    def test_continuity_change_preserves_thickness_and_fit_cache_until_refit(self):
        previous = state.fit_cache
        self.change('continuity_level')
        self.assertIs(state.fit_cache, previous)
        self.assertTrue(state.needs_refit)

    def test_command_reset_clears_both_surface_counts(self):
        state.reset_state()
        self.assertIsNone(state.current_cp_count_upper)
        self.assertIsNone(state.current_cp_count_lower)
