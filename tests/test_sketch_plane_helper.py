"""Offline regression tests for support policy, not Fusion's geometry kernel.

Run with: python -m unittest discover -s tests -v
"""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def load_helper():
    adsk = ModuleType('adsk')
    core = ModuleType('adsk.core')
    core.Application = SimpleNamespace(get=lambda: SimpleNamespace(log=Mock()))
    core.ValueInput = SimpleNamespace(createByReal=lambda value: value)
    adsk.core = core
    spec = importlib.util.spec_from_file_location(
        'plane_helper_under_test',
        Path(__file__).resolve().parents[1] / 'utils' / 'sketch_plane_helper.py',
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'adsk': adsk, 'adsk.core': core}):
        spec.loader.exec_module(module)
    return module


helper = load_helper()


class SourceSketch:
    def __init__(self, component, reference=None, error=None, occurrence=None):
        self.parentComponent = component
        self.assemblyContext = occurrence
        self.name = 'Face chord'
        self.reference = reference
        self.error = error
        self.reference_reads = 0

    @property
    def referencePlane(self):
        self.reference_reads += 1
        if self.error:
            raise self.error
        return self.reference


class PlaneTests(unittest.TestCase):
    def setUp(self):
        self.inputs = []
        self.planes = []
        self.fail_offset = False
        self.fail_angle = False
        self.reject_sketch_angle = False

        def create_input():
            result = SimpleNamespace(
                occurrenceForCreation=None,
                setByOffset=Mock(side_effect=lambda *args: not self.fail_offset),
                setByAngle=Mock(side_effect=lambda *args: not self.fail_angle and not (
                    self.reject_sketch_angle and isinstance(args[2], SourceSketch))),
            )
            self.inputs.append(result)
            return result

        def add_plane(plane_input):
            plane = SimpleNamespace(
                isValid=True, name='', isLightBulbOn=True,
                deleteMe=Mock(), definition_input=plane_input,
            )
            self.planes.append(plane)
            return plane

        self.component = SimpleNamespace(
            constructionPlanes=SimpleNamespace(
                createInput=Mock(side_effect=create_input), add=Mock(side_effect=add_plane)),
            sketches=SimpleNamespace(addWithoutEdges=Mock()),
        )
        self.root = SimpleNamespace(
            xYConstructionPlane=object(), xZConstructionPlane=object(),
            yZConstructionPlane=object(),
        )
        self.support = SimpleNamespace(isValid=True)
        self.sketch = SourceSketch(self.component, reference=self.support)
        self.line = SimpleNamespace(parentSketch=self.sketch, assemblyContext=None)
        self.point = SimpleNamespace(x=3.0, y=4.0, z=5.0)
        self.normal = SimpleNamespace(x=0.0, y=0.0, z=1.0)

    def resolve(self, rotation):
        return helper.resolve_airfoil_plane(
            self.line, rotation, self.point, self.normal, self.root, 'NACA')

    def test_valid_face_or_construction_support_is_reused_for_zero_and_half_turn(self):
        for rotation in (0, 2):
            with self.subTest(rotation=rotation):
                self.assertIs(self.resolve(rotation), self.support)
        self.assertEqual(self.planes, [])

    def test_lookup_exceptions_do_not_reject_usable_sketch(self):
        for message in ('2 : InternalValidationError : dcSketch',
                        '2 : InternalValidationError : rMatrix->getTargetObjectPath(objectPath)'):
            with self.subTest(message=message):
                self.sketch.error = RuntimeError(message)
                plane = self.resolve(0)
                self.assertIs(plane, self.sketch)
                self.assertEqual(self.planes, [])

    def test_null_and_invalid_reference_use_sketch_support(self):
        for reference in (None, SimpleNamespace(isValid=False)):
            with self.subTest(reference=reference):
                self.sketch.reference = reference
                plane = self.resolve(2)
                self.assertIs(plane, self.sketch)
                self.assertEqual(self.planes, [])

    def test_half_turn_does_not_substitute_origin_plane_for_source_support(self):
        self.point.z = 0
        self.assertIs(self.resolve(2), self.support)
        self.assertEqual(self.planes, [])

    def test_quarter_turn_reuses_coincident_root_plane_without_reference_lookup(self):
        self.sketch.error = RuntimeError('historical face unavailable')
        self.point.z = 0
        for rotation in (1, 3):
            self.assertIs(self.resolve(rotation), self.root.xYConstructionPlane)
        self.assertEqual(self.sketch.reference_reads, 0)
        self.assertEqual(self.planes, [])

    def test_parallel_but_offset_plane_is_not_flattened_onto_origin(self):
        for rotation in (1, 3):
            with self.subTest(rotation=rotation):
                plane = self.resolve(rotation)
                plane.definition_input.setByAngle.assert_called_once_with(
                    self.line, -rotation * math.pi / 2, self.support)
                plane.definition_input.setByOffset.assert_not_called()

    def test_oblique_plane_at_origin_is_not_replaced_by_origin_plane(self):
        self.point = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.normal = SimpleNamespace(x=0.0, y=math.sqrt(0.5), z=math.sqrt(0.5))
        plane = self.resolve(1)
        plane.definition_input.setByAngle.assert_called_once()

    def test_angled_plane_keeps_its_fallback_support_alive(self):
        self.sketch.error = RuntimeError('face no longer exists')
        self.reject_sketch_angle = True
        plane = self.resolve(1)
        self.assertEqual(len(self.planes), 2)
        fallback, rotated = self.planes
        self.assertIs(plane, rotated)
        rotated.definition_input.setByAngle.assert_called_once_with(
            self.line, -math.pi / 2, fallback)
        fallback.deleteMe.assert_not_called()
        rotated.deleteMe.assert_not_called()

    def test_direct_angle_uses_only_the_final_plane(self):
        self.sketch.reference = None
        plane = self.resolve(1)
        self.assertEqual(len(self.planes), 1)
        plane.definition_input.setByAngle.assert_called_once_with(
            self.line, -math.pi / 2, self.sketch)
        plane.definition_input.setByOffset.assert_not_called()

    def test_direct_new_sketch_needs_no_helper(self):
        self.sketch.reference = None
        output = SimpleNamespace(name='', assemblyContext=None)
        self.component.sketches.addWithoutEdges.return_value = output
        result, support = helper.add_airfoil_sketch(
            self.sketch, self.resolve(0), 'NACA', return_support=True)
        self.assertIs(result, output)
        self.assertIs(support, self.sketch)
        self.assertEqual(self.planes, [])

    def test_rejected_new_sketch_returns_effective_support(self):
        output = SimpleNamespace(name='', assemblyContext=None)
        for rejection in (None, RuntimeError('unsupported sketch')):
            with self.subTest(rejection=rejection):
                self.component.sketches.addWithoutEdges.side_effect = [rejection, output]
                result, support = helper.add_airfoil_sketch(
                    self.sketch, self.sketch, 'NACA', return_support=True)
                self.assertIs(result, output)
                self.assertIs(support, self.planes[-1])
                support.definition_input.setByOffset.assert_called_once_with(self.sketch, 0)

    def test_partial_import_vetoes_retry_and_helper_creation(self):
        operation = Mock(side_effect=RuntimeError('partial import'))
        with self.assertRaisesRegex(RuntimeError, 'partial import'):
            helper.with_sketch_support(self.sketch, self.sketch, 'NACA',
                                       operation, can_retry=lambda: False)
        operation.assert_called_once_with(self.sketch)
        self.assertEqual(self.planes, [])

    def test_rejected_direct_operation_retries_once_with_retained_plane(self):
        result = object()
        operation = Mock(side_effect=[RuntimeError('unsupported sketch'), result])
        actual, support = helper.with_sketch_support(
            self.sketch, self.sketch, 'NACA', operation)
        self.assertIs(actual, result)
        self.assertEqual(operation.call_count, 2)
        self.assertIs(operation.call_args.args[0], support)
        self.assertEqual(len(self.planes), 1)
        support.deleteMe.assert_not_called()

    def test_fallback_and_angled_creation_preserve_occurrence_context(self):
        occurrence = object()
        self.sketch.assemblyContext = occurrence
        self.line.assemblyContext = occurrence
        self.sketch.error = RuntimeError('face unavailable')
        self.reject_sketch_angle = True
        self.resolve(1)
        self.assertEqual(len(self.inputs), 3)
        for plane_input in self.inputs:
            self.assertIs(plane_input.occurrenceForCreation, occurrence)

    def test_failed_fallback_does_not_silently_choose_arbitrary_support(self):
        self.sketch.reference = None
        self.fail_offset = True
        with self.assertRaisesRegex(helper.AirfoilPlaneError, 'zero-offset'):
            helper._create_support_plane(self.sketch, 'NACA')
        self.assertEqual(self.planes, [])

    def test_failed_angle_is_reported_instead_of_inserting_in_source_sketch(self):
        self.fail_angle = True
        with self.assertRaisesRegex(helper.AirfoilPlaneError, 'angled'):
            self.resolve(1)
        self.assertEqual(self.planes, [])

    def test_null_plane_result_is_reported(self):
        self.component.constructionPlanes.add.side_effect = None
        self.component.constructionPlanes.add.return_value = None
        self.sketch.reference = None
        with self.assertRaisesRegex(helper.AirfoilPlaneError, 'did not create'):
            helper._create_support_plane(self.sketch, 'NACA')

    def test_final_sketch_is_new_and_has_no_projected_face_edges(self):
        new_sketch = SimpleNamespace(name='', assemblyContext=None)
        self.component.sketches.addWithoutEdges.return_value = new_sketch
        result = helper.add_airfoil_sketch(self.sketch, self.support, 'NACA')
        self.assertIs(result, new_sketch)
        self.assertIsNot(result, self.sketch)
        self.assertEqual(result.name, 'NACA')
        self.component.sketches.addWithoutEdges.assert_called_once_with(self.support, None)

    def test_new_component_sketch_receives_occurrence_argument_and_returns_proxy(self):
        occurrence = object()
        self.sketch.assemblyContext = occurrence
        proxy = SimpleNamespace(assemblyContext=occurrence)
        native = SimpleNamespace(name='', assemblyContext=None,
                                 createForAssemblyContext=Mock(return_value=proxy))
        self.component.sketches.addWithoutEdges.return_value = native
        result = helper.add_airfoil_sketch(self.sketch, self.support, 'NACA')
        self.assertIs(result, proxy)
        self.component.sketches.addWithoutEdges.assert_called_once_with(self.support, occurrence)
        native.createForAssemblyContext.assert_called_once_with(occurrence)

    def test_null_new_sketch_is_reported(self):
        self.component.sketches.addWithoutEdges.return_value = None
        with self.assertRaises(helper.AirfoilPlaneError):
            helper.add_airfoil_sketch(self.sketch, self.support, 'NACA')


if __name__ == '__main__':
    unittest.main()
