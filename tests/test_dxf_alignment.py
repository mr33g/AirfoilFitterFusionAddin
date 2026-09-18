"""Exercise the DXF import/alignment path with numerical API doubles.

These verify coordinate bookkeeping, not Fusion's importer or spline solver.
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np


class Point:
    def __init__(self, x, y, z=0):
        self.x, self.y, self.z = x, y, z

    create = staticmethod(lambda x, y, z: Point(x, y, z))

    def array(self):
        return np.array([self.x, self.y, self.z, 1.0])

    def distanceTo(self, other):
        return np.linalg.norm(self.array()[:3] - other.array()[:3])

    @property
    def length(self):
        return np.linalg.norm(self.array()[:3])


class Matrix:
    def __init__(self):
        self.data = np.eye(4)

    create = staticmethod(lambda: Matrix())

    def setCell(self, row, col, value):
        self.data[row, col] = value


class Collection(list):
    create = staticmethod(lambda: Collection())
    count = property(lambda self: len(self))
    item = list.__getitem__
    add = list.append


def placement(angle, translation, tilt=0):
    c, s = math.cos(angle), math.sin(angle)
    ct, st = math.cos(tilt), math.sin(tilt)
    result = np.eye(4)
    result[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.array(
        [[1, 0, 0], [0, ct, -st], [0, st, ct]])
    result[:3, 3] = translation
    return result


def mapped(matrix, point):
    return Point(*(matrix @ point.array())[:3])


class ImportedSketch:
    def __init__(self, sketch_to_component, blunt):
        self.sketch_to_component = sketch_to_component
        self.proxy_calls = []
        self.moves = 0
        splines = Collection()
        for sign in (1, -1):
            points = [NS(geometry=Point(0, 0)), NS(geometry=Point(4, sign)),
                      NS(geometry=Point(10, sign * .2 if blunt else 0))]
            splines.append(NS(controlPoints=points, startSketchPoint=points[0],
                              endSketchPoint=points[-1], controlFrameLines=[]))
        lines = Collection()
        if blunt:
            endpoints = [NS(geometry=Point(10, .2)), NS(geometry=Point(10, -.2))]
            lines.append(NS(controlPoints=endpoints, startSketchPoint=endpoints[0],
                            endSketchPoint=endpoints[1]))
        self.sketchCurves = NS(sketchControlPointSplines=splines, sketchLines=lines,
                              sketchFittedSplines=Collection(), sketchFixedSplines=Collection())

    def modelToSketchSpace(self, point):
        return mapped(np.linalg.inv(self.sketch_to_component), point)

    def createForAssemblyContext(self, occurrence):
        self.proxy_calls.append(occurrence)
        inverse = np.linalg.inv(occurrence.matrix @ self.sketch_to_component)
        # Deliberately expose conversion only: movement must remain native.
        return NS(modelToSketchSpace=lambda point: mapped(inverse, point))

    def move(self, entities, transform):
        self.moves += 1
        moved_points = set()
        for entity in entities:
            for point in entity.controlPoints:
                if id(point) in moved_points:
                    continue
                moved_points.add(id(point))
                point.geometry = mapped(transform.data, point.geometry)
        return True


class DxfAlignmentTests(unittest.TestCase):
    def setUp(self):
        adsk = ModuleType('adsk')
        adsk.core = ModuleType('adsk.core')
        adsk.fusion = ModuleType('adsk.fusion')
        self.app = NS(log=Mock())
        adsk.core.Application = NS(get=lambda: self.app)
        adsk.core.Point3D = adsk.core.Vector3D = Point
        adsk.core.Matrix3D = Matrix
        adsk.core.ObjectCollection = Collection
        self.exporter = NS(export_transformed_bspline_to_dxf=Mock(return_value=NS(saveas=Mock())))
        utils = ModuleType('utils')
        utils.__path__ = [str(Path(__file__).resolve().parents[1] / 'utils')]
        utils.dxf_exporter = self.exporter
        spec = importlib.util.spec_from_file_location('dxf_helper_under_test',
            Path(__file__).resolve().parents[1] / 'utils' / 'fusion_geometry_helper.py')
        self.helper = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'adsk': adsk, 'adsk.core': adsk.core,
                                     'adsk.fusion': adsk.fusion, 'utils': utils}):
            spec.loader.exec_module(self.helper)

    def fixture(self):
        sketch = ImportedSketch(np.eye(4), False)
        expected = [[Point(p.geometry.x, p.geometry.y, p.geometry.z)
                     for p in s.controlPoints]
                    for s in sketch.sketchCurves.sketchControlPointSplines]
        return sketch, expected

    def align(self, sketch, expected):
        self.helper._align_airfoil_with_chord(
            sketch, Point(0, 0), Point(10, 0), expected_control_points=expected)

    def test_rejected_move_raises(self):
        sketch, expected = self.fixture()
        sketch.move = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, 'could not move'):
            self.align(sketch, expected)

    def test_move_exception_propagates(self):
        sketch, expected = self.fixture()
        sketch.move = Mock(side_effect=RuntimeError('solver failure'))
        with self.assertRaisesRegex(RuntimeError, 'solver failure'):
            self.align(sketch, expected)

    def test_successful_return_without_movement_is_rejected(self):
        sketch, expected = self.fixture()
        sketch.move = Mock(return_value=True)
        with self.assertRaisesRegex(RuntimeError, 'endpoints failed'):
            self.helper._align_airfoil_with_chord(sketch, Point(3, 4), Point(13, 4))

    def test_wrong_length_is_rejected_without_rescaling(self):
        sketch, expected = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'length does not match'):
            self.helper._align_airfoil_with_chord(sketch, Point(0, 0), Point(20, 0))
        self.assertEqual(sketch.moves, 0)

    def test_wrong_plane_is_rejected(self):
        sketch, expected = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'sketch plane'):
            self.helper._align_airfoil_with_chord(sketch, Point(0, 0, 2), Point(10, 0, 2))

    def test_shape_damage_with_correct_endpoints_is_rejected(self):
        sketch, expected = self.fixture()
        sketch.sketchCurves.sketchControlPointSplines[0].controlPoints[1].geometry.y = 2
        with self.assertRaisesRegex(RuntimeError, 'shape or orientation'):
            self.align(sketch, expected)

    def test_mirrored_asymmetric_airfoil_is_rejected(self):
        sketch, expected = self.fixture()
        expected[0][1].y = 2
        sketch.sketchCurves.sketchControlPointSplines[0].controlPoints[1].geometry.y = -2
        sketch.sketchCurves.sketchControlPointSplines[1].controlPoints[1].geometry.y = 1
        with self.assertRaisesRegex(RuntimeError, 'shape or orientation'):
            self.align(sketch, expected)

    def test_surface_enumeration_order_does_not_matter(self):
        sketch, expected = self.fixture()
        sketch.sketchCurves.sketchControlPointSplines.reverse()
        self.align(sketch, expected)

    def test_disconnected_trailing_edge_is_rejected(self):
        sketch = ImportedSketch(np.eye(4), True)
        sketch.sketchCurves.sketchLines[0].startSketchPoint.geometry.y = 3
        with self.assertRaisesRegex(RuntimeError, 'connector failed'):
            self.align(sketch, None)

    def test_control_frame_lines_are_not_trailing_edge_connectors(self):
        for blunt in (False, True):
            with self.subTest(blunt=blunt):
                sketch = ImportedSketch(np.eye(4), blunt)
                for spline in sketch.sketchCurves.sketchControlPointSplines:
                    for a, b in zip(spline.controlPoints, spline.controlPoints[1:]):
                        line = NS(controlPoints=[a, b], startSketchPoint=a, endSketchPoint=b)
                        spline.controlFrameLines.append(line)
                        sketch.sketchCurves.sketchLines.append(line)
                self.helper._align_airfoil_with_chord(sketch, Point(3, 4), Point(3, 14))
                self.assertEqual(sketch.moves, 1)
                # A genuine connector error must still fail with control frames present.
                if blunt:
                    sketch.sketchCurves.sketchLines[0].startSketchPoint.geometry.x += 1
                    with self.assertRaisesRegex(RuntimeError, 'connector failed'):
                        self.helper._align_airfoil_with_chord(sketch, Point(3, 4), Point(3, 14))

    def test_bad_import_results_raise(self):
        sketch, expected = self.fixture()
        temporary = NS(parentComponent=NS(sketches=Collection()), assemblyContext=None)
        for success, results in [(False, Collection()), (True, Collection()),
                                 (True, Collection([sketch, sketch]))]:
            with self.subTest(success=success, count=len(results)):
                self.app.importManager = NS(
                    createDXF2DImportOptions=Mock(return_value=NS(
                        isCreateControlPointSplines=False, results=results)),
                    importToTarget=Mock(return_value=success))
                with patch.object(self.helper.os.path, 'exists', return_value=False):
                    with self.assertRaisesRegex(RuntimeError, 'DXF import'):
                        self.helper.import_splines_via_dxf(
                            temporary, object(), [], [], 8, [], [], 8, True,
                            Point(0, 0), Point(10, 0))

    def test_import_alignment_across_occurrences_and_sketch_orientations(self):
        contexts = [None, NS(matrix=placement(0, (17, -9, 6))),
                    NS(matrix=placement(.8, (17, -9, 6), .4))]
        for context in contexts:
            for rotation in range(4):
                for flip in (False, True):
                    for blunt in (False, True):
                        with self.subTest(context=context, rotation=rotation, flip=flip, blunt=blunt):
                            sketch_frame = placement(.3, (2, 3, 4), rotation * math.pi / 2)
                            imported = ImportedSketch(sketch_frame, blunt)
                            sketches = Collection()
                            component = NS(sketches=sketches)
                            temporary = NS(parentComponent=component, assemblyContext=context)
                            options = NS(isCreateControlPointSplines=False, results=Collection([imported]))
                            self.app.importManager = NS(
                                createDXF2DImportOptions=Mock(return_value=options),
                                importToTarget=Mock(return_value=True))
                            world_frame = sketch_frame if context is None else context.matrix @ sketch_frame
                            start, end = Point(3, -2), Point(9, 6)  # length 10
                            if flip:
                                start, end = end, start
                            start_world, end_world = mapped(world_frame, start), mapped(world_frame, end)
                            before = start_world.array().copy(), end_world.array().copy()
                            # Expected export data is already placed in the temporary sketch.
                            angle = math.atan2(end.y-start.y, end.x-start.x)
                            desired = placement(angle, (start.x, start.y, 0))
                            surfaces = [[mapped(desired, p.geometry).array()[:3].tolist()
                                         for p in spline.controlPoints]
                                        for spline in imported.sketchCurves.sketchControlPointSplines]
                            temporary.sketchToModelSpace = lambda point: mapped(world_frame, point)
                            knots = [0, 1]
                            with patch.object(self.helper.os.path, 'exists', return_value=False):
                                result = self.helper.import_splines_via_dxf(
                                    temporary, object(), surfaces[0], knots, 8, surfaces[1], knots, 7,
                                    not blunt, start_world, end_world)
                            self.assertIs(result, imported)
                            self.assertTrue(options.isCreateControlPointSplines)
                            self.assertEqual(imported.proxy_calls, [] if context is None else [context])
                            self.assertEqual(imported.moves, 1)
                            splines = imported.sketchCurves.sketchControlPointSplines
                            for attribute, expected in [('startSketchPoint', start_world),
                                                        ('endSketchPoint', end_world)]:
                                midpoint = Point(*np.mean([getattr(s, attribute).geometry.array()[:3]
                                                          for s in splines], axis=0))
                                self.assertLess(mapped(world_frame, midpoint).distanceTo(expected), 1e-10)
                            if blunt:
                                connector = imported.sketchCurves.sketchLines[0].controlPoints
                                for point, spline in zip(connector, splines):
                                    self.assertLess(point.geometry.distanceTo(spline.endSketchPoint.geometry), 1e-10)
                            np.testing.assert_array_equal(start_world.array(), before[0])
                            np.testing.assert_array_equal(end_world.array(), before[1])
                            args = self.exporter.export_transformed_bspline_to_dxf.call_args.args
                            self.assertEqual((args[2], args[5]), (8, 7))
                            self.app.log.assert_not_called()


if __name__ == '__main__':
    unittest.main()
