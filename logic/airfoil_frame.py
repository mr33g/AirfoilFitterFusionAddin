"""The same chord frame for initial insertion and custom-feature recompute."""
import math
import adsk.core


def chord_frame(line, rotation, flip):
    start = line.startSketchPoint.worldGeometry
    end = line.endSketchPoint.worldGeometry
    x = start.vectorTo(end)
    length = x.length
    if length <= 1e-9:
        raise ValueError('The chord must have a positive length.')
    x.normalize()
    sketch = line.parentSketch
    matrix = sketch.transform
    if sketch.assemblyContext:
        matrix.transformBy(sketch.assemblyContext.transform2)
    normal = adsk.core.Vector3D.create(matrix.getCell(0, 2), matrix.getCell(1, 2), matrix.getCell(2, 2))
    normal.normalize()
    y_plane = normal.crossProduct(x)
    y_plane.normalize()
    if flip:
        start = end
        x.scaleBy(-1)
    theta = rotation * math.pi / 2
    y = y_plane.copy()
    y.scaleBy(math.cos(theta))
    part = normal.copy()
    part.scaleBy(math.sin(theta))
    y.add(part)
    z = normal.copy()
    z.scaleBy(math.cos(theta))
    part = y_plane.copy()
    part.scaleBy(-math.sin(theta))
    z.add(part)
    result = adsk.core.Matrix3D.create()
    result.setWithCoordinateSystem(start, x, y, z)
    return result, length


def sketch_points(points, frame, length, sketch):
    result = []
    for point in points:
        world = adsk.core.Point3D.create(float(point[0]) * length, float(point[1]) * length, 0)
        world.transformBy(frame)
        result.append(sketch.modelToSketchSpace(world))
    return result
