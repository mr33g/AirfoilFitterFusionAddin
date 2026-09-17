"""Resolve sketch supports without requiring a surviving historical face."""

import math

import adsk.core


class AirfoilPlaneError(RuntimeError):
    """Fusion could not supply or construct the required sketch support."""


def _source_support(sketch, name, log):
    """Prefer the original support; only create a helper when lookup fails.

    A sketch can still define a usable plane after its original face has ceased
    to exist at the current timeline position. Fusion accepts a sketch as the
    reference for a zero-offset construction plane. Keep that helper: subsequent
    sketches/angled planes depend on it. Do not redefine the user's sketch or
    roll the user's timeline back to recover a historical BRepFace.
    """
    try:
        support = sketch.referencePlane
        if support and support.isValid:
            return support
        log("AirfoilFitter: source sketch referencePlane is unavailable.")
    except Exception as exc:
        log(f"AirfoilFitter: cannot read source sketch referencePlane: {exc}")

    planes = sketch.parentComponent.constructionPlanes
    plane_input = planes.createInput()
    if sketch.assemblyContext:
        plane_input.occurrenceForCreation = sketch.assemblyContext
    try:
        if not plane_input.setByOffset(sketch, adsk.core.ValueInput.createByReal(0)):
            raise AirfoilPlaneError("Fusion rejected the zero-offset sketch support.")
        plane = planes.add(plane_input)
        if not plane:
            raise AirfoilPlaneError("Fusion did not create the sketch support.")
    except Exception as exc:
        raise AirfoilPlaneError(
            f"Cannot construct a support from sketch '{sketch.name}': {exc}"
        ) from exc
    plane.name = f"{name} - sketch support"
    plane.isLightBulbOn = False
    log(f"AirfoilFitter: retained zero-offset support for sketch '{sketch.name}'.")
    return plane


def resolve_airfoil_plane(selected_line, rotation_state, point_world, normal_world,
                          root_component, name, log):
    """Return a planar entity shared by the fixed and adjustable output paths."""
    source_sketch = selected_line.parentSketch
    rotation_state %= 4

    # A half-turn changes the airfoil orientation, not its supporting plane.
    if rotation_state in (0, 2):
        return _source_support(source_sketch, name, log)

    # Reuse an origin plane only if both the normal and the offset match.
    # Inputs here are world coordinates, so these must be ROOT origin planes.
    tol = 1e-6
    for coordinate, plane in (
        ('z', root_component.xYConstructionPlane),
        ('y', root_component.xZConstructionPlane),
        ('x', root_component.yZConstructionPlane),
    ):
        if (abs(abs(getattr(normal_world, coordinate)) - 1.0) < tol
                and abs(getattr(point_world, coordinate)) < tol):
            return plane

    support = _source_support(source_sketch, name, log)
    planes = source_sketch.parentComponent.constructionPlanes
    plane_input = planes.createInput()
    if selected_line.assemblyContext:
        plane_input.occurrenceForCreation = selected_line.assemblyContext
    try:
        theta = rotation_state * math.pi / 2.0
        if not plane_input.setByAngle(
                selected_line, adsk.core.ValueInput.createByReal(-theta), support):
            raise AirfoilPlaneError("Fusion rejected the angled airfoil plane.")
        plane = planes.add(plane_input)
        if not plane:
            raise AirfoilPlaneError("Fusion did not create the angled airfoil plane.")
    except Exception as exc:
        raise AirfoilPlaneError(f"Cannot construct the rotated airfoil plane: {exc}") from exc
    plane.name = name
    plane.isLightBulbOn = False
    return plane


def add_airfoil_sketch(source_sketch, support, name):
    """Create an empty output sketch in the source component and context."""
    occurrence = source_sketch.assemblyContext
    # occurrenceForCreation is a method argument, not a Sketches property.
    # Avoid projecting the face boundary into the airfoil's new sketch.
    sketch = source_sketch.parentComponent.sketches.addWithoutEdges(support, occurrence)
    if not sketch:
        raise AirfoilPlaneError("Fusion did not create the airfoil sketch.")
    sketch.name = name
    # The collection returns a native sketch. Coordinate conversion from world
    # space must see the same occurrence as the selected chord line.
    if occurrence and not sketch.assemblyContext:
        sketch = sketch.createForAssemblyContext(occurrence)
        if not sketch:
            raise AirfoilPlaneError("Fusion did not provide the airfoil sketch occurrence.")
    return sketch
