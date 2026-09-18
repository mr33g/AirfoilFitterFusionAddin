"""Resolve sketch supports without requiring a surviving historical face."""

import math

import adsk.core


class AirfoilPlaneError(RuntimeError):
    """Fusion could not supply or construct the required sketch support."""


def _source_support(sketch):
    """Use the historical support when available, otherwise try the sketch itself."""
    try:
        support = sketch.referencePlane
        if support and support.isValid:
            return support
    except Exception:
        # A historical reference lookup can fail for an otherwise healthy sketch.
        pass

    return sketch


def _create_support_plane(sketch, name):
    """Retain a zero-offset support only when a consuming API rejects the sketch."""
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
    return plane


def with_sketch_support(source_sketch, support, name, operation, can_retry=lambda: True):
    """Return (result, effective support); retry only rejected direct-sketch input.

    Import callers must veto a retry if Fusion has already created output.
    Never delete a helper after downstream features have started using it.
    """
    try:
        result = operation(support)
        if not result:
            raise AirfoilPlaneError("Fusion did not create the requested geometry.")
    except Exception:
        if support != source_sketch or not can_retry():
            raise
        support = _create_support_plane(source_sketch, name)
        result = operation(support)
        if not result:
            raise AirfoilPlaneError("Fusion did not create the requested geometry with a support plane.")
    return result, support


def resolve_airfoil_plane(selected_line, rotation_state, point_world, normal_world,
                          root_component, name):
    """Return a planar entity shared by the fixed and adjustable output paths."""
    source_sketch = selected_line.parentSketch
    rotation_state %= 4

    # A half-turn changes the airfoil orientation, not its supporting plane.
    if rotation_state in (0, 2):
        return _source_support(source_sketch)

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

    support = _source_support(source_sketch)
    planes = source_sketch.parentComponent.constructionPlanes
    def create_angled(candidate):
        plane_input = planes.createInput()
        if selected_line.assemblyContext:
            plane_input.occurrenceForCreation = selected_line.assemblyContext
        theta = rotation_state * math.pi / 2.0
        if not plane_input.setByAngle(
                selected_line, adsk.core.ValueInput.createByReal(-theta), candidate):
            raise AirfoilPlaneError("Fusion rejected the angled airfoil plane.")
        return planes.add(plane_input)

    try:
        plane, _ = with_sketch_support(
            source_sketch, support, name, create_angled)
    except Exception as exc:
        raise AirfoilPlaneError(f"Cannot construct the rotated airfoil plane: {exc}") from exc
    plane.name = name
    plane.isLightBulbOn = False
    return plane


def add_airfoil_sketch(source_sketch, support, name, return_support=False):
    """Create an empty output sketch in the source component and context."""
    occurrence = source_sketch.assemblyContext
    # occurrenceForCreation is a method argument, not a Sketches property.
    # Avoid projecting the face boundary into the airfoil's new sketch.
    sketch, support = with_sketch_support(
        source_sketch, support, name,
        lambda candidate: source_sketch.parentComponent.sketches.addWithoutEdges(candidate, occurrence))
    if not sketch:
        raise AirfoilPlaneError("Fusion did not create the airfoil sketch.")
    sketch.name = name
    # The collection returns a native sketch. Coordinate conversion from world
    # space must see the same occurrence as the selected chord line.
    if occurrence and not sketch.assemblyContext:
        sketch = sketch.createForAssemblyContext(occurrence)
        if not sketch:
            raise AirfoilPlaneError("Fusion did not provide the airfoil sketch occurrence.")
    return (sketch, support) if return_support else sketch
