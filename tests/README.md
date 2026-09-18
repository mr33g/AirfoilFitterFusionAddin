# Reference-plane regression checks

## Offline policy tests

Run `python -m unittest discover -s tests -v` from the repository root.
These tests use API doubles. They check fallback selection, dependency retention,
origin-plane offset checks, creation context and separate-sketch policy; they
cannot prove that Fusion accepts a support or computes the correct geometry.

The DXF alignment test uses numerical API doubles to exercise 48 combinations
of root/translated/rotated occurrences, sketch orientation, Flip and sharp/blunt
trailing edges. It checks world-space endpoint alignment, native sketch movement,
connector movement and unchanged degree arguments above 5. It does not run
Fusion's DXF importer or prove the resulting splines remain editable.
Failure tests cover rejected/ineffective moves, incorrect length or plane,
mirrored or damaged geometry, disconnected trailing-edge connectors, and
missing/ambiguous import results. Placement now verifies every control point
against the intended geometry (allowing either surface enumeration order).
The temporary sketch and single rigid movement remain; no automatic 180-degree
retry or rescaling is performed. Validation failures propagate to the existing
command error handler and Text Commands log.

## Manual checks in Fusion

Load the changed checkout, not an older installed copy. Restart Fusion after
updating the add-in so its imported Python modules are refreshed. Use a copy of
the test document and keep the original for comparison.

Check these source sketches:

1. The original XY rectangle/chord (baseline).
2. A sketch on a simple extrusion face, in a document without component operations.
3. A face-supported sketch in the existing test document, including the body
   converted into a component and moved later, and the second extrusion.
4. A sketch made inside an internal component created first, then moved/rotated.
   Fixed output is a regression control. Adjustable output now uses the selected
   occurrence for endpoint conversion; this correction needs validation in Fusion.

For each source, start with Keep Adjustable off, and test 0, 90, 180 and 270
degrees. Repeat with Keep Adjustable on. Include an offset or tilted sketch so
the 90-degree result cannot reuse a root origin plane. Try Flip as well.

Expected results:

- Selecting a chord is not rejected just because `referencePlane` fails.
- Preview and final fixed geometry agree in position, orientation and length.
- Adjustable output also follows the chord in moved/rotated components. Check
  both endpoints, airfoil side/orientation and blunt trailing-edge closure. Use
  degree 6 or higher and verify that control points remain editable after import.
- Every successful insertion leaves one new output sketch in the chord's
  component; final curves are not appended to the source sketch.
- A face boundary is not projected into the new fixed-output sketch.
- 0 and 180 degrees reuse the source support when available. They do not create
  an angled plane. If reference lookup fails, the source sketch is tried directly.
  No support helper should appear when Fusion accepts it. A hidden zero-offset
  helper is retained only if the consuming API rejects direct sketch support.
- 90 and 270 degrees reuse a root origin plane only when geometrically coincident;
  otherwise an angled plane is expected. Its fallback support, if any, remains.
- After Compute All, and after saving/reopening the test copy, generated planes
  and sketches have no new reference warnings. Inspect existing warnings before
  testing so they can be distinguished from new ones.
- Cancel the preview once: there should be no final output sketch or new support
  plane from that cancelled command.

If a case fails, capture the source sketch, rotation, Flip and Keep Adjustable
settings, the failure stage, and the Text Commands error log (Ctrl+Alt+C).
Successful support selection and recovered reference lookups are silent.
The add-in does not move the timeline or redefine the source sketch to repair it.

For the direct-sketch experiment, check both fixed and adjustable output at all
four rotations. Inspect the timeline for any fallback support plane.
Verify Compute All and save/reopen as well as initial placement;
a reference lookup exception alone does not imply an unhealthy source sketch.
