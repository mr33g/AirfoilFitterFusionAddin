# Reference-plane regression checks

## Offline policy tests

Run `python -m unittest discover -s tests -v` from the repository root.
These tests use API doubles. They check fallback selection, dependency retention,
origin-plane offset checks, creation context and separate-sketch policy; they
cannot prove that Fusion accepts a support or computes the correct geometry.

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
   Fixed output is a regression control; adjustable alignment remains a separate
   known issue and is not claimed fixed by this change.

For each source, start with Keep Adjustable off, and test 0, 90, 180 and 270
degrees. Repeat with Keep Adjustable on. Include an offset or tilted sketch so
the 90-degree result cannot reuse a root origin plane. Try Flip as well.

Expected results:

- Selecting a chord is not rejected just because `referencePlane` fails.
- Preview and final fixed geometry agree in position, orientation and length.
- Every successful insertion leaves one new output sketch in the chord's
  component; final curves are not appended to the source sketch.
- A face boundary is not projected into the new fixed-output sketch.
- 0 and 180 degrees reuse the source support when available. They do not create
  an angled plane. If reference lookup fails, a hidden zero-offset helper based
  on the source sketch is expected and must remain in the timeline.
- 90 and 270 degrees reuse a root origin plane only when geometrically coincident;
  otherwise an angled plane is expected. Its fallback support, if any, remains.
- After Compute All, and after saving/reopening the test copy, generated planes
  and sketches have no new reference warnings. Inspect existing warnings before
  testing so they can be distinguished from new ones.
- Cancel the preview once: there should be no final output sketch or new support
  plane from that cancelled command.

If a case fails, capture the source sketch, rotation, Flip and Keep Adjustable
settings, the failure stage, and the Text Commands log (Ctrl+Alt+C). The fallback
logs the original reference exception and whether a retained support was created.
The add-in does not move the timeline or redefine the source sketch to repair it.
