# Parametric AirfoilFitter regression checks

## Offline tests

From the repository root, run:

```text
python -m unittest discover -s tests -v
```

Numerical tests and API doubles cover fitting, embedded recipes, caching,
native/proxy coordinates, support fallbacks, curve replacement and rollback,
edit preview lifecycle, and highlighting. They do not run Fusion's geometry
kernel or prove that downstream operations update correctly.

## Fusion checks

Use this checkout rather than an installed build. Restart the add-in after
changes and use copies of the test documents.

### Creation and placement

Test an XY chord sketch, a sketch on an extrusion face, a sketch on a body later
converted to a component, and a sketch inside a moved/rotated internal component.
Include an offset or tilted sketch. Test all four initial rotations, Flip,
and sharp/blunt trailing edges.

- Preview and output match in position, orientation, scale, and closure.
- Each insertion creates one AF timeline item enclosing its new output sketch
  and generated supports. Source sketches receive no preview or final curves.
- Face boundaries are not projected into the output sketch.
- Missing historical referencePlane access alone does not reject a healthy sketch.
- Source-sketch supports and coincident origin planes are reused where possible.
- Multiple airfoils can use the same chord without freezing the preview.
- Cancel leaves no output geometry, support plane, or preview graphics.

### Parametric edits

Create a loft or sweep between airfoils, then add surface offsets and splits.

- Change TE thickness, source file, smoothing, continuity, and point counts.
- Check the initial edit preview and visibility of other airfoils.
- Open/accept without changes, double-Flip/accept, and Cancel: downstream
  geometry should not change due to those no-op edits.
- Chord selection and Rotate 90 degrees remain disabled during Edit.
- Verify Undo/Redo, Compute All, and save/reopen with the add-in running.
- Move/remove the original .dat file; embedded data should still permit edits.
- Change upstream chord length and twist and inspect both the AF sketch
  and downstream surfaces. Capture stale geometry before using Compute All.
- Selecting an AF timeline item highlights its curves; Edit Feature remains
  available. Its built-in context-menu icon may remain Fusion's generic icon.

Manual control-point editing (Keep Adjustable) is not supported in this branch.
The removed DXF workflow no longer has tests or a runtime dependency.

## Wersy wing: missed downstream update

Source: [forum post 175](https://www.rc-network.de/threads/airfoil-fitter-airfoil-fitter-add-in-f%C3%BCr-fusion.12099445/post-13336739),
attachment `Flügel mit AFitter.f3d.txt` (rename to `.f3d` when opening).
The uploaded file is already in the failing state; no parameter change is
needed to reproduce the reported symptom. Compute All repairs it according
to Micha's test. Do not save over the original after computing.

Before proposing a fix, compare the stored airfoil curves with the expected
curves from the current chord and recipe, and capture downstream feature health
and body geometry before and after Compute All. Record whether custom-feature
compute callbacks ran. This separates an outdated AF result from downstream
update propagation. The cause is not yet established; no unconditional
computeAll call has been added to normal add-in operation.

### Compute refresh experiment (2026-09-21)

The diagnostic run showed unchanged chord frames and sketch transforms, less
than 0.5 nanometer of spline control-point differences, but substantial changes
to the final wing body after Compute All. Changing twist again reproduced the
stale downstream result. This is not just an old saved-file state.

An experiment explicitly called replaceGeometry on existing output splines on
all compute callbacks, including unchanged sketch-local curves. Micha tested
this in Fusion: downstream geometry still needed Compute All. The experiment
and its dedicated tests were removed; it added work without fixing the issue.
This rules out skipping equal curves as a sufficient explanation, but does not
establish whether the remaining cause is in AF or Fusion's dependency handling.

Compute All remains the manual workaround. No automatic full-design recompute
has been added. Any further fix needs a live regression check: start with a
computed wing, change twist twice without Compute All, inspect downstream
surfaces/skin, and verify Undo/Redo and a no-op AF edit.
