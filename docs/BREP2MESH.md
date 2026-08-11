# B-rep → mesh

`solid-diff mesh` tessellates the Parasolid B-rep extracted from a SLDPRT
(see PARASOLID.md) into a triangle mesh.

```sh
solid-diff mesh part.SLDPRT -o part.obj --stl part.stl [--tol 1e-4] --stats
```

## How it works

The B-rep is exported to STEP (`rust/src/step.rs`) and meshed by OpenCASCADE
(`rust/src/occt.rs`). A native per-face tessellator used to live in
`tess.rs`; it was removed when the project went all-in on the OCCT path.

1. **STEP export** — surfaces are written exactly wherever STEP has the type
   (plane, cylinder, cone, sphere, torus, B-spline); everything else is
   sampled. Edges are **sampled polylines** from the shared two-pass
   `EdgeSampler` (`rust/src/sample.rs`): each edge is sampled once, in 3D, at
   the finest spacing either adjacent face wants, so both faces reference one
   curve entity and the shell sews. This sidesteps the two-arcs ambiguity on
   periodic curves, null-curve edges (reconstructed as surface×surface
   intersections when both supports are exact), `INTERSECTION` and
   `SP_CURVE`. Faces on closed surfaces get their seam emitted explicitly —
   OCCT's reader-side `FixMissingSeam` does not repair a dropped seam.
2. **OCCT import + meshing** — the STEP reader runs shape healing on the way
   in (pcurve computation, seam insertion, wire orientation), then
   `BRepMesh_IncrementalMesh` triangulates at a deflection of 0.05% of part
   size. Nodes along shared edges are bit-identical on both sides, so exact
   welding closes the shell without tolerance guesswork.
3. **Face identity** — STEP transfer can reorder and split faces, so each
   OCCT face is matched back to the Parasolid FACE whose surface it lies on
   (geometric probe, `FaceMatcher`). That keeps `diff` colouring working.
   The curve/surface evaluators in `rust/src/geom/` (LINE, CIRCLE, ELLIPSE,
   B_CURVE, PLANE … BLENDED_EDGE) drive the exporter, the matcher, and the
   redundancy invariants in the tests.

`tessellate()` returns one welded `Mesh` (vertices, triangles, per-triangle
source FACE id, face colors from `SDL/TYSA_COLOUR`); a body the round trip
cannot carry comes back empty and is counted as a failed part. Writers: OBJ
(grouped per face) and binary STL.

## Validation (2026-07-25, 33-part corpus: 14 public samples + 19 vault parts)

Every part meshes; every face of every part tessellates (no fallback-only
faces remain in the corpus — all surface types present are evaluated
exactly). Ring test part volume matches analytic to 0.1%
(+5.10e-5 vs 5.105e-5 m³). Typical boundary-edge counts are 0–100 on parts
up to ~30k triangles; the worst offenders are thread helices (parts 09/13)
and two open sheet bodies where open edges are real geometry.

## Rendering

Two styles, both fed from the same `Mesh`.

**Translucent x-ray SVG** (`rust/src/render.rs`): exact back-to-front
ordering via a BSP tree (crossing polygons split; auto-fallback to centroid
depth sort beyond 12k triangles), real per-face colors (`SDL/TYSA_COLOUR`)
with a `color_map={face_id: rgb}` override hook for diff rendering,
feature-edge strokes (open edges + dihedral > 28°), orthographic or `--fov`
perspective, key/fill lighting with interior (backface) tinting.

**Matte isometric PNG** (`rust/src/iso.rs`): area-weighted random points
are scattered over the triangles, projected isometrically and z-buffered
with a 2×2 splat, then shaded two-sided Lambert
(`0.22 + 0.78·|N·L|`) in one steel-blue hue on a transparent background.
Because cost scales with pixels rather than triangles, a 68k-triangle part
renders in ~6 s at 700 px and a million-triangle part costs the same. The
style (and the annotated contact sheet built on it — name, bounding box,
triangle count, auto-scaled scale bar) matches the GLB tooling in
`tamalpais-configuration`. Framing fits the *projected* silhouette, not the
axis extents, so nothing spills out of frame at any orientation.

## Known gaps

- Face-level orientation is the biggest remaining defect class (#21): on
  NURBS-heavy parts roughly half of all shared edges have their two triangles
  wound opposite each other.
- Thread helices (swept/spun on B-curve profiles) tessellate but with elevated
  open-edge counts; 3_DOF_ARM_SEGMENT still shows a negative signed volume
  (orientation of one face class) — unresolved.
- Blend evaluation assumes center-locus supports (offset walls); exotic
  variable-radius blends would need the thumb_weight/range fields.
- Per-face color survives into `Mesh.colors` but the OBJ writer doesn't emit
  materials yet.
- Doubly-periodic (torus) faces and pole-touching faces rely on a
  material-left heuristic; loop orientation out of assembly is not reliable
  enough to trust globally.
