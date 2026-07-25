# B-rep → mesh

`solid_diff/brep2mesh.py` tessellates the Parasolid B-rep extracted from a
SLDPRT (see PARASOLID.md) into a triangle mesh, pure Python (numpy + scipy).

```sh
python -m solid_diff.brep2mesh part.SLDPRT -o part.obj --stl part.stl [--tol 1e-4]
```

## How it works

Per FACE node (`tess.py`):

1. **Boundary loops** — walk `FACE.loop → LOOP.halfedge` (via the `backward`
   link, which empirically visits halfedges in chaining order), sampling each
   edge's 3D curve adaptively. Edges are sampled ONCE, shared by both
   adjacent faces via a two-pass `EdgeSampler` (pass 1 negotiates the finest
   spacing any face wants), so neighbors get bit-identical boundary points
   and vertex welding closes the mesh. Curve evaluators (`geom.py`): LINE,
   CIRCLE, ELLIPSE, B_CURVE (NURBS via de Boor), TRIMMED_CURVE, and
   INTERSECTION (via its CHART's precomputed sample points).
2. **UV mapping** — invert the face's surface: PLANE, CYLINDER, CONE, SPHERE,
   TORUS, SWEPT_SURF, SPUN_SURF, OFFSET_SURF (fixed-point inversion),
   B_SURFACE (tensor de Boor + Gauss-Newton inversion), and BLENDED_EDGE
   (rolling-ball fillets: spine param × slerped arc between tangency
   directions recovered by projecting onto the support walls).
   Parameterizations only need to be eval/inv self-consistent, so Parasolid's
   exact conventions don't matter.
3. **Universal trimming** — no seam-cut special cases: boundary loops are
   unwrapped, aligned to a common period window, and points are classified by
   **crossing parity** against all loop segments replicated across parameter
   periods. Whenever a parameter direction is open, the parity anchor sits
   provably outside the loops' extent (majority-voted across 3 anchor rays to
   dodge ray-through-vertex errors), making the test exact and independent of
   loop orientation. This one code path handles plain polygons, holes, any
   winding configuration, hemispheres-to-the-pole (via natural parameter
   bounds), and fully closed surfaces. Doubly-periodic faces (torus) use a
   material-left heuristic with an empty-result flip as backstop.
4. **Triangulation** — interior grid at a curvature-probed step (midpoint
   chord error vs tolerance), grid points within half a step of a boundary
   segment dropped (T-junction prevention), metric-scaled scipy Delaunay,
   triangles kept by centroid parity.
5. **Back to 3D** — edge samples keep their exact shared 3D coordinates;
   added points are surface-evaluated. Triangles are wound outward: the XT
   outward normal is the parametric normal × surface-node sense × face sense
   — validated by signed-volume checks against analytic volumes.
6. **Fallback** — faces on any remaining unsupported surface are triangulated
   on a best-fit plane of their boundary: coarse but always present.

`tessellate()` welds everything into one `Mesh` (vertices, triangles,
per-triangle source FACE id, face colors from `SDL/TYSA_COLOUR`). Writers:
OBJ (grouped per face) and binary STL.

## Validation (2026-07-25, 33-part corpus: 14 public samples + 19 vault parts)

Every part meshes; every face of every part tessellates (no fallback-only
faces remain in the corpus — all surface types present are evaluated
exactly). Ring test part volume matches analytic to 0.1%
(+5.10e-5 vs 5.105e-5 m³). Typical boundary-edge counts are 0–100 on parts
up to ~30k triangles; the worst offenders are thread helices (parts 09/13)
and two open sheet bodies where open edges are real geometry.

## Rendering (`render.py`)

Translucent painter's-algorithm SVG: exact back-to-front ordering via a BSP
tree (crossing polygons split; auto-fallback to centroid depth sort beyond
12k triangles), real per-face colors (`SDL/TYSA_COLOUR`) with a
`color_map={face_id: rgb}` override hook for diff rendering, feature-edge
strokes (open edges + dihedral > 28°), orthographic or `--fov` perspective,
key/fill lighting with interior (backface) tinting.

## Known gaps

- Delaunay is unconstrained; concave boundary corners can occasionally be
  cut, leaving small boundary-edge counts (T-junction prevention and shared
  edge sampling make this rare). A true CDT would finish the job.
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
