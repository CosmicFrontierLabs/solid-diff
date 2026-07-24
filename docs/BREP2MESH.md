# B-rep → mesh

`solid_diff/brep2mesh.py` tessellates the Parasolid B-rep extracted from a
SLDPRT (see PARASOLID.md) into a triangle mesh, pure Python (numpy + scipy).

```sh
python -m solid_diff.brep2mesh part.SLDPRT -o part.obj --stl part.stl [--tol 1e-4]
```

## How it works

Per FACE node (`tess.py`):

1. **Boundary loops** — walk `FACE.loop → LOOP.halfedge → HALFEDGE.forward`,
   sample each edge's 3D curve adaptively to the chordal tolerance, and chain
   segments into closed polylines by endpoint continuity. Curve evaluators
   (`geom.py`): LINE, CIRCLE, ELLIPSE, B_CURVE (NURBS via de Boor, from the
   NURBS_CURVE/BSPLINE_VERTICES/KNOT_SET/KNOT_MULT nodes), TRIMMED_CURVE, and
   INTERSECTION (via its CHART's precomputed sample points).
2. **UV mapping** — invert the face's surface analytically: PLANE, CYLINDER,
   CONE, SPHERE, TORUS, SWEPT_SURF (section curve + direction), OFFSET_SURF
   (base + normal offset, fixed-point inversion). Parameterizations only need
   to be eval/inv self-consistent, so Parasolid's exact conventions don't
   matter.
3. **Periodic seam cut** — loops that wind the periodic direction (e.g. the
   two circles bounding a cylinder wall) are unwrapped and joined into one
   simple polygon spanning a single period, bridged at a seam.
4. **Triangulation** — boundary polylines are densified in UV, an interior
   grid is added at a curvature-probed step (midpoint chord error vs
   tolerance), points are metric-scaled (|dS/du|, |dS/dv|) and run through
   scipy Delaunay; triangles are kept if their centroid is inside the outer
   polygon and outside holes (vectorized ray casting).
5. **Back to 3D** — original edge samples keep their exact 3D coordinates
   (so adjacent faces weld watertight); added points are surface-evaluated.
   Triangles are wound outward: the XT outward normal is the parametric
   normal × surface-node sense × face sense — validated by signed-volume
   checks against analytic volumes of the sample parts.
6. **Fallback** — faces on unsupported surfaces (BLENDED_EDGE fillets,
   SPUN_SURF, B_SURFACE, …) are triangulated on a best-fit plane of their
   boundary: coarse but always present.

`tessellate()` welds everything into one `Mesh` (vertices, triangles,
per-triangle source FACE id, face colors from `SDL/TYSA_COLOUR`). Writers:
OBJ (grouped per face) and binary STL.

## Validation (2026-07-24, 4 sample parts)

| part | tris | boundary edges | signed volume |
|---|---|---|---|
| 3_DOF_ARM_BASE | 492 | 0 (watertight) | +5.36e-5 m³ (analytic ≈ +5.1e-5) |
| 4_WHEELER_WHEEL | 300 | 0 (watertight) | +8.27e-5 |
| MacroFeatureMultiExtrude | 417 | 2 | +4.54e-4 |
| bbox-precision | 793 | 234 (fillet fallbacks) | +7.31e-5 |

## Known gaps

- **BLENDED_EDGE** (rolling-ball fillet) faces use the planar fallback →
  open edges where they meet neighbors. Proper fix: evaluate the blend from
  its spine + range, or match the two long boundary chains as a ruled strip.
- SPUN_SURF (revolve) and B_SURFACE (NURBS surface) evaluators not yet
  written (no sample coverage); they fall back too.
- OFFSET_SURF applies the node's sense to the offset sign *and* the normal —
  untested as a face surface (only seen as blend supports).
- Delaunay is unconstrained; extremely coarse `--tol` can let triangles skip
  over concave boundary bits. Boundary densification makes this rare.
- Per-face color survives into `Mesh.colors` but the OBJ writer doesn't emit
  materials yet.
