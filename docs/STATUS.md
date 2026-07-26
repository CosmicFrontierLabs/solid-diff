# Status and remaining work

Snapshot 2026-07-26. Numbers come from sweeps of the **full 1536-part PDM
vault export** plus a 36-part working corpus (`samples/` + `vault/`), measured
with the directed-edge check in `solid-diff mesh --stats`.

## Where we are

The pipeline reads SolidWorks 2015+ part files with no SolidWorks, no
Windows and no commercial SDK, and produces meshes and renders:

```
.SLDPRT → chunk container → zlib sections → Parasolid XT → node graph
        → surface/curve evaluation → tessellation → OBJ/STL/SVG
```

Everything is the Rust crate in `rust/`. An earlier prototype proved the
format and has been removed along with the snapshots it produced; the suite
now tests invariants the files assert about themselves (`tests/invariants.rs`)
rather than agreement with any prior implementation.

### Vault coverage (1536 files)

From a full sweep of the vault export.

| outcome | files | share |
|---|---|---|
| **meshed successfully (Rust)** | **1520** | **99.0%** |
| legacy pre-2015 OLE2 container | 16 | 1.0% |
| carries no B-rep at all (a 1-node stub partition) | 1 | 0.1% |

Rendering all 1536 into 24 isometric contact sheets takes **118 s wall**
(9m16s CPU, 12-way, 690 MB peak) — every part, nothing excluded.

That is **26.5 million triangles** with no crash, hang or unbounded
allocation anywhere in the run. Median part is 5,943 triangles; the largest
is 641,978. Mesh quality at scale is the honest weak spot: only 7.6% of
parts come out fully watertight and the median part has 92 open edges,
which is what #7 (constrained Delaunay) and #5 are about.

Notably the one file that OOM-killed the old prototype's sweep at 37.8 GB (#1)
takes well under a second here — it carries no geometry at all, so that OOM
was a bug in the removed prototype, not a hard part.

74,528 faces and 173,307 edges were surveyed. Everything present is
evaluated exactly except two node types:

| type | instances | share | files affected |
|---|---|---|---|
| `SPUN_SURF` (surface of revolution) | 55 faces | 0.07% | 15 |
| `SP_CURVE` (curve in surface parameter space) | 75 edges | 0.04% | 14 |

Surface mix: PLANE 47.3%, CYLINDER 38.7%, CONE 7.3%, TORUS 2.6%,
B_SURFACE 2.3%, SWEPT_SURF 0.8%, SPHERE 0.6%, BLENDED_EDGE 0.24%.
Curve mix: LINE 53.5%, CIRCLE 36.4%, B_CURVE 3.5%, INTERSECTION 2.9%,
TRIMMED_CURVE 1.8%, ELLIPSE 0.8%.

Storage location splits almost evenly between the older
`Contents/Config-N-Partition` (792 files) and the ~2024
`Config-N-FeatureBodies/LocalBodies` (727) — both are handled, and neither
name is hardcoded (streams are ranked by face count).

### Mesh quality (36-part working corpus)

Measured with the directed-edge check (#19): every undirected edge is
classified once, so the counts do not overlap.

| | before #20 | now |
|---|---|---|
| holes (edge used by one triangle) | 6,023 | **3,354** |
| non-manifold (edge used by >2) | 8,405 | **98** |
| winding mismatches | 7,854 | 10,468 |

Constrained Delaunay plus flood-fill trimming (#20) nearly halved the holes
and all but eliminated non-manifold junctions. The rise in winding mismatches
is disclosure rather than regression: with edges now actually meeting,
orientation errors that used to present as holes surface as what they are.
**No part is yet fully watertight**, and face-level orientation (#21) is the
biggest remaining defect class.

## Open work

All tracked as GitHub issues. Summary of what is left, worst first.

### Correctness

- **[#4] `BLENDED_EDGE` keeps a ~0.2r residual on SWEPT_SURF-supported
  blends.** The arc no longer mirrors across its spine (mean error 0.69r ->
  0.20r, worst face 1.86r -> 0.04r), but 35 of 39 sampled points still miss
  the surface by more than tolerance. The rolling-ball model and the radius
  sign both measured correct; the remaining cause is undiagnosed.
- **[#5] The material-left fallback fires on degenerate sliver faces**, not
  tori as originally thought: 235 faces across 400 vault parts, every one a
  cone whose loop spans nearly a full u period with zero v extent and zero
  area. Three attempted fixes each left the count at exactly 235.
- **[#6] Deltas transmits never parse** (extended node types absent from
  schema 13006). Not ignorable: `CVS-22055[TAM]_03202026__00000914_v2` has a
  307-byte stub partition and keeps all 109 kB of its geometry in the deltas
  transmit, so it silently meshes to zero triangles.

Fixed since the last revision of this document:

| | was | now |
|---|---|---|
| winding mismatches (corpus) | 10,468 | **54** |
| holes (corpus) | 4,201 | 4,092 |
| non-manifold (corpus) | 98 | 25 |
| INTERSECTION off-surface | 96/226 | **11/226** |
| worst INTERSECTION error | 9.2e-2 | **3.1e-3** x model scale |
| SP_CURVE nodes evaluated | 0 | **4,211** |

- **[#21] closed** -- winding is now made consistent across shared edges by
  flood fill, with the global sign per component decided by majority vote
  rather than by forcing positive volume, which would invert internal voids.
- **[#23] closed** -- INTERSECTION curves are refined onto the two surfaces
  they run along, and `inv` projects onto segments instead of snapping to
  stored samples.
- **[#25] closed** -- edges with a null curve are reconstructed by marching
  along the intersection of the two faces that meet there, cutting holes on
  the 119 affected parts from 72,349 to 64,396.
- **[#3] closed** -- SP_CURVE (a B-curve in a surface's parameter space) is
  implemented.

### Performance

- **[#14] Per-face work is not parallelised.** The two pathologies are fixed
  (a uniform-grid index over boundary segments, and analytic NURBS
  derivatives), which took the worst parts from *never finishing* to 22 s and
  the full 1536-part sheet run from ~22 min to **118 s**. Faces are
  independent, so rayon over the per-face loop is the remaining lever.

### Renderer

- **[#9] Feature edges crossing a BSP split are dropped** rather than
  clipped, so silhouettes can gain gaps.

### Documentation

- **[#10]** `docs/FORMAT.md` corrections: ELLIPSE field names, SPUN_SURF
  field names, the fact that §2's section framing is never needed (and that
  a truncated stream must keep its partial inflate), and that the base
  schema yields 104 usable types because `TAG_VALUES` is silently dropped.

### Product

- **[#11] The actual diff renderer does not exist yet.** The renderer takes a
  `color_map: face_id → rgb` override, so the rendering half is ready, but
  faces must be matched **geometrically**: measured across real vault
  revisions, no stored attribute identifies a face from one revision to the
  next (see #11).
- **[#12] No assembly (`.SLDASM`) support**; the vault holds 759 of them.

Closed as out of scope: **[#13]** pre-2015 OLE2 files (see `CLAUDE.md`).
