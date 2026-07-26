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

Everything is the Rust crate in `rust/`. A Python prototype proved the format
and acted as the cross-check oracle while the port was built; it has been
removed, and the golden files it produced live on in `rust/tests/data/` as
frozen ground truth that the Rust decoders are still tested against.

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

Notably the one file that OOM-killed the old Python sweep at 37.8 GB (#1)
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

Historical: while the Python prototype existed, the two agreed to
bit-identical XT decoding (32,473 nodes), geometry evaluation within
4.3e-10 m over 2,157 evaluations, and byte-identical `.x_b` extraction. Rust
was ~17x faster.

## Open work

All tracked as GitHub issues. Summary of what is left, worst first:

### Correctness

- **[#1] One vault part carries no B-rep.**
  `CVS-22055[TAM]_03202026__00000914_v2` has a 1-node stub partition and no
  `LocalBodies` stream, so there is nothing to mesh. (It also OOM-killed the
  removed Python prototype at 37.8 GB; that bug went with it.)
- **[#5] Faces winding in both parameter directions still guess.** Mostly
  fixed: a periodic direction the loops do not cover has a provably-outside
  anchor in the gap, which took corpus open edges from 13,348 to 12,711 and
  the worst part (the screw-mount nut) from 970 open edges to 263. Only
  faces that wind in *both* directions still fall back to the material-left
  heuristic.
- **[#2] `SPUN_SURF` is dead code in Python** — `geom.py` reads
  `section`/`pvec` but the schema defines `profile`/`base`/`axis`/`x_axis`,
  so it always falls back to a plane. 55 faces / 15 files.
- **[#3] `SP_CURVE` unimplemented in both** — 75 edges / 14 files fall back
  to straight chords.
- **[#4] `BLENDED_EDGE range[0]` can be negative**; the sign encodes
  convexity. Python uses it verbatim and mirrors the fillet to the wrong
  side. Rust takes the magnitude.
- **[#7] Unconstrained Delaunay.** Both implementations triangulate then
  filter by centroid, so concave corners can be cut. `spade` is already a
  dependency and supports a real CDT. This is the main lever on the
  watertightness numbers above (7.6% of parts fully closed).
- **[#14] Per-face work is not parallelised.** The two pathologies are fixed
  (a uniform-grid index over boundary segments, and analytic NURBS
  derivatives), which took the worst parts from *never finishing* to 22 s and
  the full 1536-part sheet run from ~22 min to **118 s**. Faces are
  independent, so rayon over the per-face loop is the remaining lever.
- **[#6] Deltas transmits never parse** (extended node types absent from
  schema 13006). Harmless for meshing; blocks reading edit history.

### Renderer bugs found in Python, fixed in Rust

- **[#8] Paint order and the front-face test were both inverted** — the two
  cancel visually, which is how they survived.
- **[#9] Feature edges crossing a BSP split are dropped** rather than
  clipped, so silhouettes can gain gaps (present in both).

### Documentation

- **[#10]** `docs/FORMAT.md` corrections: ELLIPSE field names, SPUN_SURF
  field names, the fact that §2's section framing is never needed (and that
  a truncated stream must keep its partial inflate), and that the base
  schema yields 104 usable types because `TAG_VALUES` is silently dropped.

### Product

- **[#11] The actual diff renderer does not exist yet.** Everything upstream
  is in place: stable `FACE_ID`s survive parsing and the renderer takes a
  `color_map: face_id → rgb` override.
- **[#12] No assembly (`.SLDASM`) support**; the vault holds 759 of them.
- **[#13] Legacy OLE2 (pre-2015) files are unread** — 16 in the vault, all
  purchased vendor content.
