# Status and remaining work

Snapshot 2026-07-25, after the Rust port reached parity with the Python
prototype. Numbers come from a census of the **full 1536-part PDM vault
export** (`tools/census.py`) plus a 36-part Python/Rust parity run
(`tools/parity.py`).

## Where we are

The pipeline reads SolidWorks 2015+ part files with no SolidWorks, no
Windows and no commercial SDK, and produces meshes and renders:

```
.SLDPRT → chunk container → zlib sections → Parasolid XT → node graph
        → surface/curve evaluation → tessellation → OBJ/STL/SVG
```

Two implementations exist. **Rust (`rust/`) is the one to build on**; the
Python (`solid_diff/`) is the reference prototype that proved the format and
now serves as the cross-check oracle.

### Vault coverage (1536 files)

| outcome | files | share |
|---|---|---|
| parsed, geometry found | 1519 | 98.9% |
| legacy pre-2015 OLE2 container | 16 | 1.0% |
| killed (runaway memory, see #1) | 1 | 0.1% |

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

### Rust vs Python parity (36-file corpus)

| metric | result |
|---|---|
| files where both produce a mesh | 33 / 33 parseable (3 are OLE2, both reject) |
| XT decode | **bit-identical**: 32,473 nodes, all fields hash-equal |
| geometry evaluation | ≤ 4.3e-10 m over 2157 real evaluations |
| extracted `.x_b` transmits | byte-identical |
| surface area within 2% | 27 / 33 (worst remaining difference 4.8%) |
| triangle count within 25% | 33 / 33 |
| open edges (corpus total) | Python 12,987 · **Rust 12,711** |
| speed | median **6× faster**, up to 98× on NURBS-heavy parts |

Rust is not merely equivalent: it produces **fewer open edges** than Python
across the corpus, and its renderer **fixes two bugs** in the Python one
(see #8). Surface area is the headline comparison because it ignores
orientation and so stays meaningful on open meshes; signed volume does not.

Where the two still differ by a few percent (`template`, the screw-mount
nut, the socket-head screw, `13_570115-99`, `15_M83513_01-FN_PART7`,
`17_NONE-42.step`), neither is verified correct — settling it needs a
ground-truth mesh from a real kernel.

## Open work

All tracked as GitHub issues. Summary of what is left, worst first:

### Correctness

- **[#1] Runaway memory on one vault part.**
  `CVS-22055[TAM]_03202026__00000914_v2` drove the Python sweep to 37.8 GB
  and was OOM-killed — the only file of 1536 we cannot read.
  `tools/census.py` now isolates each file, but the blow-up is unfixed.
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
  dependency and supports a real CDT.
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
