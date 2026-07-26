# solid-diff

Renders visual diffs between revisions of SolidWorks part files (`.SLDPRT`).

Given two versions of a part, produce image renderings that highlight what
changed — added/removed/modified geometry — suitable for PR-style review of
CAD changes out of the PDM vault.

## Status

**We read modern (2015+) SLDPRT files end to end with no SolidWorks, no
Windows and no commercial SDK**: chunk container → Parasolid B-rep →
tessellation → OBJ/STL/SVG. On the full 1536-part vault export, 98.9% of
files parse and mesh; the rest are pre-2015 OLE2 containers.

`solid-diff diff` compares two revisions and colours what changed: unchanged
material muted, moved amber, added green, removed red, both sides rendered to
one shared scale. Faces are matched geometrically -- by surface type, that
surface's parameters and the extent of the boundary -- because SolidWorks'
per-face ids turn out to be a feature-provenance chain that no face survives
an edit with.

Everything lives in the Rust crate under `rust/`. The tests assert invariants
the part files state about themselves rather than any recorded reference
output — see `rust/tests/invariants.rs`. For coverage numbers and every known
gap, see
[`docs/STATUS.md`](docs/STATUS.md) for coverage numbers and every known gap.

```sh
cd rust && cargo build --release
R=target/release/solid-diff

$R scan    part.SLDPRT --streams        # container streams + Parasolid found
$R extract part.SLDPRT -o out/          # carve embedded .x_b transmits
$R mesh    part.SLDPRT -o part.obj --stl part.stl --stats
$R render  *.SLDPRT -o sheet.svg --size 360     # translucent x-ray SVG
$R iso     *.SLDPRT -o sheet.png --size 420     # matte isometric PNG
$R diff    old.SLDPRT new.SLDPRT -o diff.png    # what changed between revisions
```

Two rendering styles, for different jobs:

| | `render` (SVG) | `iso` (PNG) |
|---|---|---|
| look | translucent x-ray, feature edges | matte solid, soft shading |
| shows | internal structure through the part | outward form and surface finish |
| method | painter's algorithm over exact BSP order | area-sampled point splats, z-buffered |
| output | vector, scales losslessly | raster, transparent background |
| cost | grows with triangle count | grows with pixels, not triangles |

`iso` also builds annotated contact sheets — part name, bounding box in cm,
triangle count and an auto-scaled scale bar per tile — matching the visual
style of the GLB tooling in `tamalpais-configuration`.

## Docs

- [`docs/STATUS.md`](docs/STATUS.md) — current coverage, mesh-quality
  numbers, and the full list of open work.
- [`docs/FORMAT.md`](docs/FORMAT.md) — **the format spec**: byte-level
  description of everything reverse-engineered, from the container through
  the Parasolid XT node stream to the tessellation algorithm.
- [`docs/PARASOLID.md`](docs/PARASOLID.md) — how the B-rep is embedded and
  extracted (container streams → zlib sections → Parasolid transmit).
- [`docs/BREP2MESH.md`](docs/BREP2MESH.md) — the tessellator: curve/surface
  evaluation, universal trimming, validation results.
- [`REFERENCES.md`](REFERENCES.md) — survey of everything else that can read
  SLDPRT (open-source, commercial, cloud) and prior art in CAD diffing.
- [`docs/RENDERING.md`](docs/RENDERING.md) — notes on the rendering approach.

## Layout

| path | what |
|---|---|
| `rust/src/container.rs` | 2015+ chunk container (ROL names, deflate chunks) |
| `rust/src/sections.rs` | zlib carving + Parasolid transmit sniffing |
| `rust/src/xt/` | Parasolid XT decoder (schema, reader, node records) |
| `rust/src/graph.rs` | node graph: topology, attributes, face colours/ids |
| `rust/src/geom/` | curve and surface evaluators |
| `rust/src/tess.rs` | tessellation (parity trimming, shared edge sampling) |
| `rust/src/render.rs` | painter's-algorithm SVG renderer (BSP ordering) |
| `rust/src/iso.rs` | isometric point-splat PNG renderer + contact sheets |
| `rust/src/font.rs` | 5x7 bitmap font for image annotations |
| `tools/` | contact-sheet batching, render gallery server |
| `samples/fetch.sh` | fetch the public test corpus (16 parts) |

Reference repos (openswx, ps-parser, sldprt-format-research) are cloned into
`vendor/`, which is gitignored.

[#11]: https://github.com/CosmicFrontierLabs/solid-diff/issues/11
