# solid-diff

Renders visual diffs between revisions of SolidWorks part files (`.SLDPRT`).

Given two versions of a part, produce image renderings that highlight what
changed — added/removed/modified geometry — suitable for PR-style review of
CAD changes (e.g. out of the PDM vault).

## Status

**We can extract the full Parasolid B-rep from modern (2015+) SLDPRT files
with pure open-source Python** — see [`docs/PARASOLID.md`](docs/PARASOLID.md).

```sh
./samples/fetch.sh                                    # grab public test parts
python3 -m solid_diff.psscan  samples/part.SLDPRT     # scan for Parasolid data
python3 -m solid_diff.extract samples/part.SLDPRT -o out/   # carve .x_b files
python3 -m solid_diff.brep2mesh samples/part.SLDPRT -o part.obj --stl part.stl
```

Docs:

- [`docs/PARASOLID.md`](docs/PARASOLID.md) — how the B-rep is embedded and
  extracted (container streams → zlib sections → Parasolid binary transmit).
- [`docs/BREP2MESH.md`](docs/BREP2MESH.md) — B-rep → triangle mesh
  tessellator (curve/surface evaluation, seam cutting, validation results).
- [`REFERENCES.md`](REFERENCES.md) — survey of everything that can read the
  SLDPRT format (open-source parsers, converters, commercial SDKs) and prior
  art in CAD diffing.
- [`docs/RENDERING.md`](docs/RENDERING.md) — notes on the rendering approach,
  based on the CAD rendering tooling in `coast-sim-orbit-visualizer`.

Code: `solid_diff/container.py` (2015+ chunk-container parser, ported from
openswx), `solid_diff/psscan.py` (Parasolid signature scanner),
`solid_diff/extract.py` (embedded `.x_b` extractor). Reference repos are
cloned into `vendor/` (gitignored): openswx, ps-parser,
sldprt-format-research.
