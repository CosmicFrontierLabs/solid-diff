# solid-diff

Renders visual diffs between revisions of SolidWorks part files (`.SLDPRT`).

Given two versions of a part, produce image renderings that highlight what
changed — added/removed/modified geometry — suitable for PR-style review of
CAD changes (e.g. out of the PDM vault).

## Status

Research phase. Start here:

- [`REFERENCES.md`](REFERENCES.md) — survey of everything that can read the
  SLDPRT format (open-source parsers, converters, commercial SDKs) and prior
  art in CAD diffing.
- [`docs/RENDERING.md`](docs/RENDERING.md) — notes on the rendering approach,
  based on the CAD rendering tooling in `coast-sim-orbit-visualizer`.
