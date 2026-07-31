# Working notes for solid-diff

Context that isn't obvious from the code. See `docs/FORMAT.md` for the file
format itself and `docs/STATUS.md` for current coverage and open work.

## Scope

The full 1536-part PDM vault export lives at
`~/vault_latest/pdm-sldprt-latest/` (not committed; it used to sit in `/tmp`
and got lost on reboot). Corpus sweeps, contact sheets and the review server
all read from it.

**Pre-2015 SolidWorks files are out of scope.** Those use the older OLE2/CFB
container (magic `D0 CF 11 E0 A1 B1 1A E1`) rather than the 2015+ chunked
format everything here is built around. They are 16 of 1536 files in the vault
export (1%), all purchased vendor content (the `M83513_01-AN_PART*` connector
family and `mwdmxx-*`), none of it our own design. `container::parse` returns
`Error::NotModernSldprt` for them and callers are expected to skip. Do not add
OLE2 support without a reason to revisit that trade.

## Layout

Everything is the Rust crate in `rust/`. An earlier prototype proved the
format and was deleted once the port passed it; nothing of it remains, and
nothing here is written to agree with it.

`tools/` holds utilities that are not part of the library: shell and Python
(contact-sheet batching, the render gallery server) plus `frame-review/`, a
standalone axum crate for ✓/✗ frame-by-frame render review — every ✗ appends
a triage task to `renders/review-verdicts/triage_queue.jsonl` for the agent.

## Tessellation architecture

One path: the B-rep is exported to STEP and meshed by OpenCASCADE
(`step.rs` + `occt.rs`, unconditional). The native tessellator and its
per-part quality gate were removed deliberately — do not reintroduce them. A
body the round trip cannot carry comes back as an **empty mesh** and shows up
in the "parts that fail to mesh" budget, not as a silent fallback. Edge
sampling lives on in `sample.rs` (`EdgeSampler`), which the STEP exporter and
the OCCT face matcher share.

The STEP exporter's load-bearing choices: surfaces are exact wherever
STEP has the type, and **edges are sampled polylines** from the shared
`EdgeSampler` — that one choice sidesteps the two-arcs ambiguity, null-curve
edges, `INTERSECTION` and `SP_CURVE` in a single move, and makes both faces
of an edge reference one curve entity so the shell sews. Faces on closed
surfaces get their seam emitted explicitly (the seam edge appears twice in
the wire), because OCCT's reader-side `FixMissingSeam` measurably does not
repair what it drops. The same distrust of reader-side healing applies to
`B_SURFACE` faces: their edges carry **explicit pcurves** (the same sampled
points through `Surface::inv`, on the 3-D polyline's own knots), because the
reader's pcurve *projection* is what failed on thread and vendor-import
patches. XT pads knot arrays with a null-sentinel slot paired with a zero
multiplicity — `knot_arrays` in `geom/curves.rs` tolerates that; reading the
pair with plain `f64_vec` silently discards the whole surface.

The corpus budgets in `tests/invariants.rs` ratchet the OCCT path — the
shipped path — directly.

## Testing

```sh
cd rust
cargo test --release          # 99 tests
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

With CMake ≥ 4 installed, export `CMAKE_POLICY_VERSION_MINIMUM=3.5` or the
occt-sys build of OpenCASCADE fails at configure time (CI already sets it).

CI runs all three. Corpus tests skip themselves when `samples/*.SLDPRT` are
absent (they are fetched by `samples/fetch.sh`, not committed); vault parts in
`vault/` are committed.

The toolchain is pinned in `rust-toolchain.toml` at the repo root, and rustup
reads it automatically, so local and CI lint with the same compiler. This was
not always true: CI tracked floating `stable` while the local install sat at
1.93, and clippy 1.97's `for_kv_map` rejected code that merged clean locally.
Bumping the pin is a normal PR -- change the channel, run clippy, fix what the
new release found.

`fmt`, `clippy` and `test` are three separate CI jobs and all three are
required to merge, so a red check blocks the button rather than relying on
someone noticing.

### What the tests assert against

There is no recorded reference output anywhere in this repo, on purpose. A
snapshot of some earlier run only tells you that behaviour *changed*, and it
preserves that run's mistakes indefinitely — the previous snapshots were one
reverse-engineering's guesses, and 34 of their 149 entries pinned a parse
*failure* as the expected answer.

`tests/invariants.rs` replaces that with claims the files make about
themselves, so a wrong answer can be detected as wrong:

- **Redundancy in the file.** Parasolid stores a VERTEX's coordinates *and*
  the curve it lies on, and stores an EDGE's curve *and* the two surfaces it
  separates. Those must agree, and only correct decoding plus correct
  evaluation makes them. Currently 0/3,628 and 0/10,866 disagreements, worst
  error 3e-10 — so this gate is tight enough to catch essentially any
  evaluator regression.
- **Combinatorial identities**, which need no tolerance: halfedge pairing is
  an involution with opposite senses over one edge, and loops close through
  `backward` covering exactly their member halfedges.
- **Budgets, never snapshots**, for what is still wrong — `INTERSECTION`
  curves, `BLENDED_EDGE` surfaces, and the corpus-wide open-edge count. These may only ever be *lowered*; lower them in the same commit that
  earns it. They are not per-part output, because that would fight every
  legitimate improvement.

The split is deliberate: assert exactly where the maths says the answer is
exact, ratchet where we know we are approximating.

## Measuring render quality

This is a rendering tool, not a meshing one: the number that matters is
**open edges** — places you can see through the part. `solid-diff mesh
--stats` prints the edge report (open / reversed / shared>2 / degenerate,
each undirected edge classified once). Reversed winding is invisible under
two-sided shading and overlapping surface renders as one copy, so neither is
a goal; they are printed because they explain *why* gaps appear. Do not
reintroduce closed-mesh ("watertightness") objectives — pursuing them has
repeatedly cost visible quality here.

## Conventions worth knowing

These were derived empirically and are easy to get backwards:

- Loops chain through the **`backward`** halfedge link, not `forward`.
- Outward normal = parametric normal × surface-node sense × face sense.
- Loop orientation in the file is **not** reliable; trimming must not depend
  on it.
- A surface's `pvec` is only *a* point on an unbounded surface and can sit far
  from the part — never use it to estimate part size (use `POINT` nodes).
