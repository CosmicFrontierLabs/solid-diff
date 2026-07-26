# Working notes for solid-diff

Context that isn't obvious from the code. See `docs/FORMAT.md` for the file
format itself and `docs/STATUS.md` for current coverage and open work.

## Scope

**Pre-2015 SolidWorks files are out of scope.** Those use the older OLE2/CFB
container (magic `D0 CF 11 E0 A1 B1 1A E1`) rather than the 2015+ chunked
format everything here is built around. They are 16 of 1536 files in the vault
export (1%), all purchased vendor content (the `M83513_01-AN_PART*` connector
family and `mwdmxx-*`), none of it our own design. `container::parse` returns
`Error::NotModernSldprt` for them and callers are expected to skip. Do not add
OLE2 support without a reason to revisit that trade.

## Layout

Everything is the Rust crate in `rust/`. There is no Python implementation: a
prototype proved the format and served as the cross-check oracle during the
port, then was removed. What survives from it is the recorded ground truth in
`rust/tests/data/`:

- `golden.txt` — container/section/XT decoding over the whole corpus, with an
  FNV-1a hash of every field of every node (32,473 nodes across 115 transmits).
- `geom_golden.txt` — 2,157 curve and surface evaluations on real parts.

**These are frozen.** Their generators are gone, so they cannot be
regenerated — treat a mismatch as a regression in the Rust code, never as
stale data to refresh.

`tools/` holds shell and standalone Python utilities (contact-sheet batching,
the render gallery server); none of it is part of the library.

## Testing

```sh
cd rust
cargo test --release          # 71 tests
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

CI runs all three. Corpus tests skip themselves when `samples/*.SLDPRT` are
absent (they are fetched by `samples/fetch.sh`, not committed); vault parts in
`vault/` are committed.

## Measuring mesh quality

`solid-diff mesh --stats` reports the directed-edge manifold check. Use it,
not eyeballing: counting undirected edges only finds holes, and a face wound
backwards keeps two users per edge so the naive count calls it closed. The
report classifies each undirected edge once — hole, winding mismatch,
non-manifold, or degenerate — so the numbers do not overlap.

## Conventions worth knowing

These were derived empirically and are easy to get backwards:

- Loops chain through the **`backward`** halfedge link, not `forward`.
- Outward normal = parametric normal × surface-node sense × face sense.
- Loop orientation in the file is **not** reliable; trimming must not depend
  on it.
- A surface's `pvec` is only *a* point on an unbounded surface and can sit far
  from the part — never use it to estimate part size (use `POINT` nodes).
