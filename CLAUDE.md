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

Everything is the Rust crate in `rust/`. An earlier prototype proved the
format and was deleted once the port passed it; nothing of it remains, and
nothing here is written to agree with it.

`tools/` holds shell and standalone Python utilities (contact-sheet batching,
the render gallery server); none of it is part of the library.

## Testing

```sh
cd rust
cargo test --release          # 76 tests
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

CI runs all three. Corpus tests skip themselves when `samples/*.SLDPRT` are
absent (they are fetched by `samples/fetch.sh`, not committed); vault parts in
`vault/` are committed.

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
  curves, `BLENDED_EDGE` surfaces, and corpus-wide hole/flip/non-manifold
  counts. These may only ever be *lowered*; lower them in the same commit that
  earns it. They are not per-part output, because that would fight every
  legitimate improvement.

The split is deliberate: assert exactly where the maths says the answer is
exact, ratchet where we know we are approximating.

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
