# Extracting Parasolid B-rep from SLDPRT (2015+)

**Status: working.** We can extract the full Parasolid B-rep from modern
SolidWorks part files with pure open-source Python — no SolidWorks, no
Windows, no commercial SDK. Verified 2026-07-23 on 4 public sample parts
(SolidWorks modeller versions 3000298–3100290, i.e. Parasolid v30–v31 /
roughly SW2018–SW2019 era).

This solves what REFERENCES.md §6 rated a "research project": the
`Contents/Config-0-Partition` stream that blussyya's research lists as
undecoded high-entropy data is simply zlib-wrapped Parasolid.

## The pipeline

```
SLDPRT (2015+ chunk container)
  └─ solid_diff/container.py     — Python port of openswx's modern parser
       └─ stream "Contents/Config-N-Partition"
            └─ solid_diff/extract.py — section headers + zlib carve
                 └─ Parasolid binary transmit (.x_b), 'PS' magic
                      └─ vendor/ps-parser — full node-level parse
                           WORLD → BODY[solid] → REGION → SHELL →
                           FACE/LOOP/HALFEDGE/EDGE + PLANE/CIRCLE/
                           NURBS_CURVE/KNOT_SET… + SW attributes
                           (face colors, FACE_ID, BODY_RECIPE…)
```

Try it:

```sh
python3 -m solid_diff.psscan  samples/part.SLDPRT          # find signatures
python3 -m solid_diff.extract samples/part.SLDPRT -o out/  # write .x_b files
cd vendor/ps-parser && python3 cli.py ../../out/part.…partition.x_b --tree
```

All 4/4 sample partitions parsed cleanly (127–540 nodes each), including
NURBS curve geometry. SolidWorks per-face attributes survive, notably
`SDL/TYSA_COLOUR` (face RGB) and stable `FACE_ID_2001` ids — the latter is
gold for diffing, since faces can be identity-matched across revisions
instead of geometrically matched.

## Where the geometry lives

Per configuration N, in the container streams:

| stream | content |
|---|---|
| `Contents/Config-N-Partition` | **the real B-rep**: section 1 = `TRANSMIT FILE (partition)`, section 2 = `TRANSMIT FILE (deltas)` |
| `Contents/Config-N-GhostPartition` | small partition transmit with wire `BODY` + `GHOST_REF_BODY_ID` attrs (reference/ghost bodies) |
| `Contents/Config-N-ResolvedFeatures` | mostly feature data, but can embed a plain `TRANSMIT FILE` (part transmit) at some offset |

## Partition stream layout

A Partition stream is a sequence of sections. Each section:

| offset | size | meaning |
|---|---|---|
| +0x00 | u32 LE | section length counted from offset +0x04 (next section starts at +0x04 + this) |
| +0x04 | 16 B | constant magic `23 1d d5 71 da 81 48 a2 a8 58 98 b2 1b 89 ef 99` (same in every file/stream observed) |
| +0x14 | u32 LE | uncompressed payload size (verified exact, 6/6 observed sections) |
| +0x18 | u32 LE | compressed payload size (empirically = zlib stream length − 8) |
| +0x1c | … | zlib data (`78 01`) |

Section 1 decompresses to a Parasolid **partition transmit**; section 2 to a
**deltas transmit** (Parasolid modeller-session deltas — not needed for a
geometry snapshot, and ps-parser doesn't decode it: it hits node type 257 >
schema max 205, so it uses an extended/different node table).

## The embedded transmit format

Binary Parasolid transmit, exactly what `ps-parser` handles:

```
'PS' | u32be len + banner ": TRANSMIT FILE (partition) created by modeller version NNNNNNN"
     | u32be len + schema  "SCH_3000310_30000_13006"
     | embedded delta-schemas + node stream …
```

Note there is **no** ASCII `**ABCDEF…**PARASOLID` banner block like a
standalone `.x_b` from Parasolid — the file starts straight at `PS`.
ps-parser accepts this; other Parasolid consumers may want the text header
prepended.

The schema suffix `_13006` matches ps-parser's bundled base schema
(`assets/sch_13006.s_t`); newer SolidWorks versions embed delta-schemas
inline, which ps-parser resolves.

## Known gaps / next steps

1. **Deltas transmit** doesn't parse (extended node types). Probably
   ignorable: the partition transmit alone contains the complete solid.
   Verify on more complex parts (multi-body, surfaces, sheet metal).
2. **B-rep → mesh: done** — see `docs/BREP2MESH.md` and
   `solid_diff/brep2mesh.py`. We evaluate the XT geometry ourselves and
   tessellate to OBJ/STL. Faces on unsupported surfaces (notably
   `BLENDED_EDGE` fillets) currently get a best-fit-plane fallback.
3. **Coverage:** only Config-0 single-config parts tested; test multi-config,
   assemblies (`.SLDASM`), and current (2024+) files — ideally real parts
   from the PDM vault.
4. The 16-byte section magic and the −8 in the compressed-size field are
   unexplained (cosmetic; extraction doesn't depend on them).
