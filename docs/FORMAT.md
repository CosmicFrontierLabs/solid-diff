# The SLDPRT format, as reverse-engineered

Everything we know about SolidWorks part files (2015+), at the byte level,
compiled 2026-07-25. Written as the implementation spec for the Rust crate in
`rust/`. Confidence markers: **[V]** verified
against our 1500+-file corpus, **[E]** empirical (works everywhere tried, no
spec backing), **[U]** unknown/untested.

Layered picture, outermost first:

```
.SLDPRT file
└─ 1. SW chunk container (proprietary, 2015+)          container.py
   └─ named streams, raw-deflate compressed
      └─ 2. geometry streams: sectioned zlib wrapper    extract.py
         └─ 3. Parasolid XT binary transmit ("PS")      xt.py + vendor/ps-parser
            └─ 4. node graph: topology + geometry       geom.py
               └─ 5. tessellation / rendering           tess.py, render.py
```

PDM note: vault archive blobs (`archive/…/0000000N.SLDPRT`) are **plain
SLDPRT files** — the suspected "PDM compression codec" was this native
container. Pre-2015 files are OLE2/CFB (magic `D0 CF 11 E0 A1 B1 1A E1`) and
out of scope; they appear in the vault only as old vendor imports (~1–2%).

---

## 1. The chunk container (2015+)

Byte order: **little-endian** throughout this layer.

File header **[V]**:

| offset | size | meaning |
|---|---|---|
| 0 | 4 | file-specific (checksum/hash?, varies per file) **[U]** |
| 4 | 4 | constant `00 00 00 04` — format tag |
| 7 | 1 | (overlaps above: the `04` byte) **ROL key** for stream names |
| 8 | … | first chunk begins in here; locate by marker scan |

Detection: not OLE2, not `PK\x03\x04`, and the 6-byte marker
`14 00 06 00 08 00` occurs within the first 64 bytes (empirically at offset
13–21).

Chunks are located by scanning for the marker; a chunk starts 4 bytes before
it (`si = marker_pos − 4`) **[V]**:

| offset | size | field |
|---|---|---|
| si+0x00 | 4 | `val_a` — file-specific tag, unused |
| si+0x04 | 6 | marker `14 00 06 00 08 00` |
| si+0x0a | 1 | section type: `0xDF` TOC, `0xFD` data, `0x1C` mini |
| si+0x0b | 3 | file-specific, unused |
| si+0x0e | 4 | `f1` u32 — `>= 65536` ⇒ inline chunk carrying data |
| si+0x12 | 4 | `csz` u32 — compressed payload size |
| si+0x16 | 4 | `usz` u32 — uncompressed payload size |
| si+0x1a | 4 | `nsz` u32 — stream-name length in bytes |
| si+0x1e | nsz | stream name, **ROL-encoded** UTF-8 |
| si+0x1e+nsz | csz | **raw deflate** payload (inline chunks only) |

Stream-name decode: rotate each byte left by `key & 7` bits (self-inverse
under 8−k). Reject names with bytes < 0x20 or ≥ 0x80 (marks a false-positive
marker hit). Non-inline chunks (`f1 < 65536`) are references without payload
— skip past the marker. First stream wins on duplicate names. Payloads
decompress with raw deflate (zlib, `wbits=-15`).

The file version is encoded in stream *names*: `_MO_VERSION_NNNNN/…`
(`11000` ≈ SW2019-era, `18000` ≈ SW2024 **[E]**). ZIP/OPC (`PK…`) is reserved
for 3DExperience files **[U]**.

### Streams that matter

| stream | content |
|---|---|
| `Contents/Config-N-Partition` | B-rep, older files (see §2) |
| `Config-N-FeatureBodies/LocalBodies` | B-rep, newer (~2024) files — Partition is a 1-node stub there |
| `Contents/Config-N-GhostPartition` | reference wire bodies (skip) |
| `Contents/Config-N-ResolvedFeatures` | feature data; sometimes embeds a plain part transmit |
| `PreviewPNG` | embedded thumbnail (plain PNG) |
| `SW-MassProp*`, `CustomProperty`, `docProps/*` | metadata (see openswx) |
| `Contents/DisplayLists` | display tessellation, partially RE'd by blussyya (unused by us) |

**Don't hardcode**: the robust strategy is to carve *every* stream (§2) and
keep whichever transmits contain FACE nodes, preferring the graph with the
most faces. Multi-config files carry `Config-N-…` per configuration.

---

## 2. Geometry stream wrapper

A geometry stream is a sequence of sections **[V]**:

| offset | size | field |
|---|---|---|
| +0x00 | 4 | u32 LE section length, counted from +0x04 (next section at +0x04+len) |
| +0x04 | 16 | constant magic `23 1d d5 71 da 81 48 a2 a8 58 98 b2 1b 89 ef 99` |
| +0x14 | 4 | u32 LE uncompressed size (exact) |
| +0x18 | 4 | u32 LE compressed size (= zlib stream length − 8, empirically) |
| +0x1c | … | **zlib** data (`78 01`), standard header+adler |

Partition streams: section 1 = `TRANSMIT FILE (partition)`, section 2 =
`TRANSMIT FILE (deltas)` (Parasolid session deltas — ignorable, and it uses
extended node types our parser rejects). LocalBodies streams: one section,
plain `TRANSMIT FILE` (part). In practice we just scan for zlib headers and
try-inflate (`carve_zlib`), which also survives the framing variants.

---

## 3. Parasolid XT binary transmit

Byte order: **big-endian** throughout this layer. Magic `PS`. No ASCII
`**PARASOLID` banner (unlike standalone .x_b exports — prepend one if feeding
external Parasolid consumers).

Primitive encodings **[V]** (from vendor/ps-parser, MIT):

| code | type | encoding |
|---|---|---|
| `u` | u8 | 1 byte |
| `c` | char | ASCII byte(s); arrays are one string |
| `l` | logical | 1 byte, 0/1 |
| `n` | i16 | 2B BE; null sentinel `80 04` (−32764) |
| `w` | utf16 | UTF-16 BE code units |
| `d` | i32 | 4B BE |
| `p` | pointer | i16 BE node id; **value 1 = null** |
| `f` | f64 | 8B BE IEEE-754; null sentinel `C2 BC 92 8F 99 6E 00 00` (−3.14158e13) |
| `i` | interval | 2 × f64 |
| `v`, `h` | vector | 3 × f64 |
| `b` | box | 3 × interval |

Strings: `str_u8_len` = u8 length + ASCII; `str_i32_len` = i32 BE length +
ASCII.

Header **[V]**: `PS`, then str_i32_len modeler version (in SLDPRT the string
begins `": TRANSMIT FILE (partition) created by modeller version NNNNNNN"`),
str_i32_len schema name (`SCH_3601228_36001_13006`-style; suffix `_13006`
matches ps-parser's bundled base schema), i16 `schema_max_type`, i16
`schema_min_type`, 2 unknown bytes.

Node records, repeated until terminator **[V]**:

1. `node_type` i16. Type **1 = terminator**: one more i16 (partition value),
   then EOF.
2. **First occurrence of a type** carries its schema: u8 `field_count`.
   - `field_count == 255`: use the base-schema layout for this type verbatim.
   - Type known in base schema: **delta instructions** until `Z`:
     `C` copy next base field, `D` drop next base field, `I` insert an
     embedded field here, `A` append an embedded field. (Quirk: inserted
     fields' `n_elements` is stored +1 when > 2 **[E]**.)
   - Type NOT in base schema: **full schema**: str_u8 node name, str_u8
     description, then `field_count` embedded field defs.
   - Embedded field def: str_u8 name; i16 `ptr_class`; i16 `n_elements`;
     if `ptr_class == 0` a str_u8 type code (table above) else type is
     pointer; if `n_elements == 2` one extra bool byte (`xmt_code`, marks the
     variable-length tail field).
3. Node payload: if the type is *variable-length*: i32 `count` first. Then
   i16 `id` (the node's graph id). Then each field per the resolved layout;
   `n_elements > 1` means a fixed-size array; for variable types the LAST
   field repeats `count` times.

The base schema (`sch_13006.s_t`) is a text file: lines
`TYPE NAME; description; transmitted n_fields variable\n` followed by field
lines `name; typecode; transmitted nodeclass n_elements`. Ship it with the
crate (ps-parser is MIT).

Node ids are the pointer targets; build a `HashMap<i16, Node>`.

---

## 4. Node graph semantics

### Topology **[V]**

```
WORLD → BODY (body_type: 1=solid, 3=sheet [E]) → REGION → SHELL → FACE
FACE.surface → geometry node        FACE.loop → first LOOP
LOOP.next → sibling loop of same face
LOOP.halfedge → first HALFEDGE;  **traverse via the `backward` link** —
  it visits halfedges in head-to-tail chaining order ('forward' gives the
  reversed cycle)  [E, important]
HALFEDGE.edge → EDGE; HALFEDGE.other → mate; HALFEDGE.vertex → VERTEX|null
HALFEDGE.sense '-' ⇒ this halfedge runs against the edge's direction
EDGE.curve → geometry; EDGE.halfedge → one of its halfedges
VERTEX.point → POINT.pvec (exact endpoint coordinates)
```

Edge direction: the halfedge with sense `'+'` starts at its own `vertex`
and ends at `other.vertex`; sample the curve from `inv(start)` to
`inv(end)`, adding one period when the range comes out ≤ 0 on periodic
curves. Closed edges have null vertices → sample the full range. Edges with
no curve occur (6 in corpus) → straight line between vertices.

**Loop orientation out of assembly is NOT reliable** — do not build
inside/outside logic on CW/CCW or material-left. See §5.

### Attributes **[V]**

`node.attributes_features` → chain (`next`) of ATTRIBUTE nodes.
`ATTRIBUTE.definition → ATTRIB_DEF.identifier → ATT_DEF_ID.string`;
`ATTRIBUTE.fields` → one id or a list of ids of INT_VALUES/REAL_VALUES/…
Notable identifiers: `SDL/TYSA_COLOUR` (face RGB, 3 × f64 0–1),
`FACE_ID_2001` / `ATOM_ID_2001` (stable ids — the diff anchor),
`BODY_RECIPE_2001`, `SWEntUnchanged`.

### Geometry nodes and parameterizations **[V] fields, [E] conventions**

All positions in meters. `y_axis = cross(main_axis, x_axis)`. Every geometry
node has `sense` (`'+'`/`'-'`); surfaces built with `sense_sign = ±1`.

Curves:

| node | fields | eval |
|---|---|---|
| LINE | `pvec`, `direction` | `p + t·d` |
| CIRCLE | `centre`, `normal`, `x_axis`, `radius` | `c + r(cos t·x + sin t·y)`, period 2π |
| ELLIPSE | `centre`, `normal`, `x_axis`, `r1`, `r2` | ditto with r1/r2 |
| B_CURVE | → `nurbs` (NURBS_CURVE) | de Boor; `degree, vertex_dim, n_vertices, rational, closed, periodic`; → BSPLINE_VERTICES `vertices` (flat, v-dim stride; rational = (wx,wy,wz,w)), KNOT_SET `knots` + KNOT_MULT `mult` (expand by repetition); param range `[knot[deg], knot[ncp]]` |
| TRIMMED_CURVE | `basis_curve`, `parm_1/2`, `point_1/2` | basis restricted to [p1,p2] |
| INTERSECTION | `chart` → CHART `hvec` (list of 3D points, ordered; `chordal_error` given) | interpolate the chart polyline; exact eval would need surface–surface intersection |

Surfaces (u,v):

| node | fields | eval |
|---|---|---|
| PLANE | `pvec, normal, x_axis` | `p + u·x + v·y` |
| CYLINDER | `pvec, axis, radius, x_axis` | `p + r(cos u·x + sin u·y) + v·a`; period_u 2π |
| CONE | `pvec, axis, radius, x_axis, sin_half_angle, cos_half_angle` | radius grows `r + v·tan`; apex at `v = −r/tan` (natural bound) |
| SPHERE | `centre, radius, axis, x_axis` | lat/long; v ∈ [−π/2, π/2] natural bounds (poles) |
| TORUS | `centre, axis, x_axis, major_radius, minor_radius` | doubly periodic |
| SWEPT_SURF | `section` (curve), `sweep` (dir), `scale` | `C(u) + v·d`; period_u if section closed |
| SPUN_SURF | `section`, `pvec`, `axis` | profile revolved; u = angle |
| OFFSET_SURF | `surface` (base), `offset`, `sense`, `check`, `true_offset` | `B(uv) + o·n̂(uv)`; sense flips o; inv by fixed-point iteration |
| B_SURFACE | → `nurbs` (NURBS_SURF) | tensor de Boor; `u_degree, v_degree, n_u_vertices, n_v_vertices, vertex_dim, rational, u/v_periodic, u/v_closed`; control net stored **v-fastest** (`(iu,iv) → iu·nv+iv`); u/v knots+mults separate; inversion: Gauss-Newton, grid-seeded |
| BLENDED_EDGE | `blend_type` ('R'=rolling ball), `surface` [2 supports], `spine` (curve), `range` [r,r], `thumb_weight`, `boundary` | ball center on spine; cross-section = arc slerped between tangency directions toward each support; supports are typically the walls **offset by r** (center-locus) — project the center onto the offset's *base* to get direction |
| BLEND_BOUND | `blend`, `boundary` | boundary curves of a blend (appear as loop edges) |

Self-consistency principle: eval/inv only need to invert each other —
Parasolid's exact parameter scaling conventions never matter downstream.

**Outward normal [E, validated by signed volumes]:**
`outward = normalize(∂S/∂u × ∂S/∂v) · surface.sense_sign · face.sense_sign`.

---

## 5. Tessellation algorithm (tess.py — port as-is)

1. **Two-pass shared edge sampling.** Pass 1 (dry run) computes each face's
   UV grid step and *requests* a max 3D spacing per edge; pass 2 samples each
   edge once at the finest requested spacing (adaptive: double n until
   midpoint chord error ≤ tol AND spacing met; exact vertex coords pinned at
   the ends). Both adjacent faces get bit-identical boundary points ⇒ vertex
   welding closes the mesh. Loop assembly chains segments by endpoint
   continuity (eps ≈ 50·tol), reversing when needed.
2. **UV mapping.** `surf.inv(points)`, then unwrap periodic coordinates along
   each polyline (`c[i] −= P·round((c[i]−c[i−1])/P)`), then align every loop
   to the first loop's period window (shift by `P·round(Δmean/P)`).
3. **Domain.** Window = loop bbox; expand to one full period where loops wind
   or are absent; expand a near-zero v-span to natural bounds (pole/apex).
4. **Inside/outside = crossing parity** of segment anchor→query against all
   loop segments, replicated at ±period in each periodic dim. Winding loops
   are closed "the short way" (closing segment targets `first + winding·P`).
   Anchor: any open parameter direction ⇒ place beyond the loops' extent
   (provably outside; use 3 offset anchors + majority vote to dodge
   ray-through-vertex errors). Doubly-periodic faces and pole-extended faces
   have no provable outside ⇒ material-left heuristic anchor with
   empty-result flip as backstop.
5. **Triangulation.** Interior grid at curvature-probed steps (midpoint chord
   error vs tol, cap 96/dim); drop grid points within 0.45 step of any
   boundary *segment* (T-junction prevention); scale UV by metric
   (|∂S/∂u|, |∂S/∂v|); Delaunay (retry with joggle on failure); keep
   triangles whose centroid classifies inside.
6. **Emit.** Boundary points reuse exact shared 3D samples; interior points
   are `surf.eval`. Orient by the outward-normal rule (§4), flipping winding
   wholesale per face. Weld vertices on a `tol·1e−3` grid; drop degenerate
   triangles. Record per-triangle source FACE id + face color.

Tolerance default: 0.2% of the model's coordinate extent.

Known weaknesses (Rust crate should improve): unconstrained Delaunay can cut
concave corners (a true CDT fixes it); thread helices produce elevated
open-edge counts; torus/pole heuristic; one corpus part with a face-class
orientation flip. See docs/BREP2MESH.md "Known gaps".

## 6. Rendering (render.py — port if desired)

Painter's algorithm SVG: camera-space transform, exact back-to-front via BSP
(split crossing polygons; iterative build/traversal — trees exceed recursion
limits; auto-fallback to centroid depth sort > 12k tris), shading
`0.30 + 0.55·key + 0.15·fill` with backface interior tinting, per-face colors
from `SDL/TYSA_COLOUR`, `color_map: face_id → rgb` override (the diff hook),
feature-edge strokes (open edges + dihedral > 28°), orthographic or
perspective.

## 7. Suggested crate layout

| stage | Rust module | notes |
|---|---|---|
| container.py | `container` | pure byte-slicing + `flate2` raw deflate |
| extract.py | `sections` | zlib carve + banner sniff |
| ps-parser | `xt` | schema table + node decoder; ship `sch_13006.s_t`; the delta-schema machinery is essential (newer files embed deltas over the base) |
| xt.py | `graph` | id map + typed accessors + attribute walk |
| geom.py | `geom` | trait `Surface { eval, inv, normal, periods, v_bounds, sense }`, trait `Curve` |
| tess.py | `tess` | consider `spade` (CDT!) instead of filtered Delaunay |
| render.py | `render` | optional; `svg` crate |

Test corpus: `samples/fetch.sh` (public, includes 2 legacy OLE2 negatives)
plus vault parts. Golden checks: ring part volume = 5.105e-5 m³ ±0.5%;
watertightness (boundary-edge count) on the simple sample parts; 19/20 vault
blobs parse with faces.
