# References: reading the SLDPRT format

Survey of everything known to read SolidWorks part files (`.SLDPRT`) without a
SolidWorks installation, plus prior art for CAD diffing. Compiled 2026-07-23.
Items marked **(verified)** were confirmed against the actual repo/docs;
others come from search results and should be re-checked before depending on
them.

## 1. The file format

- Pre-2015 files are OLE2/CFB (Microsoft Compound File) containers, magic
  `D0 CF 11 E0 A1 B1 1A E1`. **SolidWorks 2015+ files are NOT plain OLE2** —
  they use a proprietary chunked container (marker bytes `14 00 06 00 08 00`),
  encoded stream names, and raw-deflate compression. Most internet advice
  ("just use olefile/gsf") silently assumes pre-2015 files. (verified via
  openswx and blussyya, below)
- Geometry is Parasolid-kernel B-rep, stored per-configuration, proprietary
  and undocumented. Newer files can also embed tessellated display data
  ("Display Data Mark", SW2019+) which is what eDrawings renders.
- Readable without reverse-engineering the geometry: `PreviewPNG` thumbnails
  (document- and per-config), custom properties, mass properties (mass,
  volume, CoG, inertia tensor), configuration names, assembly references,
  cut lists.

Reverse-engineering resources:

- **blussyya/sldprt-format-research** — https://github.com/blussyya/sldprt-format-research
  (MIT, JS, active July 2026). **(verified)** Best public knowledge of the
  modern container. Has decoded the `Contents/DisplayLists` face-block layout
  (vertex positions + normals confirmed across 595 faces); triangle index
  decoding still unsolved. Explicitly research-only, not a converter. See its
  `KNOWN_INVARIANTS.md` / `FAILED_HYPOTHESES.md`.
- **heybryan.org writeup** — http://heybryan.org/solidworks_file_format.html
  **(verified)** Classic OLE2-era RE notes: `gsf` stream browsing, PreviewPNG
  extraction, `Contents/DisplayLists__ZLB` identification.
- **PRONOM fmt/1967** — https://www.nationalarchives.gov.uk/PRONOM/fmt/1967 —
  distinct file signature for SolidWorks 2015+ documents.
- daeken/SLDPRT ("picking apart the 2015 format") — repo now empty; dead end.

## 2. Open-source readers

| Tool | License / lang | Reads from SLDPRT | Notes |
|---|---|---|---|
| [openswx](https://github.com/schwitters/openswx) | MIT, C++20 | Previews (PNG), custom + mass properties, configs, assembly refs, cut lists, doc version. **No geometry.** | **(verified)** Handles BOTH pre-2015 OLE2 and 2015+ chunk container. `swx_dump` CLI with JSON output; Linux-friendly, near-zero deps. Early stage (July 2026, no releases). Best starting point. |
| [sldprt-format-research](https://github.com/blussyya/sldprt-format-research) | MIT, JS | Mesh vertices + normals (partially; indices unsolved) | **(verified)** Research repo, not a library. |
| [olefile](https://github.com/decalage2/olefile) | BSD, Python | Stream listing, previews, property sets — **pre-2015 files only** | Generic OLE2 reader. Useless on 2015+ container. Same for libgsf/`gsf`. |
| FreeCAD | LGPL | **Nothing** — no SLDPRT importer | Standard advice: convert to STEP first. Useful downstream only. |
| [Mayo](https://github.com/fougue/mayo) | BSD-2, C++/OCCT | **Nothing** — no SLDPRT support | **(verified, v0.10.0 July 2026)** Valuable downstream: headless CLI STEP/IGES → glTF/STL. |
| OpenCascade (OCCT) | LGPL | Nothing open-source | SolidWorks + Parasolid import exist only as paid Open Cascade / Datakit add-on components. |
| [sw2urdf](https://github.com/ros/solidworks_urdf_exporter) | MIT, C# | n/a — is a SolidWorks **add-in** (requires SW) | Prior art for mesh-export automation only. |

No libredwg-style community reimplementation of SolidWorks exists; openswx +
blussyya are the state of the art.

## 3. Official-but-free (no SolidWorks install, Windows-only)

- **SOLIDWORKS Document Manager API (swDocMgr)** —
  https://www.codestack.net/solidworks-document-manager-api/ **(verified)**
  Standalone COM DLL, no SolidWorks required; free license key for customers
  on active subscription. Extracts custom properties (r/w), references,
  configs, 2D previews, **tessellation data (if stored in model)**, and
  **Parasolid geometry**. The most complete legitimate no-SolidWorks read
  path — but Windows COM + license-key-gated, awkward for a Linux service.
- **eDrawings API** — https://www.codestack.net/edrawings-api/output/export/
  **(verified example)** Free; batch SLDPRT → STL/PNG/JPG without SolidWorks.
  Windows ActiveX control needing a window host.

## 4. Commercial SDKs (full B-rep, cross-platform)

| SDK | Notes |
|---|---|
| [HOOPS Exchange](https://docs.techsoft3d.com/hoops/exchange/start/format/solidworks_reader.html) (Tech Soft 3D) | SW 97–2026, Win/Linux/mac, C++. Full B-rep via its Parasolid reader **and** direct read of stored tessellation (fast tess-only mode). |
| [CAD Exchanger](https://cadexchanger.com/sldprt/) (CADEX) | C++/C#/Java/Python, Win/Linux/mac. B-rep + meshes + metadata. **CAD Exchanger CLI**: on-prem Linux batch SLDPRT → STEP/glTF/STL + PNG thumbnails — probably the best no-cloud converter. |
| [Datakit CrossManager / CrossCad](https://www.datakit.com/en/) | B-rep readers; the engine behind many third-party viewers and OCC's SolidWorks plug-in. |
| [ODA MCAD SDK](https://www.opendesign.com/products/mcad-sdk) | New (2025). Reads/writes .sldprt/.sldasm/.slddrw, SW 2011–2024, full geometry. Membership licensing. |

## 5. Conversion routes (SLDPRT → STEP/STL/GLB)

Linux-scriptable, no SolidWorks:

- **Zoo (KittyCAD) Design API** — `sldprt` is an explicit input format in the
  official Python client (**verified** in `kittycad/models/file_import_format.py`);
  converts to STEP/STL/glTF/OBJ/PLY via REST. Official Rust CLI
  ([KittyCAD/cli](https://github.com/KittyCAD/cli), MIT, active). Cloud,
  per-use pricing. **Most promising headless route.**
- **Autodesk APS (Forge) Model Derivative** — cloud REST, SLDPRT among
  supported inputs → OBJ/STL/STEP + thumbnails (confirm via its `/formats`
  endpoint).
- **Onshape API** — cloud import of .sldprt (translated to Parasolid), then
  export STEP/Parasolid/STL/glTF. Free plan exists. Import is non-parametric.
- **CAD Exchanger CLI** — on-prem Linux (commercial, §4).

Windows-only: eDrawings API (→STL/PNG), Document Manager API (→Parasolid /
tessellation). Downstream once you have STEP: **Mayo CLI** or FreeCAD CLI →
GLB/STL.

## 6. The Parasolid angle

- SLDPRT embeds Parasolid B-rep (proven by Document Manager's "get Parasolid
  geometry" API). No public code extracts the Parasolid stream from a modern
  SLDPRT — the relevant `Contents/Config-0-Partition` stream is still
  undecoded. Rate as "research project".
- The Parasolid **XT format itself is publicly documented** (Siemens "XT
  Format Reference", public PDF mirrors, e.g.
  http://www.13thmonkey.org/documentation/CAD/Parasolid-XT-format-reference.pdf).
- [khoanguyen-3fc/ps-parser](https://github.com/khoanguyen-3fc/ps-parser)
  **(verified)** — Python, MIT, zero-dep parser for binary `.x_b` node
  streams → JSON topology. Parses structure only; evaluating B-rep needs a
  kernel. Other repos (breploader, parasolid-rs) are wrappers requiring the
  commercial Parasolid SDK.

## 7. Diff prior art

- **SOLIDWORKS Compare utility** — volume-boolean diff coloring
  common/added/removed material, plus feature/property/BOM compare. Requires
  SolidWorks; its add/remove/common coloring is the canonical UX to emulate.
- **GitHub's built-in STL diff** — red=removed, green=added,
  wireframe=unchanged, revision slider
  (https://github.blog/news-insights/product-news/3d-file-diffs/). STL only.
- **Onshape compare** — visual diff between versions with emphasis slider
  (Part Studios only).
- **Argus Diff** — https://argusdiff.dev/ — "git diff for atoms": rendered
  visual diffs, mass/volume deltas, CI gates; STEP/STL/3MF/OBJ/PLY, no SLDPRT.
- **diff3d** — https://github.com/bdlucas1/diff3d — visual 3D diff for
  STL/OBJ/3MF/STEP (pyvista).
- **diffstl** (https://github.com/SebKuzminsky/diffstl),
  **stldiff** (https://github.com/scottlawsonbc/stldiff) — git-integrated STL
  diffs.
- GrabCAD Workbench had version compare — discontinued 2022/2023.

## Takeaways

1. **No open-source library extracts geometry from SLDPRT today.** Open
   source gets thumbnails + metadata + mass properties (openswx); mesh
   extraction is ~80% reverse-engineered (blussyya) with triangle indexing
   unsolved.
2. **Pragmatic geometry pipeline is conversion**: Zoo API (cloud) or CAD
   Exchanger CLI (on-prem) → STEP/GLB → mesh diff render.
3. **Cheap zero-licensing first milestone**: diff the embedded `PreviewPNG`
   thumbnails + mass-property deltas (mass, volume, CoG, inertia) read
   straight from the file — all verified extractable cross-platform.
4. **Mind the 2015+ container gotcha** — `olefile`/`gsf` recipes only work on
   ≤2014 files.
