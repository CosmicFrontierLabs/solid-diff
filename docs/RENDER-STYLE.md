# The matte render style

The look everyone likes — clean matte steel-blue solids on a transparent
background — comes from the isometric point-splat renderer in `src/iso.rs`
(`solid-diff iso`). These are its settings, recorded so the style survives any
future renderer work. The reference implementation it matches is the GLB
tooling in `tamalpais-configuration` (`tools/render_glb_iso.py`).

## Shading

- **Two-sided Lambert:** `intensity = 0.22 + 0.78 · |N·L|`, clamped to [0, 1].
  The absolute value is deliberate: back faces of an open shell light like
  front ones, so reversed winding is invisible (see CLAUDE.md — open edges are
  the only defect a viewer can see).
- **Light direction:** `[-0.4, 0.55, 0.8]`, normalized — over the viewer's
  left shoulder, slightly above.
- **No specular, no shadows, no ambient occlusion.** The 0.22 floor plays the
  role of ambient; everything else is the single directional light.

## Color

- **Base hue:** steel blue `#9BAFD7` (`[155, 175, 215]`), multiplied by the
  Lambert intensity per pixel. One hue for the whole part unless
  `face_colors` pulls per-face colours from the file.
- **Background:** transparent (RGBA 0). Contact sheets composite tiles onto
  Tokyo-Night `#1A1B26` with `#C0CAF5` text, `#565F89` borders, `#E0AF68`
  scale bars, `#9ECE6A` triangle counts.

## Camera and framing

- **Isometric projection** (no perspective), azimuth **-35°**, elevation
  **25°** by default.
- **Margin:** 6% of the image left empty around the part; default 1000 px.
- Diffs override the fitted bounding box with the union of both revisions'
  boxes so growth stays visible (`IsoOptions::frame`).

## Rendering method

Not a rasteriser: area-weighted random points are scattered over the
triangles, projected, and z-buffered, each painted as a **2×2 splat** so flat
faces stay solid. Sample count is `max(6·px², 4·triangles)` clamped to
[60k, 12M], so cost follows resolution rather than triangle count —
million-triangle parts render as fast as small ones.

## The other style

`solid-diff render` (`src/render.rs`) is the translucent SVG "x-ray": BSP
depth-sorted polygons at alpha 0.55 with feature edges and perspective. Use it
when internal structure matters; use the matte iso style for contact sheets,
review previews, and anything a human judges at a glance.
