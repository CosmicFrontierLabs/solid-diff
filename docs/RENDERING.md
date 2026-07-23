# Rendering notes

Prior art: the CAD-model rendering tooling in `coast-sim-orbit-visualizer`
(deployed via `cf-services` as the `fly` service). Only the rendering stack is
relevant here — the orbit/plan machinery is not.

## What the existing pipeline does

Stack: **three.js ^0.183** rendering a **GLB** (binary glTF, optionally
Draco-compressed) in the browser.

Reusable pieces (paths are into `coast-sim-orbit-visualizer`):

- `src/model_loader.ts` — the core worth lifting:
  - `GLTFLoader` with a `DRACOLoader` fallback chain (tries Draco, then plain,
    then a placeholder box mesh).
  - Auto-fit: computes the model's `Box3`, scales so the bounding diagonal hits
    a target size, recenters to origin. Exactly what a diff renderer wants so
    two part revisions land in the same framing.
  - Material normalization: forces `FrontSide`, zeroes emissive, clamps
    roughness. (Also flips vertex normals — a workaround for that specific
    GLB's inverted normals; don't cargo-cult it.)
- `src/scene_setup.ts:193` (`createSceneGraph`) — renderer/camera/light setup:
  `WebGLRenderer({antialias:true})`, `PerspectiveCamera(45°)`, `OrbitControls`,
  `AmbientLight` + one shadow-casting `DirectionalLight` (2048² PCF soft
  shadows). A sane default studio setup for part renders.
- `src/model_config.ts:68` — config-driven CAD-frame → render-frame rotation
  (SolidWorks Y-up vs three.js conventions), applied at load time.

## What it does NOT have (gaps solid-diff must fill)

1. **CAD → GLB conversion.** The GLB is produced out-of-band and staged from
   the `cfl-models` S3 bucket; no converter exists in either repo. solid-diff
   needs its own SLDPRT → mesh stage (see `REFERENCES.md`).
2. **Headless / static-image rendering.** Rendering is strictly in-browser
   against a live DOM; the renderer isn't even created with
   `preserveDrawingBuffer`, so pixels can't be read back. For producing diff
   images we need one of:
   - **puppeteer/playwright** driving the same three.js scene offscreen
     (`preserveDrawingBuffer: true` or render-to-`WebGLRenderTarget` +
     `readRenderTargetPixels`), or
   - **headless-gl** with three.js in Node, or
   - a native offscreen renderer (e.g. Python: `trimesh`/`pyrender` with EGL,
     or Rust: `rend3`/`wgpu`) if we drop the three.js reuse.

## Diff-rendering sketch

Render both revisions with identical camera/lighting after a **shared**
auto-fit (fit the union of both bounding boxes, not each independently), then
composite: added geometry in one accent color, removed in another, unchanged
neutral — plus per-pixel depth/silhouette diff for a 2D changed-region overlay.
