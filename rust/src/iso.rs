//! Isometric point-splat renderer and annotated contact sheets (PNG).
//!
//! A second visual style alongside the translucent SVG renderer in
//! [`crate::render`], matching the look of the GLB tooling in
//! `tamalpais-configuration` (`tools/render_glb_iso.py`): instead of drawing
//! polygons, it scatters area-weighted random points over the triangles,
//! projects them isometrically, and z-buffers them into an image. Shading is
//! two-sided Lambert (`0.22 + 0.78·|N·L|`) in a single steel-blue hue on a
//! transparent background, which reads as a clean matte solid rather than the
//! translucent x-ray of the SVG style.
//!
//! Sampling rather than rasterising means cost scales with pixels, not
//! triangles, so million-triangle parts render in about the same time as
//! small ones.

use std::path::Path;

use crate::font::{glyph, text_width, ADVANCE, GLYPH_H, GLYPH_W};
use crate::geom::{cross, dot, norm, sub, P3};
use crate::mesh::Mesh;

/// Steel blue, matching the reference tooling.
pub const BASE_COLOR: [u8; 3] = [155, 175, 215];
/// Tokyo-Night background used for contact sheets.
pub const SHEET_BG: [u8; 4] = [26, 27, 38, 255];
const TEXT: [u8; 4] = [192, 202, 245, 255];
const TEXT_DIM: [u8; 4] = [130, 140, 170, 255];
const ACCENT: [u8; 4] = [224, 175, 104, 255];
const GREEN: [u8; 4] = [158, 206, 106, 255];
const BORDER: [u8; 4] = [86, 95, 137, 255];

#[derive(Debug, Clone)]
pub struct IsoOptions {
    /// Azimuth in degrees (rotation about the vertical axis).
    pub az: f64,
    /// Elevation in degrees.
    pub el: f64,
    /// Sample count; `None` picks one from the pixel and triangle counts.
    pub samples: Option<usize>,
    pub size: u32,
    /// Fraction of the image left empty around the part.
    pub margin: f64,
    pub base_color: [u8; 3],
    /// Use each face's own colour (from the file) instead of `base_color`.
    pub face_colors: bool,
    /// Override the fitted bounding box.
    ///
    /// A diff renders two revisions side by side, and each fitting itself
    /// would rescale them independently -- a part that grew would come out the
    /// same size on screen as the one it grew from, which is exactly the thing
    /// the reader is trying to see. Giving both the union of their boxes keeps
    /// them to one scale.
    pub frame: Option<([f64; 3], [f64; 3])>,
}

impl Default for IsoOptions {
    fn default() -> Self {
        IsoOptions {
            az: -35.0,
            el: 25.0,
            samples: None,
            size: 1000,
            margin: 0.06,
            base_color: BASE_COLOR,
            face_colors: false,
            frame: None,
        }
    }
}

/// An RGBA8 image buffer.
pub struct Image {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u8>,
}

impl Image {
    pub fn new(w: usize, h: usize, fill: [u8; 4]) -> Self {
        let mut px = Vec::with_capacity(w * h * 4);
        for _ in 0..w * h {
            px.extend_from_slice(&fill);
        }
        Image { w, h, px }
    }

    #[inline]
    pub fn set(&mut self, x: usize, y: usize, c: [u8; 4]) {
        if x < self.w && y < self.h {
            let i = (y * self.w + x) * 4;
            self.px[i..i + 4].copy_from_slice(&c);
        }
    }

    /// Alpha-composite `src` over this image at (ox, oy).
    pub fn blit(&mut self, src: &Image, ox: usize, oy: usize) {
        for y in 0..src.h {
            for x in 0..src.w {
                let s = (y * src.w + x) * 4;
                let a = src.px[s + 3] as u32;
                if a == 0 {
                    continue;
                }
                let (dx, dy) = (ox + x, oy + y);
                if dx >= self.w || dy >= self.h {
                    continue;
                }
                let d = (dy * self.w + dx) * 4;
                for k in 0..3 {
                    let sv = src.px[s + k] as u32;
                    let dv = self.px[d + k] as u32;
                    self.px[d + k] = ((sv * a + dv * (255 - a)) / 255) as u8;
                }
                self.px[d + 3] = 255;
            }
        }
    }

    pub fn hline(&mut self, x0: usize, x1: usize, y: usize, th: usize, c: [u8; 4]) {
        for t in 0..th.max(1) {
            for x in x0..=x1 {
                self.set(x, y + t, c);
            }
        }
    }

    pub fn vline(&mut self, x: usize, y0: usize, y1: usize, th: usize, c: [u8; 4]) {
        for t in 0..th.max(1) {
            for y in y0..=y1 {
                self.set(x + t, y, c);
            }
        }
    }

    pub fn rect_outline(&mut self, x0: usize, y0: usize, x1: usize, y1: usize, c: [u8; 4]) {
        self.hline(x0, x1, y0, 1, c);
        self.hline(x0, x1, y1, 1, c);
        self.vline(x0, y0, y1, 1, c);
        self.vline(x1, y0, y1, 1, c);
    }

    /// Draw `text` with the built-in 5x7 font, magnified by `scale`.
    pub fn text(&mut self, x: usize, y: usize, text: &str, scale: usize, c: [u8; 4]) {
        let scale = scale.max(1);
        for (i, ch) in text.chars().enumerate() {
            let g = glyph(ch);
            for (col, bits) in g.iter().enumerate().take(GLYPH_W) {
                for row in 0..GLYPH_H {
                    if bits & (1 << row) == 0 {
                        continue;
                    }
                    let px = x + (i * ADVANCE + col) * scale;
                    let py = y + row * scale;
                    for dy in 0..scale {
                        for dx in 0..scale {
                            self.set(px + dx, py + dy, c);
                        }
                    }
                }
            }
        }
    }

    pub fn write_png(&self, path: &Path) -> std::io::Result<()> {
        let file = std::fs::File::create(path)?;
        let w = std::io::BufWriter::new(file);
        let mut enc = png::Encoder::new(w, self.w as u32, self.h as u32);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut writer = enc.write_header()?;
        writer.write_image_data(&self.px)?;
        Ok(())
    }
}

/// Deterministic PCG-XSH-RR; keeps renders reproducible without a dependency.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(6364136223846793005).wrapping_add(1))
    }

    #[inline]
    fn next_u32(&mut self) -> u32 {
        let old = self.0;
        self.0 = old
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    #[inline]
    fn next_f64(&mut self) -> f64 {
        self.next_u32() as f64 / u32::MAX as f64
    }
}

/// Camera rotation: azimuth about Z, then elevation, matching the reference
/// tool's `Rx(el) @ Ry(az)` applied to a Y-up view of a Z-up model.
fn camera(az_deg: f64, el_deg: f64) -> [P3; 3] {
    let (a, e) = (az_deg.to_radians(), el_deg.to_radians());
    // Rows of Rx(e) * Ry(a), with the model's Z mapped to the view's Y so
    // parts stand upright.
    let (ca, sa, ce, se) = (a.cos(), a.sin(), e.cos(), e.sin());
    [
        [ca, sa, 0.0],
        [-se * sa, se * ca, ce],
        [ce * sa, -ce * ca, se],
    ]
}

#[inline]
fn apply(m: &[P3; 3], v: P3) -> P3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

/// Render a mesh in the isometric point-splat style onto a transparent image.
pub fn render_iso(mesh: &Mesh, opts: &IsoOptions) -> Image {
    render_iso_scaled(mesh, opts).0
}

/// As [`render_iso`], also returning the projection scale in pixels per model
/// unit — what a scale bar needs.
pub fn render_iso_scaled(mesh: &Mesh, opts: &IsoOptions) -> (Image, f64) {
    let size = opts.size.max(16) as usize;
    let mut img = Image::new(size, size, [0, 0, 0, 0]);
    if mesh.triangles.is_empty() {
        return (img, 1.0);
    }

    // Per-triangle unit normal and area (the sampling weight).
    let n_tri = mesh.triangles.len();
    let mut normals = Vec::with_capacity(n_tri);
    let mut cum = Vec::with_capacity(n_tri + 1);
    let mut total = 0.0f64;
    cum.push(0.0);
    for t in &mesh.triangles {
        let (a, b, c) = (
            mesh.vertices[t[0] as usize],
            mesh.vertices[t[1] as usize],
            mesh.vertices[t[2] as usize],
        );
        let fnv = cross(sub(b, a), sub(c, a));
        let area = norm(fnv);
        normals.push(if area > 1e-300 {
            [fnv[0] / area, fnv[1] / area, fnv[2] / area]
        } else {
            [0.0, 0.0, 1.0]
        });
        total += area * 0.5;
        cum.push(total);
    }
    if total <= 0.0 {
        return (img, 1.0);
    }

    let rc = camera(opts.az, opts.el);

    let (lo, hi) = opts.frame.unwrap_or_else(|| mesh.bounds());
    let centre = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];
    let margin = opts.margin.clamp(0.0, 0.45);
    let usable = size as f64 * (1.0 - 2.0 * margin);
    let (px_per_unit, cx, cy) = if opts.frame.is_some() {
        // A caller that supplied a frame wants successive renders to line up,
        // and the silhouette fit below cannot do that: a shape's projected
        // width changes as it turns, so fitting it per frame makes a turntable
        // breathe and drift. Fit the bounding sphere of the frame instead --
        // its radius is the same from every direction, so the scale is fixed
        // and the centre stays at the origin.
        let r = (0..3)
            .map(|i| (hi[i] - lo[i]) * 0.5)
            .fold(0.0f64, |a, b| a + b * b)
            .sqrt()
            .max(1e-30);
        (usable / (2.0 * r), 0.0, 0.0)
    } else {
        // Fit the projected silhouette: a shape can be much wider on screen
        // than along any one axis, and normalising by axis extent alone lets
        // corners spill out of frame.
        let (mut xmin, mut xmax, mut ymin, mut ymax) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for v in &mesh.vertices {
            let p = apply(&rc, sub(*v, centre));
            xmin = xmin.min(p[0]);
            xmax = xmax.max(p[0]);
            ymin = ymin.min(p[1]);
            ymax = ymax.max(p[1]);
        }
        let extent = (xmax - xmin).max(ymax - ymin).max(1e-30);
        (usable / extent, (xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
    };
    let light = {
        let l = [-0.4, 0.55, 0.8];
        let n = norm(l);
        [l[0] / n, l[1] / n, l[2] / n]
    };

    // Enough samples to cover every pixel several times over; sampling cost
    // is driven by resolution, not by triangle count.
    let n_samples = opts
        .samples
        .unwrap_or_else(|| (size * size * 6).max(n_tri * 4).clamp(60_000, 12_000_000));

    let mut zbuf = vec![f64::NEG_INFINITY; size * size];
    let mut rng = Rng::new(0x5EED);
    let half = size as f64 / 2.0;

    for _ in 0..n_samples {
        // Pick a triangle with probability proportional to its area.
        let target = rng.next_f64() * total;
        let ti = match cum.binary_search_by(|p| p.partial_cmp(&target).unwrap()) {
            Ok(i) => i.min(n_tri - 1),
            Err(i) => i.saturating_sub(1).min(n_tri - 1),
        };
        // Uniform point in the triangle (fold the unit square onto it).
        let (mut u, mut v) = (rng.next_f64(), rng.next_f64());
        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        let t = mesh.triangles[ti];
        let (a, b, c) = (
            mesh.vertices[t[0] as usize],
            mesh.vertices[t[1] as usize],
            mesh.vertices[t[2] as usize],
        );
        let p = [
            a[0] + u * (b[0] - a[0]) + v * (c[0] - a[0]) - centre[0],
            a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1]) - centre[1],
            a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2]) - centre[2],
        ];
        let sv = apply(&rc, p);
        let x = ((sv[0] - cx) * px_per_unit + half) as isize;
        let y = (-(sv[1] - cy) * px_per_unit + half) as isize;
        if x < 0 || y < 0 || x >= size as isize || y >= size as isize {
            continue;
        }
        let depth = sv[2];
        let nv = apply(&rc, normals[ti]);
        // Two-sided Lambert: back faces of an open shell light like front ones.
        let inten = 0.22 + 0.78 * dot(nv, light).abs().clamp(0.0, 1.0);
        let rgb = if opts.face_colors {
            mesh.colors
                .get(&mesh.face_ids[ti])
                .map(|c| {
                    [
                        (c[0] * 255.0) as u8,
                        (c[1] * 255.0) as u8,
                        (c[2] * 255.0) as u8,
                    ]
                })
                .unwrap_or(opts.base_color)
        } else {
            opts.base_color
        };
        let col = [
            (rgb[0] as f64 * inten).clamp(0.0, 255.0) as u8,
            (rgb[1] as f64 * inten).clamp(0.0, 255.0) as u8,
            (rgb[2] as f64 * inten).clamp(0.0, 255.0) as u8,
            255,
        ];
        // 2x2 splat keeps large flat faces solid at moderate sample counts.
        for dy in 0..2isize {
            for dx in 0..2isize {
                let (px, py) = (x + dx, y + dy);
                if px < 0 || py < 0 || px >= size as isize || py >= size as isize {
                    continue;
                }
                let idx = py as usize * size + px as usize;
                if depth > zbuf[idx] {
                    zbuf[idx] = depth;
                    img.set(px as usize, py as usize, col);
                }
            }
        }
    }
    (img, px_per_unit)
}

// ── Contact sheets ──────────────────────────────────────────────────────────

/// One tile's worth of information.
pub struct Tile {
    pub name: String,
    pub mesh: Mesh,
}

/// Round to 1, 2 or 5 times a power of ten — a readable scale-bar length.
fn nice_number(x: f64) -> f64 {
    if x <= 0.0 || !x.is_finite() {
        return 1.0;
    }
    let e = x.log10().floor();
    let f = x / 10f64.powf(e);
    let m = if f >= 5.0 {
        5.0
    } else if f >= 2.0 {
        2.0
    } else {
        1.0
    };
    m * 10f64.powf(e)
}

/// Format a length in metres with a sensible unit.
fn fmt_len(m: f64) -> String {
    let mm = m * 1000.0;
    if mm < 10.0 {
        format!("{mm:.1} mm")
    } else if mm < 1000.0 {
        format!("{:.1} cm", mm / 10.0)
    } else {
        format!("{:.2} m", mm / 1000.0)
    }
}

fn with_thousands(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    out
}

/// Render tiles into a grid contact sheet: isometric render, part name,
/// bounding-box dimensions, triangle count and an auto-scaled scale bar.
pub fn contact_sheet(tiles: &[Tile], cols: usize, tile_px: usize, opts: &IsoOptions) -> Image {
    let cols = cols.max(1);
    let rows = tiles.len().div_ceil(cols);
    let mut sheet = Image::new(cols * tile_px, rows.max(1) * tile_px, SHEET_BG);
    let thumb = tile_px.saturating_sub(2);
    let s_name = (tile_px / 150).max(1);
    let s_small = (tile_px / 190).max(1);

    for (i, t) in tiles.iter().enumerate() {
        let (row, col) = (i / cols, i % cols);
        let (ox, oy) = (col * tile_px, row * tile_px);

        let mut o = opts.clone();
        o.size = thumb as u32;
        let (img, px_per_unit) = render_iso_scaled(&t.mesh, &o);
        sheet.blit(&img, ox + 1, oy + 1);

        let (lo, hi) = t.mesh.bounds();
        let dims = if t.mesh.vertices.is_empty() {
            [0.0; 3]
        } else {
            [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]
        };

        // Scale bar: a nice round length drawn at the render's own scale.
        if dims.iter().cloned().fold(0.0f64, f64::max) > 0.0 {
            let target = nice_number(0.42 * dims[0].max(dims[1]));
            let bar = (target * px_per_unit) as usize;
            if bar > 4 && bar < thumb {
                let by = oy + thumb - (tile_px as f64 * 0.05) as usize;
                let bx = ox + (tile_px as f64 * 0.04) as usize;
                let th = (tile_px / 140).max(2);
                sheet.hline(bx, bx + bar, by, th, ACCENT);
                sheet.vline(bx, by.saturating_sub(6), by + 6, 2, ACCENT);
                sheet.vline(bx + bar, by.saturating_sub(6), by + 6, 2, ACCENT);
                sheet.text(
                    bx,
                    by.saturating_sub(10 + GLYPH_H * s_small),
                    &fmt_len(target),
                    s_small,
                    ACCENT,
                );
            }
        }

        // Lay the triangle count out first: the name gets whatever width is
        // left, so the two never collide however long the file name is.
        let tris = format!("{} tris", with_thousands(t.mesh.triangles.len()));
        let tw = text_width(&tris, s_small);
        let name_room = tile_px.saturating_sub(tw + 32);
        let max_chars = name_room / (ADVANCE * s_name).max(1);
        let name: String = if t.name.chars().count() > max_chars && max_chars > 1 {
            t.name.chars().take(max_chars - 1).chain(['~']).collect()
        } else {
            t.name.chars().take(max_chars).collect()
        };
        sheet.text(ox + 10, oy + 8, &name, s_name, TEXT);
        sheet.text(
            ox + 10,
            oy + 12 + GLYPH_H * s_name,
            &format!(
                "{:.1}x{:.1}x{:.1} cm",
                dims[0] * 100.0,
                dims[1] * 100.0,
                dims[2] * 100.0
            ),
            s_small,
            TEXT_DIM,
        );
        sheet.text(
            ox + tile_px.saturating_sub(12 + tw),
            oy + 8,
            &tris,
            s_small,
            GREEN,
        );
        sheet.rect_outline(ox, oy, ox + tile_px - 1, oy + tile_px - 1, BORDER);
    }
    sheet
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_cube() -> Mesh {
        let mut m = Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ],
            ..Default::default()
        };
        let faces = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
        ];
        for f in faces {
            m.triangles.push([f[0], f[1], f[2]]);
            m.face_ids.push(1);
        }
        m
    }

    #[test]
    fn renders_a_solid_silhouette_on_transparent_background() {
        let m = unit_cube();
        let img = render_iso(
            &m,
            &IsoOptions {
                size: 120,
                ..Default::default()
            },
        );
        assert_eq!(img.w, 120);
        let opaque = img.px.chunks(4).filter(|p| p[3] == 255).count();
        let total = img.w * img.h;
        // A cube's isometric silhouette is a hexagon: it should fill a good
        // part of the frame, but nothing like all of it (which would mean the
        // fit had let the geometry spill past the edges).
        assert!(
            opaque > total / 5 && opaque < total * 3 / 4,
            "coverage {opaque}/{total}"
        );
        // Corners stay transparent.
        for (x, y) in [(0, 0), (119, 0), (0, 119), (119, 119)] {
            let i = (y * img.w + x) * 4;
            assert_eq!(img.px[i + 3], 0, "corner ({x},{y}) should be clear");
        }
    }

    #[test]
    fn shading_is_two_sided_and_within_range() {
        // Every painted pixel must be the base hue scaled into [0.22, 1.0].
        let img = render_iso(
            &unit_cube(),
            &IsoOptions {
                size: 80,
                ..Default::default()
            },
        );
        let mut seen_dark = false;
        let mut seen_light = false;
        for p in img.px.chunks(4).filter(|p| p[3] == 255) {
            let ratio = p[2] as f64 / BASE_COLOR[2] as f64;
            assert!(
                (0.20..=1.01).contains(&ratio),
                "intensity {ratio} out of range"
            );
            // hue preserved: channels stay in the base ratio
            let r_exp = (BASE_COLOR[0] as f64 * ratio).round();
            assert!((p[0] as f64 - r_exp).abs() <= 2.0);
            if ratio < 0.6 {
                seen_dark = true;
            }
            if ratio > 0.8 {
                seen_light = true;
            }
        }
        assert!(seen_dark && seen_light, "expected a range of face shades");
    }

    #[test]
    fn z_buffer_keeps_the_near_face() {
        // Two parallel plates; the near one is coloured, and with face_colors
        // on, the visible pixels must all come from it.
        let mut m = Mesh::default();
        for (i, z) in [0.0f64, 1.0].iter().enumerate() {
            let b = m.vertices.len() as u32;
            m.vertices.push([-1.0, -1.0, *z]);
            m.vertices.push([1.0, -1.0, *z]);
            m.vertices.push([1.0, 1.0, *z]);
            m.vertices.push([-1.0, 1.0, *z]);
            m.triangles.push([b, b + 1, b + 2]);
            m.triangles.push([b, b + 2, b + 3]);
            m.face_ids.push(i as i16);
            m.face_ids.push(i as i16);
        }
        m.colors.insert(0, [1.0, 0.0, 0.0]); // far plate, z = 0
        m.colors.insert(1, [0.0, 1.0, 0.0]); // near plate, z = 1
        let img = render_iso(
            &m,
            &IsoOptions {
                az: 0.0,
                el: 90.0, // look straight down the model's +Z
                size: 64,
                face_colors: true,
                ..Default::default()
            },
        );
        let (mut red, mut green) = (0, 0);
        for p in img.px.chunks(4).filter(|p| p[3] == 255) {
            if p[0] > p[1] {
                red += 1;
            } else if p[1] > p[0] {
                green += 1;
            }
        }
        assert!(green > 0, "near plate should be visible");
        // The 2x2 splat spreads a pixel past each silhouette edge, so the far
        // plate can fringe the outline; it must not show through the interior.
        assert!(
            red * 50 < green,
            "far plate should be occluded except for splat fringe (red {red}, green {green})"
        );
    }

    #[test]
    fn nice_numbers_and_length_formatting() {
        assert_eq!(nice_number(0.0123), 0.01);
        assert_eq!(nice_number(0.027), 0.02);
        assert_eq!(nice_number(0.06), 0.05);
        assert_eq!(fmt_len(0.005), "5.0 mm");
        assert_eq!(fmt_len(0.05), "5.0 cm");
        assert_eq!(fmt_len(2.0), "2.00 m");
        assert_eq!(with_thousands(1234567), "1,234,567");
    }

    #[test]
    fn contact_sheet_lays_out_a_grid() {
        let tiles: Vec<Tile> = (0..3)
            .map(|i| Tile {
                name: format!("part{i}"),
                mesh: unit_cube(),
            })
            .collect();
        let sheet = contact_sheet(&tiles, 2, 100, &IsoOptions::default());
        assert_eq!(sheet.w, 200);
        assert_eq!(sheet.h, 200); // 3 tiles over 2 columns = 2 rows
                                  // Tile borders sit on the outer edge; just inside is background.
        let at = |x: usize, y: usize| &sheet.px[(y * sheet.w + x) * 4..][..4];
        assert_eq!(at(0, 0), BORDER, "tile edge should be the border colour");
        assert_eq!(at(3, 3), SHEET_BG, "just inside a tile is background");
        // The unused fourth cell stays empty background all the way through.
        assert_eq!(at(150, 150), SHEET_BG, "empty cell should stay blank");
        // ...while an occupied cell has something drawn in it.
        let drawn = (0..100)
            .flat_map(|y| (0..100).map(move |x| (x, y)))
            .any(|(x, y)| at(x, y) != SHEET_BG);
        assert!(drawn, "first tile should have content");
    }
}
