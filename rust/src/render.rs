//! Painter's-algorithm SVG renderer for tessellated parts.
//!
//! Triangles are drawn back-to-front as semi-transparent shaded polygons so
//! interior structure shows through. Ordering is exact via a BSP tree (crossing
//! polygons are split) up to a size budget, falling back to centroid depth sort
//! for very large meshes. Real per-face colours are used when present, with a
//! per-face override hook (`color_map`) for diff rendering. Feature edges
//! (sharp dihedrals, open boundaries) are stroked in paint order.
//!
//! Port of `solid_diff/render.py`; see `docs/FORMAT.md` §6. Build and traversal
//! of the BSP are iterative (explicit stacks) — the trees get deep enough on
//! real parts that recursion is not safe.

use std::collections::HashMap;
use std::fmt::Write as _;

use crate::geom::{cross, dot, norm, sub, unit, P3};
use crate::mesh::Mesh;
use crate::value::NodeId;

/// Portal-friendly blue, used when a face carries no colour.
pub const BASE_RGB: [f64; 3] = [0.48, 0.64, 0.97];
const KEY_LIGHT: P3 = [0.45, 0.5, 0.75];
const FILL_LIGHT: P3 = [-0.6, -0.2, 0.35];
/// Triangle count beyond which exact (BSP) ordering costs too much.
pub const BSP_BUDGET: usize = 12000;
const FEATURE_ANGLE_DEG: f64 = 28.0;
/// Interior tint mixed into back faces.
const INTERIOR_TINT: [f64; 3] = [0.25, 0.10, 0.30];

#[derive(Debug, Clone)]
pub struct RenderOptions {
    pub alpha: f64,
    pub elev: f64,
    pub azim: f64,
    /// Perspective field of view in degrees; `None` for orthographic.
    pub fov: Option<f64>,
    pub size: f64,
    pub title: Option<String>,
    pub order: Order,
    pub edges: bool,
    /// Per-face colour override, keyed by FACE node id (the diff hook).
    pub color_map: HashMap<NodeId, [f64; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    Auto,
    Bsp,
    Depth,
}

impl Default for RenderOptions {
    fn default() -> Self {
        RenderOptions {
            alpha: 0.55,
            elev: 28.0,
            azim: -55.0,
            fov: None,
            size: 520.0,
            title: None,
            order: Order::Auto,
            edges: true,
            color_map: HashMap::new(),
        }
    }
}

// ── Geometry helpers ────────────────────────────────────────────────────────

/// View matrix whose rows are the camera's right/up/forward axes in world
/// space. Camera space is x right, y up, z *away from the eye* (into the
/// scene), so larger z means farther away.
pub fn view_matrix(elev_deg: f64, azim_deg: f64) -> [P3; 3] {
    let (e, a) = (elev_deg.to_radians(), azim_deg.to_radians());
    let fwd = [-(e.cos() * a.cos()), -(e.cos() * a.sin()), -e.sin()];
    let mut right = cross(fwd, [0.0, 0.0, 1.0]);
    if norm(right) < 1e-9 {
        right = [1.0, 0.0, 0.0];
    }
    right = unit(right);
    let up = cross(right, fwd);
    [right, up, fwd]
}

#[inline]
fn apply(m: &[P3; 3], v: P3) -> P3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

/// A convex polygon fragment carried through the BSP.
#[derive(Clone, Debug)]
struct Poly {
    /// Camera-space points.
    pts: Vec<P3>,
    /// Unit normal from the camera-space cross product. The camera basis is
    /// left-handed, so this is the *negated* rotated world normal: it points
    /// towards the eye when `normal[2] > 0`.
    normal: P3,
    /// Base rgb.
    color: [f64; 3],
    /// Vertex-index pairs (local to `pts`) to stroke as feature edges.
    edges: Vec<(usize, usize)>,
}

impl Poly {
    fn mean_z(&self) -> f64 {
        self.pts.iter().map(|p| p[2]).sum::<f64>() / self.pts.len() as f64
    }
}

/// Split a convex polygon by the plane `pn·x = pd` into (front, back).
fn split_poly(poly: &Poly, pn: P3, pd: f64, eps: f64) -> (Option<Poly>, Option<Poly>) {
    let n = poly.pts.len();
    let d: Vec<f64> = poly.pts.iter().map(|p| dot(*p, pn) - pd).collect();
    let mut front_pts: Vec<P3> = Vec::new();
    let mut back_pts: Vec<P3> = Vec::new();
    // original vertex index -> new index (used to carry feature-edge marks)
    let mut fmap: Vec<Option<usize>> = vec![None; n];
    let mut bmap: Vec<Option<usize>> = vec![None; n];
    for i in 0..n {
        let j = (i + 1) % n;
        let (di, dj) = (d[i], d[j]);
        if di >= -eps {
            fmap[i] = Some(front_pts.len());
            front_pts.push(poly.pts[i]);
        }
        if di <= eps {
            bmap[i] = Some(back_pts.len());
            back_pts.push(poly.pts[i]);
        }
        if (di > eps && dj < -eps) || (di < -eps && dj > eps) {
            let t = di / (di - dj);
            let seg = sub(poly.pts[j], poly.pts[i]);
            let x = [
                poly.pts[i][0] + t * seg[0],
                poly.pts[i][1] + t * seg[1],
                poly.pts[i][2] + t * seg[2],
            ];
            front_pts.push(x);
            back_pts.push(x);
        }
    }
    let make = |pts: Vec<P3>, vmap: &[Option<usize>]| -> Option<Poly> {
        if pts.len() < 3 {
            return None;
        }
        let edges = poly
            .edges
            .iter()
            .filter_map(|&(a, b)| {
                match (
                    vmap.get(a).copied().flatten(),
                    vmap.get(b).copied().flatten(),
                ) {
                    (Some(a2), Some(b2)) => Some((a2, b2)),
                    _ => None,
                }
            })
            .collect();
        Some(Poly {
            pts,
            normal: poly.normal,
            color: poly.color,
            edges,
        })
    };
    let f = make(front_pts, &fmap);
    let b = make(back_pts, &bmap);
    (f, b)
}

struct BspNode {
    pn: P3,
    pd: f64,
    coplanar: Vec<Poly>,
    front: Option<usize>,
    back: Option<usize>,
}

/// Iterative BSP build (explicit stack; the tree can get very deep).
///
/// Returns an arena of nodes with the root at index 0, or an empty arena for
/// an empty input.
fn build_bsp(polys: Vec<Poly>, eps: f64) -> Vec<BspNode> {
    let mut nodes: Vec<BspNode> = Vec::new();
    if polys.is_empty() {
        return nodes;
    }
    let new_node = |nodes: &mut Vec<BspNode>| -> usize {
        nodes.push(BspNode {
            pn: [0.0, 0.0, 1.0],
            pd: 0.0,
            coplanar: Vec::new(),
            front: None,
            back: None,
        });
        nodes.len() - 1
    };
    let root = new_node(&mut nodes);
    let mut stack: Vec<(usize, Vec<Poly>)> = vec![(root, polys)];
    while let Some((ni, mut items)) = stack.pop() {
        let mid = items.len() / 2;
        let splitter = items.remove(mid);
        let pn = splitter.normal;
        let pd = dot(pn, splitter.pts[0]);
        let mut coplanar = vec![splitter];
        let mut front: Vec<Poly> = Vec::new();
        let mut back: Vec<Poly> = Vec::new();
        for p in items {
            let mut all_near = true;
            let mut all_front = true;
            let mut all_back = true;
            for q in &p.pts {
                let d = dot(*q, pn) - pd;
                if d.abs() > eps {
                    all_near = false;
                }
                if d < -eps {
                    all_front = false;
                }
                if d > eps {
                    all_back = false;
                }
            }
            if all_near {
                coplanar.push(p);
            } else if all_front {
                front.push(p);
            } else if all_back {
                back.push(p);
            } else {
                let (f, b) = split_poly(&p, pn, pd, eps);
                if let Some(f) = f {
                    front.push(f);
                }
                if let Some(b) = b {
                    back.push(b);
                }
            }
        }
        nodes[ni].pn = pn;
        nodes[ni].pd = pd;
        nodes[ni].coplanar = coplanar;
        if !front.is_empty() {
            let c = new_node(&mut nodes);
            nodes[ni].front = Some(c);
            stack.push((c, front));
        }
        if !back.is_empty() {
            let c = new_node(&mut nodes);
            nodes[ni].back = Some(c);
            stack.push((c, back));
        }
    }
    nodes
}

/// Iterative back-to-front traversal for the orthographic view direction
/// `view_dir` (pointing away from the eye, into the scene).
///
/// NOTE: this differs from `render.py`, which emits *front*-to-back (it labels
/// the halfspace the plane normal points into as "near" and then emits that
/// subtree first, and its depth-sort fallback likewise sorts ascending in
/// camera z, i.e. nearest first). With translucent fills that composites the
/// far geometry over the near geometry. Here the far subtree is emitted first,
/// which is what the docstring — and the painter's algorithm — call for.
fn traverse_bsp(nodes: &mut [BspNode], view_dir: P3) -> Vec<Poly> {
    let mut out: Vec<Poly> = Vec::new();
    if nodes.is_empty() {
        return out;
    }
    let mut stack: Vec<(Option<usize>, bool)> = vec![(Some(0), false)];
    while let Some((slot, emit)) = stack.pop() {
        let Some(ni) = slot else { continue };
        if emit {
            out.append(&mut std::mem::take(&mut nodes[ni].coplanar));
            continue;
        }
        // The eye lies in the halfspace the normal points into when
        // pn·view_dir < 0; the *other* halfspace is farther and goes first.
        let (first, second) = if dot(nodes[ni].pn, view_dir) < 0.0 {
            (nodes[ni].back, nodes[ni].front)
        } else {
            (nodes[ni].front, nodes[ni].back)
        };
        stack.push((second, false));
        stack.push((Some(ni), true));
        stack.push((first, false));
    }
    out
}

// ── Feature edges ───────────────────────────────────────────────────────────

/// Per-triangle local vertex-index pairs lying on sharp or open edges.
fn feature_edges(mesh: &Mesh) -> Vec<Vec<(usize, usize)>> {
    let v = &mesh.vertices;
    let t = &mesh.triangles;
    let normals: Vec<P3> = t
        .iter()
        .map(|tri| {
            let (a, b, c) = (v[tri[0] as usize], v[tri[1] as usize], v[tri[2] as usize]);
            let n = cross(sub(b, a), sub(c, a));
            let l = norm(n).max(1e-30);
            [n[0] / l, n[1] / l, n[2] / l]
        })
        .collect();

    // Insertion-ordered edge -> owning triangles, so output is deterministic
    // and matches the Python (which relies on dict insertion order).
    let mut owner: HashMap<(u32, u32), usize> = HashMap::with_capacity(t.len() * 2);
    let mut order: Vec<((u32, u32), Vec<usize>)> = Vec::with_capacity(t.len() * 2);
    for (ti, tri) in t.iter().enumerate() {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = (a.min(b), a.max(b));
            match owner.get(&key) {
                Some(&idx) => order[idx].1.push(ti),
                None => {
                    owner.insert(key, order.len());
                    order.push((key, vec![ti]));
                }
            }
        }
    }

    let cos_lim = FEATURE_ANGLE_DEG.to_radians().cos();
    let mut marks: Vec<Vec<(usize, usize)>> = vec![Vec::new(); t.len()];
    for ((a, b), tris) in &order {
        let sharp = match tris.len() {
            1 => true,
            2 => {
                let (t0, t1) = (tris[0], tris[1]);
                let c = dot(normals[t0], normals[t1]);
                c < cos_lim || (mesh.face_ids[t0] != mesh.face_ids[t1] && c < 0.9995)
            }
            _ => false,
        };
        if !sharp {
            continue;
        }
        for &ti in tris {
            let tri = &t[ti];
            let la = tri.iter().position(|x| x == a);
            let lb = tri.iter().position(|x| x == b);
            if let (Some(la), Some(lb)) = (la, lb) {
                marks[ti].push((la, lb));
            }
        }
    }
    marks
}

// ── Number formatting ───────────────────────────────────────────────────────

/// Compact number: integral values print without a fractional part.
fn num(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e15 {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

/// Python-ish float repr (`1.0` stays `1.0`, `0.55` stays `0.55`).
fn pynum(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e15 {
        format!("{x:.1}")
    } else {
        format!("{x}")
    }
}

fn escape_xml(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            _ => out.push(c),
        }
    }
    out
}

// ── Rendering ───────────────────────────────────────────────────────────────

/// Camera-space vertices plus one `Poly` per non-degenerate triangle.
fn build_polys(mesh: &Mesh, opts: &RenderOptions) -> (Vec<P3>, Vec<Poly>) {
    let m = view_matrix(opts.elev, opts.azim);
    // Camera space: x right, y up, z into the scene (away from the eye).
    let pv: Vec<P3> = mesh.vertices.iter().map(|v| apply(&m, *v)).collect();

    let marks = if opts.edges {
        feature_edges(mesh)
    } else {
        vec![Vec::new(); mesh.triangles.len()]
    };

    let mut polys: Vec<Poly> = Vec::with_capacity(mesh.triangles.len());
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let pts = [
            pv[tri[0] as usize],
            pv[tri[1] as usize],
            pv[tri[2] as usize],
        ];
        let nrm = cross(sub(pts[1], pts[0]), sub(pts[2], pts[0]));
        let ln = norm(nrm);
        if ln < 1e-30 {
            continue;
        }
        let fid = mesh.face_ids[ti];
        let base = opts
            .color_map
            .get(&fid)
            .or_else(|| mesh.colors.get(&fid))
            .copied()
            .unwrap_or(BASE_RGB);
        polys.push(Poly {
            pts: pts.to_vec(),
            normal: [nrm[0] / ln, nrm[1] / ln, nrm[2] / ln],
            color: base,
            edges: marks[ti].clone(),
        });
    }
    (pv, polys)
}

/// Exact (BSP) or approximate (centroid depth) back-to-front paint order.
fn order_polys(pv: &[P3], polys: Vec<Poly>, opts: &RenderOptions) -> Vec<Poly> {
    // scene scale: max |coordinate - mean| over all vertices and axes
    let n = pv.len() as f64;
    let mut mean = [0.0f64; 3];
    for p in pv {
        for k in 0..3 {
            mean[k] += p[k] / n;
        }
    }
    let mut scene_scale = 0.0f64;
    for p in pv {
        for k in 0..3 {
            scene_scale = scene_scale.max((p[k] - mean[k]).abs());
        }
    }
    if scene_scale == 0.0 {
        scene_scale = 1.0;
    }

    let use_bsp = match opts.order {
        Order::Bsp => true,
        Order::Depth => false,
        Order::Auto => polys.len() <= BSP_BUDGET,
    };
    if use_bsp {
        let mut nodes = build_bsp(polys, 1e-9 * scene_scale);
        traverse_bsp(&mut nodes, [0.0, 0.0, 1.0])
    } else {
        // Back-to-front: farthest (largest camera z) first. render.py sorts
        // ascending here, which is the same inversion noted on traverse_bsp.
        let mut p = polys;
        p.sort_by(|a, b| {
            b.mean_z()
                .partial_cmp(&a.mean_z())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        p
    }
}

/// Mean camera-space depth of each polygon fragment, in paint order (largest
/// depth = farthest from the eye first). Exposed for tests.
#[doc(hidden)]
pub fn paint_order_depths(mesh: &Mesh, opts: &RenderOptions) -> Vec<f64> {
    let (pv, polys) = build_polys(mesh, opts);
    if polys.is_empty() {
        return Vec::new();
    }
    order_polys(&pv, polys, opts)
        .iter()
        .map(|p| p.mean_z())
        .collect()
}

/// Render one mesh to an SVG `<g>` fragment of `size`x`size` px.
///
/// `opts.color_map` overrides base colours per FACE node id, the hook diff
/// rendering uses to paint added/removed/unchanged faces.
pub fn render_mesh_svg(mesh: &Mesh, opts: &RenderOptions) -> String {
    let title = opts.title.as_deref().unwrap_or("");
    if mesh.triangles.is_empty() {
        return format!(
            "<g><text y=\"20\" fill=\"#f7768e\">empty mesh: {}</text></g>",
            escape_xml(title)
        );
    }
    let (pv, polys) = build_polys(mesh, opts);
    if polys.is_empty() {
        return "<g></g>".to_string();
    }
    let ordered = order_polys(&pv, polys, opts);

    // ── projection ──
    let perspective = match opts.fov {
        Some(f) if f != 0.0 && f.is_finite() => {
            let mut lo = [f64::INFINITY; 2];
            let mut hi = [f64::NEG_INFINITY; 2];
            let mut zmin = f64::INFINITY;
            let mut zsum = 0.0;
            for p in &pv {
                for k in 0..2 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
                zmin = zmin.min(p[2]);
                zsum += p[2];
            }
            let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
            let eye_z = zmin - span / (2.0 * (f.to_radians() / 2.0).tan());
            let refd = 1.0 / (zsum / pv.len() as f64 - eye_z);
            Some((eye_z, refd))
        }
        _ => None,
    };
    let project = |p: P3| -> [f64; 2] {
        match perspective {
            Some((eye_z, refd)) => {
                let s = (1.0 / (p[2] - eye_z)) / refd;
                [p[0] * s, p[1] * s]
            }
            None => [p[0], p[1]],
        }
    };

    // ── fit to cell ──
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for poly in &ordered {
        for p in &poly.pts {
            let q = project(*p);
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
    }
    let mut span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    if span == 0.0 {
        span = 1.0;
    }
    let size = opts.size;
    let pad = 0.06 * span;
    let scale = size / (span + 2.0 * pad);
    let off_x = pad + (span - (hi[0] - lo[0])) / 2.0;
    let off_y = pad + (span - (hi[1] - lo[1])) / 2.0;
    let px = |p: P3| -> [f64; 2] {
        let q = project(p);
        [
            (q[0] - lo[0] + off_x) * scale,
            size - (q[1] - lo[1] + off_y) * scale,
        ]
    };

    // ── lights (camera space) ──
    let m = view_matrix(opts.elev, opts.azim);
    let key = apply(&m, unit(KEY_LIGHT));
    let fill = apply(&m, unit(FILL_LIGHT));

    let alpha = opts.alpha;
    let stroke_alpha = (alpha + 0.25).min(1.0);
    let mut out = String::with_capacity(ordered.len() * 160);
    out.push_str("<g>");
    if !title.is_empty() {
        let _ = write!(
            out,
            "\n<text x=\"{:.0}\" y=\"16\" text-anchor=\"middle\" \
             fill=\"#c0caf5\" font-size=\"13\" font-family=\"sans-serif\">{}</text>",
            size / 2.0,
            escape_xml(title)
        );
    }
    for p in &ordered {
        // `normal` is a cross product taken in camera space, and the camera
        // basis is left-handed (right x up == -fwd, det == -1), so it is the
        // *negated* rotated world normal. A face pointing at the eye therefore
        // has normal[2] > 0 here.
        //
        // NOTE: render.py tests `normal[2] < 0`, which flags the faces that
        // point away from the eye as front faces — it shades the far shell and
        // puts the interior tint on the near one. That inversion cancels
        // against its inverted paint order (see traverse_bsp), so its images
        // look plausible; both are corrected here. The shading expression below
        // is unchanged: with nl oriented towards the eye in this left-handed
        // frame, `-nl·key` is exactly the diffuse term `n_world·key_world`.
        let facing = p.normal[2] > 0.0;
        let nl = if facing {
            p.normal
        } else {
            [-p.normal[0], -p.normal[1], -p.normal[2]]
        };
        let shade = 0.30 + 0.55 * (-dot(nl, key)).max(0.0) + 0.15 * (-dot(nl, fill)).max(0.0);
        let mut rgb = [p.color[0] * shade, p.color[1] * shade, p.color[2] * shade];
        if !facing {
            for k in 0..3 {
                rgb[k] = rgb[k] * 0.55 + INTERIOR_TINT[k] * 0.45;
            }
        }
        let c: Vec<i32> = rgb
            .iter()
            .map(|c| (255.0 * c.min(1.0)) as i32)
            .map(|v| v.clamp(0, 255))
            .collect();
        let p2: Vec<[f64; 2]> = p.pts.iter().map(|q| px(*q)).collect();
        let mut pts_s = String::with_capacity(p2.len() * 12);
        for (i, q) in p2.iter().enumerate() {
            if i > 0 {
                pts_s.push(' ');
            }
            let _ = write!(pts_s, "{:.1},{:.1}", q[0], q[1]);
        }
        let _ = write!(
            out,
            "\n<polygon points=\"{}\" fill=\"rgb({},{},{})\" \
             fill-opacity=\"{}\" stroke=\"rgb({},{},{})\" \
             stroke-opacity=\"{:.2}\" stroke-width=\"0.3\"/>",
            pts_s,
            c[0],
            c[1],
            c[2],
            pynum(alpha),
            c[0],
            c[1],
            c[2],
            alpha * 0.25
        );
        for &(a, b) in &p.edges {
            if a < p2.len() && b < p2.len() {
                let _ = write!(
                    out,
                    "\n<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" \
                     stroke=\"#e8ecff\" stroke-opacity=\"{:.2}\" stroke-width=\"0.8\"/>",
                    p2[a][0],
                    p2[a][1],
                    p2[b][0],
                    p2[b][1],
                    stroke_alpha * 0.55
                );
            }
        }
    }
    out.push_str("\n</g>");
    out
}

/// Lay fragments out in a `cols`-wide grid of `cell`-sized cells and wrap them
/// in an `<svg>` document. No background rect: these render on a dark page.
pub fn svg_document(fragments: &[String], cols: usize, cell: f64) -> String {
    let cols = cols.max(1);
    let rows = fragments.len().div_ceil(cols);
    let (w, h) = (cols as f64 * cell, rows as f64 * cell);
    let (ws, hs) = (num(w), num(h));
    let mut out = String::new();
    let _ = write!(
        out,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{ws}\" height=\"{hs}\" \
         viewBox=\"0 0 {ws} {hs}\">"
    );
    for (i, frag) in fragments.iter().enumerate() {
        let x = (i % cols) as f64 * cell;
        let y = (i / cols) as f64 * cell;
        let _ = write!(
            out,
            "\n<g transform=\"translate({},{})\">{}</g>",
            num(x),
            num(y),
            frag
        );
    }
    out.push_str("\n</svg>");
    out
}
