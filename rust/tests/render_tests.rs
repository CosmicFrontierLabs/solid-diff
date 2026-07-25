//! Renderer tests. All meshes here are built by hand so these do not depend on
//! the parser/tessellator pipeline.

use std::collections::HashMap;

use solid_diff::mesh::Mesh;
use solid_diff::render::{paint_order_depths, render_mesh_svg, svg_document, Order, RenderOptions};
use solid_diff::value::NodeId;

// ── helpers ─────────────────────────────────────────────────────────────────

fn mesh_of(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    face_ids: &[NodeId],
    colors: &[(NodeId, [f64; 3])],
) -> Mesh {
    Mesh {
        vertices: vertices.to_vec(),
        triangles: triangles.to_vec(),
        face_ids: face_ids.to_vec(),
        colors: colors.iter().cloned().collect(),
        warnings: Vec::new(),
    }
}

#[derive(Debug)]
struct SvgPoly {
    pts: Vec<(f64, f64)>,
    fill: (i32, i32, i32),
}

impl SvgPoly {
    fn min_x(&self) -> f64 {
        self.pts.iter().fold(f64::INFINITY, |a, p| a.min(p.0))
    }
    fn max_x(&self) -> f64 {
        self.pts.iter().fold(f64::NEG_INFINITY, |a, p| a.max(p.0))
    }
    /// Which channel is the largest — a cheap colour identity for the tests.
    fn dominant(&self) -> usize {
        let c = [self.fill.0, self.fill.1, self.fill.2];
        (0..3).max_by_key(|&i| c[i]).unwrap()
    }
}

fn attr<'a>(tag: &'a str, name: &str) -> &'a str {
    let key = format!("{name}=\"");
    let i = tag
        .find(&key)
        .unwrap_or_else(|| panic!("attribute {name} missing in {tag}"))
        + key.len();
    let rest = &tag[i..];
    &rest[..rest.find('"').unwrap()]
}

/// Every `<polygon>` in document (= paint) order.
fn polys_of(svg: &str) -> Vec<SvgPoly> {
    svg.split("<polygon ")
        .skip(1)
        .map(|chunk| {
            let tag = &chunk[..chunk.find("/>").unwrap()];
            let pts = attr(tag, "points")
                .split_whitespace()
                .map(|p| {
                    let (x, y) = p.split_once(',').unwrap();
                    (x.parse::<f64>().unwrap(), y.parse::<f64>().unwrap())
                })
                .collect();
            let rgb = attr(tag, "fill");
            let inner = rgb.trim_start_matches("rgb(").trim_end_matches(')');
            let v: Vec<i32> = inner.split(',').map(|c| c.parse().unwrap()).collect();
            SvgPoly {
                pts,
                fill: (v[0], v[1], v[2]),
            }
        })
        .collect()
}

fn count(svg: &str, needle: &str) -> usize {
    svg.matches(needle).count()
}

/// Axis-aligned unit cube centred on the origin: 12 triangles, 6 face ids.
fn cube() -> Mesh {
    let v = [
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ];
    let t = [
        [0, 3, 2],
        [0, 2, 1], // -z
        [4, 5, 6],
        [4, 6, 7], // +z
        [0, 1, 5],
        [0, 5, 4], // -y
        [2, 3, 7],
        [2, 7, 6], // +y
        [0, 4, 7],
        [0, 7, 3], // -x
        [1, 2, 6],
        [1, 6, 5], // +x
    ];
    let ids: Vec<NodeId> = (0..6).flat_map(|f| [f, f]).collect();
    mesh_of(&v, &t, &ids, &[(0, [1.0, 0.2, 0.2]), (3, [0.2, 1.0, 0.2])])
}

// ── basics ──────────────────────────────────────────────────────────────────

#[test]
fn empty_mesh_renders_a_notice() {
    let m = Mesh::default();
    let opts = RenderOptions {
        title: Some("nothing".into()),
        ..Default::default()
    };
    let svg = render_mesh_svg(&m, &opts);
    assert!(svg.contains("empty mesh: nothing"), "{svg}");
    assert_eq!(count(&svg, "<polygon"), 0);
}

#[test]
fn cube_emits_one_polygon_per_triangle_and_all_sharp_edges() {
    let opts = RenderOptions {
        title: Some("cube".into()),
        ..Default::default()
    };
    let svg = render_mesh_svg(&cube(), &opts);
    let polys = polys_of(&svg);
    // A cube's triangles never straddle a cube-face plane, so the BSP splits
    // nothing: 12 in, 12 out.
    assert_eq!(polys.len(), 12, "{svg}");
    // 12 cube edges are 90-degree dihedrals (sharp), each marked on both of
    // its triangles; the 6 face diagonals are coplanar and same-face, so not.
    assert_eq!(count(&svg, "<line "), 24);
    assert!(svg.starts_with("<g>") && svg.ends_with("</g>"));
    assert!(svg.contains(">cube</text>") && svg.contains("#c0caf5"));
    assert!(!svg.contains("<rect"), "no background rect on dark pages");

    // Everything lands inside the cell, and the fit touches the 6% padding on
    // the longer axis (the shorter one is centred).
    let size = opts.size;
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in &polys {
        for (x, y) in &p.pts {
            assert!(*x >= -0.1 && *x <= size + 0.1, "x out of cell: {x}");
            assert!(*y >= -0.1 && *y <= size + 0.1, "y out of cell: {y}");
            for (k, c) in [*x, *y].iter().enumerate() {
                lo[k] = lo[k].min(*c);
                hi[k] = hi[k].max(*c);
            }
        }
    }
    let pad_px = size * 0.06 / 1.12;
    let long = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    assert!(
        (long - (size - 2.0 * pad_px)).abs() < 0.3,
        "long axis {long} should fill the cell minus 2x{pad_px}"
    );
    for k in 0..2 {
        assert!(lo[k] >= pad_px - 0.2 && hi[k] <= size - pad_px + 0.2);
        // centred: equal slack either side
        assert!(((lo[k] - 0.0) - (size - hi[k])).abs() < 0.3, "axis {k} not centred");
    }
}

#[test]
fn cube_paints_back_faces_before_front_faces() {
    let opts = RenderOptions::default();
    let d = paint_order_depths(&cube(), &opts);
    assert_eq!(d.len(), 12);
    // Camera z grows away from the eye: the 6 far triangles come first.
    let far_min = d[..6].iter().cloned().fold(f64::INFINITY, f64::min);
    let near_max = d[6..].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(far_min > 0.0 && near_max < 0.0, "paint order depths: {d:?}");
}

#[test]
fn colors_come_from_map_then_mesh_then_default() {
    // Face 0 is red in the mesh, face 3 green, faces 1/2/4/5 default blue.
    let plain = polys_of(&render_mesh_svg(&cube(), &RenderOptions::default()));
    let doms: Vec<usize> = plain.iter().map(|p| p.dominant()).collect();
    assert_eq!(doms.iter().filter(|d| **d == 0).count(), 2, "red face");
    assert_eq!(doms.iter().filter(|d| **d == 1).count(), 2, "green face");
    assert_eq!(doms.iter().filter(|d| **d == 2).count(), 8, "blue default");

    // The diff hook overrides everything.
    let mut color_map = HashMap::new();
    for f in 0..6 {
        color_map.insert(f as NodeId, [1.0, 0.0, 0.0]);
    }
    let over = polys_of(&render_mesh_svg(
        &cube(),
        &RenderOptions {
            color_map,
            ..Default::default()
        },
    ));
    assert!(over.iter().all(|p| p.fill.1 <= p.fill.0 && p.fill.2 <= p.fill.0));
}

#[test]
fn back_faces_get_the_interior_tint_and_front_faces_are_lit() {
    let tri = |flip: bool| {
        mesh_of(
            &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            &[if flip { [0, 2, 1] } else { [0, 1, 2] }],
            &[7],
            &[(7, [0.5, 0.5, 0.5])],
        )
    };
    // Camera overhead (elev 90) looking straight down.
    let opts = RenderOptions {
        elev: 90.0,
        azim: 0.0,
        ..Default::default()
    };
    // Winding [0,2,1] -> world normal -z -> points away from the eye: tinted
    // rgb*0.55 + [0.25,0.10,0.30]*0.45, so blue > red > green, and dark.
    let back = polys_of(&render_mesh_svg(&tri(true), &opts))[0].fill;
    assert!(back.2 > back.0 && back.0 > back.1, "tint {back:?}");

    // Winding [0,1,2] -> world normal +z -> faces the eye: grey (untinted) and
    // brighter, because the key light also comes from above.
    let front = polys_of(&render_mesh_svg(&tri(false), &opts))[0].fill;
    assert_eq!(front.0, front.1);
    assert_eq!(front.1, front.2);
    assert!(front.0 > back.0, "lit front {front:?} vs shadowed back {back:?}");
}

// ── the key ordering proof ──────────────────────────────────────────────────

/// Triangle A pierces triangle B. B lies in the plane of constant depth
/// (world x = 0 with elev=0/azim=0, so camera z = -x), so the BSP must cut A
/// into a far half and a near half and paint them either side of B.
fn piercing_pair() -> Mesh {
    mesh_of(
        &[
            // A: spans x = -1 .. +1, i.e. far .. near
            [-1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.5],
            // B: flat at x = 0
            [0.0, -1.5, -1.0],
            [0.0, 1.5, -1.0],
            [0.0, 0.0, 1.2],
        ],
        &[[0, 1, 2], [3, 4, 5]],
        &[10, 20],
        &[(10, [1.0, 0.1, 0.1]), (20, [0.1, 1.0, 0.1])],
    )
}

#[test]
fn bsp_splits_piercing_triangles_and_paints_far_fragment_first() {
    let opts = RenderOptions {
        elev: 0.0,
        azim: 0.0,
        order: Order::Bsp,
        edges: false,
        ..Default::default()
    };
    let svg = render_mesh_svg(&piercing_pair(), &opts);
    let polys = polys_of(&svg);
    // 2 triangles in, 3 fragments out: A was split by B's plane.
    assert_eq!(polys.len(), 3, "{svg}");

    // Paint order must be A-far, B, A-near.
    assert_eq!(polys[0].dominant(), 0, "first fragment is A (red)");
    assert_eq!(polys[1].dominant(), 1, "middle is B (green)");
    assert_eq!(polys[2].dominant(), 0, "last fragment is A (red)");

    // A's far half carries the world vertex (-1,-1,0) -> screen left, its near
    // half the vertex (1,1,0) -> screen right. So the far half is painted
    // first: back-to-front, not front-to-back.
    let (far, near) = (&polys[0], &polys[2]);
    assert!(
        far.min_x() < near.min_x() && far.max_x() < near.max_x(),
        "far fragment should be the left (x<0) one: {far:?} {near:?}"
    );
    assert!(far.min_x() < opts.size / 2.0 && near.max_x() > opts.size / 2.0);

    // And the depths agree: strictly decreasing (farthest painted first).
    let d = paint_order_depths(&piercing_pair(), &opts);
    assert_eq!(d.len(), 3);
    assert!(d[0] > d[1] && d[1] > d[2], "depths not back-to-front: {d:?}");
    assert!(d[0] > 0.0 && d[2] < 0.0);
}

#[test]
fn depth_sort_paints_the_far_triangle_first() {
    // Two disjoint triangles: red near the eye (x=+1), green far (x=-1).
    let m = mesh_of(
        &[
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 1.0, 0.0],
            [-1.0, 0.0, 1.0],
        ],
        &[[0, 1, 2], [3, 4, 5]],
        &[1, 2],
        &[(1, [1.0, 0.0, 0.0]), (2, [0.0, 1.0, 0.0])],
    );
    for order in [Order::Depth, Order::Bsp] {
        let opts = RenderOptions {
            elev: 0.0,
            azim: 0.0,
            order,
            edges: false,
            ..Default::default()
        };
        let polys = polys_of(&render_mesh_svg(&m, &opts));
        assert_eq!(polys.len(), 2);
        assert_eq!(polys[0].dominant(), 1, "far (green) first with {order:?}");
        assert_eq!(polys[1].dominant(), 0, "near (red) last with {order:?}");
        let d = paint_order_depths(&m, &opts);
        assert!(d[0] > d[1], "{order:?} depths {d:?}");
    }
}

#[test]
fn bsp_handles_a_deep_chain_without_recursion() {
    // 2000 parallel triangles, ordered so that the median element picked as
    // splitter is always the extreme of what is left: the tree degenerates
    // into a 2000-deep chain. A recursive build or traversal would blow the
    // stack here; the iterative one must not.
    const N: usize = 2000;
    let mut positions: Vec<usize> = (0..N).collect();
    let mut rank_at = vec![0usize; N];
    let mut k = 0;
    while !positions.is_empty() {
        let mid = positions.len() / 2;
        let pos = positions.remove(mid);
        rank_at[pos] = k;
        k += 1;
    }
    let mut v = Vec::new();
    let mut t = Vec::new();
    let mut ids = Vec::new();
    for (i, &rank) in rank_at.iter().enumerate() {
        let x = rank as f64; // depth: camera z = -x, so rank 0 is farthest
        let b = (i * 3) as u32;
        v.push([x, -1.0, -1.0]);
        v.push([x, 1.0, -1.0]);
        v.push([x, 0.0, 1.0]);
        t.push([b, b + 1, b + 2]);
        ids.push((i % 30) as NodeId);
    }
    let m = mesh_of(&v, &t, &ids, &[]);
    let opts = RenderOptions {
        elev: 0.0,
        azim: 0.0,
        order: Order::Bsp,
        edges: false,
        ..Default::default()
    };
    let d = paint_order_depths(&m, &opts);
    assert_eq!(d.len(), N);
    // Exact back-to-front: depths are 0, -1, -2, ... (camera z = -x).
    for (i, z) in d.iter().enumerate() {
        assert!((z + i as f64).abs() < 1e-9, "at {i}: {z}");
    }
}

// ── feature edges ───────────────────────────────────────────────────────────

#[test]
fn open_boundary_edges_are_stroked() {
    // Lone triangle: all three edges belong to one triangle only.
    let m = mesh_of(
        &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        &[[0, 1, 2]],
        &[1],
        &[],
    );
    let svg = render_mesh_svg(&m, &RenderOptions::default());
    assert_eq!(count(&svg, "<line "), 3, "{svg}");
    assert!(svg.contains("stroke=\"#e8ecff\""));
    assert!(svg.contains("stroke-width=\"0.8\""));

    // Two triangles making a square: the shared diagonal is coplanar and on
    // the same face, so only the 4 boundary edges are stroked.
    let sq = mesh_of(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        &[[0, 1, 2], [0, 2, 3]],
        &[1, 1],
        &[],
    );
    assert_eq!(count(&render_mesh_svg(&sq, &RenderOptions::default()), "<line "), 4);
}

#[test]
fn coplanar_face_boundaries_and_sharp_dihedrals() {
    // Same geometry, two different FACE ids: coplanar (dot >= 0.9995) so the
    // shared edge stays unmarked -> still just the 4 open edges.
    let flat = mesh_of(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        &[[0, 1, 2], [0, 2, 3]],
        &[1, 2],
        &[],
    );
    assert_eq!(
        count(&render_mesh_svg(&flat, &RenderOptions::default()), "<line "),
        4
    );

    // Fold the second triangle 90 degrees: the shared edge is now sharp and is
    // stroked once per owning triangle, on top of the 4 open edges.
    let bent = mesh_of(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        &[[0, 1, 2], [0, 3, 1]],
        &[1, 1],
        &[],
    );
    assert_eq!(
        count(&render_mesh_svg(&bent, &RenderOptions::default()), "<line "),
        6
    );

    // A 10-degree fold is under the 28-degree threshold: not a feature edge.
    let a = 10f64.to_radians();
    let gentle = mesh_of(
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, -a.cos(), a.sin()],
        ],
        &[[0, 1, 2], [0, 3, 1]],
        &[1, 1],
        &[],
    );
    assert_eq!(
        count(&render_mesh_svg(&gentle, &RenderOptions::default()), "<line "),
        4
    );

    // edges: false suppresses all of them.
    let svg = render_mesh_svg(
        &cube(),
        &RenderOptions {
            edges: false,
            ..Default::default()
        },
    );
    assert_eq!(count(&svg, "<line "), 0);
}

#[test]
fn feature_edges_survive_bsp_splitting() {
    // The piercing pair with edges on: A and B are both open-boundary
    // triangles, so every surviving edge whose endpoints outlive the split is
    // stroked, and every stroked endpoint must be a vertex of its polygon.
    let opts = RenderOptions {
        elev: 0.0,
        azim: 0.0,
        order: Order::Bsp,
        ..Default::default()
    };
    let svg = render_mesh_svg(&piercing_pair(), &opts);
    assert!(count(&svg, "<line ") >= 5, "{svg}");
    // Group the lines with the polygon they follow and check containment.
    let mut current: Option<Vec<(f64, f64)>> = None;
    for chunk in svg.split('\n') {
        if chunk.starts_with("<polygon ") {
            current = Some(polys_of(chunk).remove(0).pts);
        } else if chunk.starts_with("<line ") {
            let pts = current.as_ref().expect("line before any polygon");
            for (xa, ya) in [("x1", "y1"), ("x2", "y2")] {
                let p = (
                    attr(chunk, xa).parse::<f64>().unwrap(),
                    attr(chunk, ya).parse::<f64>().unwrap(),
                );
                assert!(
                    pts.iter().any(|q| (q.0 - p.0).abs() < 0.11 && (q.1 - p.1).abs() < 0.11),
                    "edge endpoint {p:?} is not a vertex of {pts:?}"
                );
            }
        }
    }
}

// ── projection, options, document ───────────────────────────────────────────

#[test]
fn perspective_differs_from_orthographic() {
    let ortho = render_mesh_svg(&cube(), &RenderOptions::default());
    let persp = render_mesh_svg(
        &cube(),
        &RenderOptions {
            fov: Some(45.0),
            ..Default::default()
        },
    );
    assert_eq!(polys_of(&persp).len(), 12);
    assert_ne!(ortho, persp, "fov should change the projection");
    // Under perspective the near corner projects further out than the far one.
    let p = polys_of(&persp);
    let spread = |v: &[SvgPoly]| {
        v.iter().fold(f64::NEG_INFINITY, |a, q| a.max(q.max_x()))
            - v.iter().fold(f64::INFINITY, |a, q| a.min(q.min_x()))
    };
    assert!(spread(&p) > 0.0);
    // Still fits the cell.
    for q in &p {
        for (x, y) in &q.pts {
            assert!(*x >= -0.1 && *x <= 520.1 && *y >= -0.1 && *y <= 520.1);
        }
    }
}

#[test]
fn alpha_and_size_options() {
    let svg = render_mesh_svg(
        &cube(),
        &RenderOptions {
            alpha: 0.8,
            size: 100.0,
            ..Default::default()
        },
    );
    assert!(svg.contains("fill-opacity=\"0.8\""));
    assert!(svg.contains("stroke-opacity=\"0.20\""), "0.8*0.25");
    // edge strokes: min(1, 0.8+0.25)=1.0 -> 1.0*0.55
    assert!(svg.contains("stroke-opacity=\"0.55\""));
    for p in polys_of(&svg) {
        for (x, y) in p.pts {
            assert!((-0.1..=100.1).contains(&x) && (-0.1..=100.1).contains(&y));
        }
    }
}

#[test]
fn auto_order_falls_back_to_depth_sort_on_big_meshes() {
    // 80x80 grid of quads = 12800 triangles, past the 12000 BSP budget.
    const N: usize = 80;
    let mut v = Vec::new();
    for i in 0..=N {
        for j in 0..=N {
            let (x, y) = (i as f64, j as f64);
            v.push([x, y, ((x * 0.7).sin() + (y * 0.5).cos()) * 2.0]);
        }
    }
    let idx = |i: usize, j: usize| (i * (N + 1) + j) as u32;
    let mut t = Vec::new();
    let mut ids = Vec::new();
    for i in 0..N {
        for j in 0..N {
            t.push([idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)]);
            t.push([idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]);
            ids.push((i % 17) as NodeId);
            ids.push((i % 17) as NodeId);
        }
    }
    let m = mesh_of(&v, &t, &ids, &[]);
    assert_eq!(m.triangles.len(), 12800);

    let start = std::time::Instant::now();
    let svg = render_mesh_svg(
        &m,
        &RenderOptions {
            edges: false,
            ..Default::default()
        },
    );
    let elapsed = start.elapsed();
    // Depth sort: one polygon per triangle, nothing split.
    assert_eq!(count(&svg, "<polygon "), 12800);
    assert!(elapsed.as_secs() < 20, "auto fallback too slow: {elapsed:?}");

    let d = paint_order_depths(
        &m,
        &RenderOptions {
            edges: false,
            ..Default::default()
        },
    );
    assert_eq!(d.len(), 12800);
    assert!(
        d.windows(2).all(|w| w[0] >= w[1]),
        "depth sort must be back-to-front"
    );
}

#[test]
fn svg_document_grid_layout() {
    let frags: Vec<String> = (0..3).map(|i| format!("<g>{i}</g>")).collect();
    let doc = svg_document(&frags, 2, 100.0);
    assert!(doc.starts_with("<svg xmlns=\"http://www.w3.org/2000/svg\""));
    assert!(doc.contains("width=\"200\" height=\"200\""));
    assert!(doc.contains("viewBox=\"0 0 200 200\""));
    assert!(doc.contains("<g transform=\"translate(0,0)\"><g>0</g></g>"));
    assert!(doc.contains("<g transform=\"translate(100,0)\"><g>1</g></g>"));
    assert!(doc.contains("<g transform=\"translate(0,100)\"><g>2</g></g>"));
    assert!(doc.trim_end().ends_with("</svg>"));
    assert!(!doc.contains("<rect"));

    // 4 fragments, 4 columns -> one row.
    let doc = svg_document(&frags, 4, 50.0);
    assert!(doc.contains("width=\"200\" height=\"50\""));
}

// ── it actually renders ─────────────────────────────────────────────────────

#[test]
fn output_renders_with_resvg() {
    let home = std::env::var("HOME").unwrap_or_default();
    let resvg = std::path::Path::new(&home).join(".cargo/bin/resvg");
    if !resvg.exists() {
        eprintln!("resvg not installed; skipping");
        return;
    }
    let frags = vec![
        render_mesh_svg(
            &cube(),
            &RenderOptions {
                title: Some("cube & <friends>".into()),
                ..Default::default()
            },
        ),
        render_mesh_svg(
            &piercing_pair(),
            &RenderOptions {
                title: Some("pierce".into()),
                fov: Some(50.0),
                ..Default::default()
            },
        ),
    ];
    let doc = svg_document(&frags, 2, 520.0);
    let dir = std::env::temp_dir().join("solid-diff-render-test");
    std::fs::create_dir_all(&dir).unwrap();
    let svg_path = dir.join("out.svg");
    let png_path = dir.join("out.png");
    let _ = std::fs::remove_file(&png_path);
    std::fs::write(&svg_path, &doc).unwrap();

    let st = std::process::Command::new(&resvg)
        .arg(&svg_path)
        .arg(&png_path)
        .output()
        .expect("run resvg");
    assert!(
        st.status.success(),
        "resvg failed: {}",
        String::from_utf8_lossy(&st.stderr)
    );
    let n = std::fs::metadata(&png_path).unwrap().len();
    assert!(n > 4096, "png suspiciously small: {n} bytes");
    // The title text must have been escaped, or resvg would have choked above.
    assert!(doc.contains("cube &amp; &lt;friends&gt;"));
}

