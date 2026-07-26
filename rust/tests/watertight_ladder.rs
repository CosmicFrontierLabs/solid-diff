//! Hand-built B-reps that must tessellate watertight.
//!
//! A B-rep states exactly which faces share which edge. If each edge is
//! sampled once and both its faces build their boundary from those same
//! points, a closed solid can only come out closed -- watertightness is a
//! property of the construction, not something to measure afterwards and hope
//! for. Nothing here should ever need mesh repair.
//!
//! The corpus cannot show this, because most of its "holes" are the genuine
//! free boundary of sheet bodies and the rest are tangled up with surface
//! evaluation. These cases are small enough to be unambiguous: every one is a
//! closed solid whose exact shape is known, so any open edge, reversed
//! triangle or non-manifold junction is a bug in the tessellator and nothing
//! else.
//!
//! They are ordered by what they add: flat faces, then a periodic direction,
//! then a pole, then two periodic directions at once.

use solid_diff::mesh::Mesh;
use solid_diff::tess::tessellate;
use solid_diff::value::{Node, NodeId, Value};
use solid_diff::Graph;

// ── a small B-rep builder ───────────────────────────────────────────────────

#[derive(Default)]
struct Brep {
    nodes: Vec<Node>,
    next: NodeId,
}

impl Brep {
    fn new() -> Self {
        Brep {
            nodes: Vec::new(),
            next: 1,
        }
    }

    fn add(&mut self, name: &str, fields: Vec<(&str, Value)>) -> NodeId {
        let id = self.next;
        self.next += 1;
        self.nodes.push(Node {
            node_type: 0,
            name: name.to_string(),
            id,
            count: None,
            fields: fields
                .into_iter()
                .map(|(k, v)| (k.to_string(), v))
                .collect(),
        });
        id
    }

    fn set(&mut self, id: NodeId, key: &str, v: Value) {
        let n = self.nodes.iter_mut().find(|n| n.id == id).expect("node");
        if let Some(slot) = n.fields.iter_mut().find(|(k, _)| k == key) {
            slot.1 = v;
        } else {
            n.fields.push((key.to_string(), v));
        }
    }

    fn vertex(&mut self, p: [f64; 3]) -> NodeId {
        let pt = self.add("POINT", vec![("pvec", v3(p))]);
        self.add("VERTEX", vec![("point", ptr(pt))])
    }

    fn line_through(&mut self, a: [f64; 3], b: [f64; 3]) -> NodeId {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        self.add("LINE", vec![("pvec", v3(a)), ("direction", v3(d))])
    }

    fn plane(&mut self, o: [f64; 3], n: [f64; 3], x: [f64; 3]) -> NodeId {
        self.add(
            "PLANE",
            vec![("pvec", v3(o)), ("normal", v3(n)), ("x_axis", v3(x))],
        )
    }

    /// An edge with its two halfedges, one per adjoining face.
    ///
    /// Returns `(positive, negative)` halfedge ids. The '+' one runs from
    /// `va` to `vb`; its mate runs the other way. Both point at the same
    /// EDGE, which is what makes the two faces share one curve.
    fn edge(&mut self, curve: NodeId, va: NodeId, vb: NodeId) -> (NodeId, NodeId) {
        let e = self.add("EDGE", vec![("curve", ptr(curve))]);
        let hp = self.add(
            "HALFEDGE",
            vec![
                ("edge", ptr(e)),
                ("vertex", ptr(va)),
                ("sense", Value::Char("+".into())),
            ],
        );
        let hm = self.add(
            "HALFEDGE",
            vec![
                ("edge", ptr(e)),
                ("vertex", ptr(vb)),
                ("sense", Value::Char("-".into())),
            ],
        );
        self.set(hp, "other", ptr(hm));
        self.set(hm, "other", ptr(hp));
        self.set(e, "halfedge", ptr(hp));
        (hp, hm)
    }

    /// A face carrying one loop, built from halfedges in traversal order.
    ///
    /// Loops chain through `backward`, so that is the link wired here; the
    /// `forward` link is filled in as its inverse for completeness.
    fn face(&mut self, surface: NodeId, halfedges: &[NodeId]) -> NodeId {
        let f = self.add(
            "FACE",
            vec![
                ("surface", ptr(surface)),
                ("sense", Value::Char("+".into())),
            ],
        );
        let lp = self.add(
            "LOOP",
            vec![("face", ptr(f)), ("halfedge", ptr(halfedges[0]))],
        );
        self.set(f, "loop", ptr(lp));
        let n = halfedges.len();
        for (i, h) in halfedges.iter().enumerate() {
            self.set(*h, "loop", ptr(lp));
            self.set(*h, "backward", ptr(halfedges[(i + 1) % n]));
            self.set(*h, "forward", ptr(halfedges[(i + n - 1) % n]));
        }
        f
    }

    fn graph(self) -> Graph {
        Graph::new(self.nodes)
    }
}

fn v3(p: [f64; 3]) -> Value {
    Value::Vec3(p)
}

fn ptr(id: NodeId) -> Value {
    Value::Ptr(Some(id))
}

/// Mesh a hand-built body and report every way it fails to be a closed solid.
#[track_caller]
fn assert_watertight(g: &Graph, what: &str, expect_volume: f64) -> Mesh {
    let mesh = tessellate(g, None);
    assert!(
        !mesh.is_empty(),
        "{what}: produced no triangles at all. warnings: {:?}",
        mesh.warnings
    );
    let r = mesh.manifold_report();
    assert!(
        r.is_watertight(),
        "{what} is a closed solid, so its mesh must be closed: {r}"
    );
    let v = mesh.signed_volume();
    let rel = (v - expect_volume).abs() / expect_volume.abs().max(1e-30);
    assert!(
        rel < 0.02,
        "{what}: volume {v:.6} is {:.1}% off the exact {expect_volume:.6}",
        rel * 100.0
    );
    mesh
}

// ── rung 1: six flat faces ──────────────────────────────────────────────────

/// An axis-aligned box. No curvature, no periodicity, no poles -- if this is
/// not watertight nothing else can be.
fn unit_box(sx: f64, sy: f64, sz: f64) -> Graph {
    let mut b = Brep::new();
    // Corner numbering: bit 0 = +x, bit 1 = +y, bit 2 = +z.
    let corner = |i: usize| {
        [
            if i & 1 != 0 { sx } else { 0.0 },
            if i & 2 != 0 { sy } else { 0.0 },
            if i & 4 != 0 { sz } else { 0.0 },
        ]
    };
    let vs: Vec<NodeId> = (0..8).map(|i| b.vertex(corner(i))).collect();

    // The twelve edges, keyed by their endpoint pair.
    let pairs = [
        (0, 1),
        (2, 3),
        (4, 5),
        (6, 7), // along x
        (0, 2),
        (1, 3),
        (4, 6),
        (5, 7), // along y
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7), // along z
    ];
    let mut he: std::collections::HashMap<(usize, usize), NodeId> =
        std::collections::HashMap::new();
    for (a, z) in pairs {
        let c = b.line_through(corner(a), corner(z));
        let (hp, hm) = b.edge(c, vs[a], vs[z]);
        he.insert((a, z), hp);
        he.insert((z, a), hm);
    }

    // Each face lists its corners so that the loop walks the boundary; the
    // outward normal is given explicitly, so winding order here only has to be
    // a consistent circuit.
    let faces: [([usize; 4], [f64; 3], [f64; 3]); 6] = [
        ([0, 2, 6, 4], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), // x = 0
        ([1, 5, 7, 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),  // x = sx
        ([0, 4, 5, 1], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]), // y = 0
        ([2, 3, 7, 6], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),  // y = sy
        ([0, 1, 3, 2], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]), // z = 0
        ([4, 6, 7, 5], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),  // z = sz
    ];
    for (ring, n, x) in faces {
        let o = corner(ring[0]);
        let s = b.plane(o, n, x);
        let hs: Vec<NodeId> = (0..4).map(|i| he[&(ring[i], ring[(i + 1) % 4])]).collect();
        b.face(s, &hs);
    }
    b.graph()
}

#[test]
fn rung1_box_is_watertight() {
    let g = unit_box(2.0, 3.0, 5.0);
    let mesh = assert_watertight(&g, "a 2x3x5 box", 30.0);
    // Flat faces need no refinement: two triangles each is the exact answer.
    assert!(
        mesh.triangles.len() >= 12,
        "a box needs at least 12 triangles, got {}",
        mesh.triangles.len()
    );
}

#[test]
fn rung1_box_surface_area_is_exact() {
    let g = unit_box(2.0, 3.0, 5.0);
    let mesh = tessellate(&g, None);
    let exact = 2.0 * (2.0 * 3.0 + 3.0 * 5.0 + 2.0 * 5.0);
    let got = mesh.surface_area();
    assert!(
        (got - exact).abs() / exact < 1e-9,
        "flat faces should be exact: {got} vs {exact}"
    );
}

// ── rung 2: one periodic direction ──────────────────────────────────────────

/// A solid cylinder: two discs and a side wall that wraps in u.
///
/// This is the first case where a face covers a full parameter period, which
/// is where the trimmer has to reason about periodicity rather than a plain
/// bounded polygon. Parasolid stores no seam edge for it -- the wall's loops
/// are just the two rim circles.
fn solid_cylinder(r: f64, h: f64) -> Graph {
    let mut b = Brep::new();
    let axis = [0.0, 0.0, 1.0];
    let xax = [1.0, 0.0, 0.0];

    // One vertex per rim, sitting on the +x side.
    let v_bot = b.vertex([r, 0.0, 0.0]);
    let v_top = b.vertex([r, 0.0, h]);

    let c_bot = b.add(
        "CIRCLE",
        vec![
            ("centre", v3([0.0, 0.0, 0.0])),
            ("normal", v3(axis)),
            ("x_axis", v3(xax)),
            ("radius", Value::F64(r)),
        ],
    );
    let c_top = b.add(
        "CIRCLE",
        vec![
            ("centre", v3([0.0, 0.0, h])),
            ("normal", v3(axis)),
            ("x_axis", v3(xax)),
            ("radius", Value::F64(r)),
        ],
    );
    // A rim is one closed edge: both halfedges start and end at the same
    // vertex, which is how a circular edge with no corner is expressed.
    let (hb_disc, hb_wall) = b.edge(c_bot, v_bot, v_bot);
    let (ht_wall, ht_disc) = b.edge(c_top, v_top, v_top);

    let s_bot = b.plane([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], xax);
    let s_top = b.plane([0.0, 0.0, h], [0.0, 0.0, 1.0], xax);
    let s_wall = b.add(
        "CYLINDER",
        vec![
            ("pvec", v3([0.0, 0.0, 0.0])),
            ("axis", v3(axis)),
            ("x_axis", v3(xax)),
            ("radius", Value::F64(r)),
        ],
    );

    b.face(s_bot, &[hb_disc]);
    b.face(s_top, &[ht_disc]);
    // The wall is bounded by both rims: two separate loops on one face.
    let f = b.add(
        "FACE",
        vec![("surface", ptr(s_wall)), ("sense", Value::Char("+".into()))],
    );
    let l1 = b.add("LOOP", vec![("face", ptr(f)), ("halfedge", ptr(hb_wall))]);
    let l2 = b.add("LOOP", vec![("face", ptr(f)), ("halfedge", ptr(ht_wall))]);
    b.set(f, "loop", ptr(l1));
    b.set(l1, "next", ptr(l2));
    for (h_id, l) in [(hb_wall, l1), (ht_wall, l2)] {
        b.set(h_id, "loop", ptr(l));
        b.set(h_id, "backward", ptr(h_id));
        b.set(h_id, "forward", ptr(h_id));
    }
    b.graph()
}

/// Currently fails: the wall is triangulated over `[0, u_max]` and the wedge
/// from `u_max` back round to the seam is never filled, leaving a slit. Every
/// open edge sits exactly at u = 0. Nothing is duplicated and nothing is
/// mis-welded -- the triangles simply are not there. See #39.
#[test]
#[ignore = "known failure: periodic faces do not close across the seam (#39)"]
fn rung2_cylinder_is_watertight() {
    let (r, h) = (1.0, 4.0);
    let g = solid_cylinder(r, h);
    assert_watertight(&g, "a solid cylinder", std::f64::consts::PI * r * r * h);
}

// ── rung 3: a pole ──────────────────────────────────────────────────────────

/// A sphere cut by a plane: a spherical cap on a disc.
///
/// Adds the case where the parameter domain runs into a pole, so one edge of
/// the UV rectangle collapses to a single point in space.
fn spherical_cap(r: f64, z_cut: f64) -> Graph {
    let mut b = Brep::new();
    let axis = [0.0, 0.0, 1.0];
    let xax = [1.0, 0.0, 0.0];
    let rc = (r * r - z_cut * z_cut).sqrt(); // radius of the cut circle

    let v = b.vertex([rc, 0.0, z_cut]);
    let circ = b.add(
        "CIRCLE",
        vec![
            ("centre", v3([0.0, 0.0, z_cut])),
            ("normal", v3(axis)),
            ("x_axis", v3(xax)),
            ("radius", Value::F64(rc)),
        ],
    );
    let (h_cap, h_disc) = b.edge(circ, v, v);

    let s_sphere = b.add(
        "SPHERE",
        vec![
            ("centre", v3([0.0, 0.0, 0.0])),
            ("axis", v3(axis)),
            ("x_axis", v3(xax)),
            ("radius", Value::F64(r)),
        ],
    );
    let s_disc = b.plane([0.0, 0.0, z_cut], [0.0, 0.0, -1.0], xax);

    b.face(s_sphere, &[h_cap]);
    b.face(s_disc, &[h_disc]);
    b.graph()
}

/// Currently fails with 3 open edges, all at the seam -- the same defect as
/// rung 2, on a surface that also has a pole. See #39.
#[test]
#[ignore = "known failure: periodic faces do not close across the seam (#39)"]
fn rung3_spherical_cap_is_watertight() {
    let (r, z) = (1.0, 0.5);
    let h = r - z; // cap height
    let vol = std::f64::consts::PI * h * h * (r - h / 3.0);
    assert_watertight(&spherical_cap(r, z), "a spherical cap", vol);
}

// ── rung 4: two periodic directions ─────────────────────────────────────────

/// A whole torus: one face, no edges at all, periodic in both directions.
///
/// The hardest trimming case in the format -- there is no boundary to anchor
/// against and no open direction to escape through.
fn whole_torus(major: f64, minor: f64) -> Graph {
    let mut b = Brep::new();
    let s = b.add(
        "TORUS",
        vec![
            ("centre", v3([0.0, 0.0, 0.0])),
            ("axis", v3([0.0, 0.0, 1.0])),
            ("x_axis", v3([1.0, 0.0, 0.0])),
            ("major_radius", Value::F64(major)),
            ("minor_radius", Value::F64(minor)),
        ],
    );
    let f = b.add(
        "FACE",
        vec![("surface", ptr(s)), ("sense", Value::Char("+".into()))],
    );
    let _ = f;
    b.graph()
}

/// Passes, which is the surprise: a torus is periodic in both directions and
/// still closes. The difference from rungs 2 and 3 is that it carries no
/// loops at all, so nothing constrains the boundary and the grid covers the
/// whole domain. The seam defect is specific to a face that both wraps *and*
/// has boundary loops.
#[test]
fn rung4_torus_is_watertight() {
    let (rmaj, rmin) = (3.0, 1.0);
    let vol = 2.0 * std::f64::consts::PI * std::f64::consts::PI * rmaj * rmin * rmin;
    assert_watertight(&whole_torus(rmaj, rmin), "a whole torus", vol);
}
