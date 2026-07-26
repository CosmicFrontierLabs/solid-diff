//! Direct tests for the XT curve/surface evaluators.
//!
//! Nodes are constructed by hand so these do not depend on the XT parser.

use solid_diff::geom::{make_curve, make_surface, Curve, Surface, P2, P3, TWO_PI};
use solid_diff::graph::Graph;
use solid_diff::value::{Node, NodeId, Value};

// ── helpers ─────────────────────────────────────────────────────────────────

fn node(id: NodeId, name: &str, fields: Vec<(&str, Value)>) -> Node {
    Node {
        node_type: 0,
        name: name.to_string(),
        id,
        count: None,
        fields: fields
            .into_iter()
            .map(|(k, v)| (k.to_string(), v))
            .collect(),
    }
}

fn v3(x: f64, y: f64, z: f64) -> Value {
    Value::Vec3([x, y, z])
}

fn f(x: f64) -> Value {
    Value::F64(x)
}

fn i(x: i64) -> Value {
    Value::I32(x as i32)
}

fn b(x: bool) -> Value {
    Value::Bool(x)
}

fn arr_f(xs: &[f64]) -> Value {
    Value::Array(xs.iter().map(|&x| Value::F64(x)).collect())
}

fn arr_i(xs: &[i64]) -> Value {
    Value::Array(xs.iter().map(|&x| Value::I32(x as i32)).collect())
}

fn ptr(id: NodeId) -> Value {
    Value::Ptr(Some(id))
}

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: P3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn dist(a: P3, b: P3) -> f64 {
    norm(sub(a, b))
}

fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[track_caller]
fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} != {b} (tol {tol})");
}

#[track_caller]
fn close3(a: P3, b: P3, tol: f64) {
    assert!(dist(a, b) <= tol, "{a:?} != {b:?} (tol {tol})");
}

/// Angular difference mod `p`.
fn wrap_diff(a: f64, b: f64, p: f64) -> f64 {
    let mut d = (a - b) % p;
    if d > p / 2.0 {
        d -= p;
    }
    if d < -p / 2.0 {
        d += p;
    }
    d.abs()
}

/// Central-difference normal, to check analytic overrides.
fn fd_normal(s: &dyn Surface, uv: P2) -> P3 {
    let h = 1e-6;
    let du = sub(s.eval([uv[0] + h, uv[1]]), s.eval([uv[0] - h, uv[1]]));
    let dv = sub(s.eval([uv[0], uv[1] + h]), s.eval([uv[0], uv[1] - h]));
    let n = [
        du[1] * dv[2] - du[2] * dv[1],
        du[2] * dv[0] - du[0] * dv[2],
        du[0] * dv[1] - du[1] * dv[0],
    ];
    let l = norm(n);
    [n[0] / l, n[1] / l, n[2] / l]
}

fn curve_of(nodes: Vec<Node>, id: NodeId) -> (Graph, Box<dyn Curve>) {
    let g = Graph::new(nodes);
    let c = make_curve(&g, g.get(id).unwrap()).expect("curve built");
    (g, c)
}

fn surface_of(nodes: Vec<Node>, id: NodeId) -> (Graph, Box<dyn Surface>) {
    let g = Graph::new(nodes);
    let s = make_surface(&g, g.get(id).unwrap()).expect("surface built");
    (g, s)
}

// ── curves ──────────────────────────────────────────────────────────────────

#[test]
fn line_eval_inv() {
    let n = node(
        1,
        "LINE",
        vec![
            ("pvec", v3(1.0, 2.0, 3.0)),
            ("direction", v3(0.0, 0.0, 2.0)),
        ],
    );
    let (_g, c) = curve_of(vec![n], 1);
    close3(c.eval(0.0), [1.0, 2.0, 3.0], 1e-12);
    // direction is normalized, so t is arc length
    close3(c.eval(2.0), [1.0, 2.0, 5.0], 1e-12);
    close(c.inv([1.0, 2.0, 8.0]), 5.0, 1e-12);
    assert!(c.period().is_none());
    assert!(c.full_range().is_none());
}

#[test]
fn circle_geometry_and_roundtrip() {
    let n = node(
        1,
        "CIRCLE",
        vec![
            ("centre", v3(1.0, 0.0, -2.0)),
            ("normal", v3(0.0, 0.0, 3.0)),
            ("x_axis", v3(1.0, 0.0, 0.0)),
            ("radius", f(2.5)),
        ],
    );
    let (_g, c) = curve_of(vec![n], 1);
    assert_eq!(c.period(), Some(TWO_PI));
    assert_eq!(c.full_range(), Some((0.0, TWO_PI)));
    for k in 0..17 {
        let t = k as f64 * 0.37;
        let p = c.eval(t);
        close(dist(p, [1.0, 0.0, -2.0]), 2.5, 1e-12);
        close(p[2], -2.0, 1e-12);
        close(wrap_diff(c.inv(p), t, TWO_PI), 0.0, 1e-9);
    }
    // right-handed: y = normal x x_axis
    close3(c.eval(std::f64::consts::FRAC_PI_2), [1.0, 2.5, -2.0], 1e-12);
}

#[test]
fn ellipse_geometry_and_roundtrip() {
    let n = node(
        1,
        "ELLIPSE",
        vec![
            ("centre", v3(0.0, 0.0, 0.0)),
            ("normal", v3(0.0, 0.0, 1.0)),
            ("x_axis", v3(1.0, 0.0, 0.0)),
            ("major_radius", f(4.0)),
            ("minor_radius", f(2.0)),
        ],
    );
    let (_g, c) = curve_of(vec![n], 1);
    for k in 0..13 {
        let t = k as f64 * 0.5;
        let p = c.eval(t);
        // on the ellipse: (x/a)^2 + (y/b)^2 == 1
        close((p[0] / 4.0).powi(2) + (p[1] / 2.0).powi(2), 1.0, 1e-12);
        close(wrap_diff(c.inv(p), t, TWO_PI), 0.0, 1e-9);
    }
}

/// B_CURVE with collinear control points must evaluate on that line.
#[test]
fn bcurve_collinear() {
    // degree 3, 5 control points on the line p = (1,1,1) + s*(1,2,3)/norm
    let dir = [1.0, 2.0, 3.0];
    let mut verts = Vec::new();
    for k in 0..5 {
        let s = k as f64 * 0.5;
        verts.extend_from_slice(&[1.0 + s * dir[0], 1.0 + s * dir[1], 1.0 + s * dir[2]]);
    }
    let nodes = vec![
        node(1, "B_CURVE", vec![("nurbs", ptr(2))]),
        node(
            2,
            "NURBS_CURVE",
            vec![
                ("degree", i(3)),
                ("n_vertices", i(5)),
                ("vertex_dim", i(3)),
                ("rational", b(false)),
                ("closed", b(false)),
                ("periodic", b(false)),
                ("bspline_vertices", ptr(3)),
                ("knots", ptr(4)),
                ("knot_mult", ptr(5)),
            ],
        ),
        node(3, "BSPLINE_VERTICES", vec![("vertices", arr_f(&verts))]),
        node(4, "KNOT_SET", vec![("knots", arr_f(&[0.0, 0.5, 1.0]))]),
        node(5, "KNOT_MULT", vec![("mult", arr_i(&[4, 1, 4]))]),
    ];
    let (_g, c) = curve_of(nodes, 1);
    assert_eq!(c.full_range(), Some((0.0, 1.0)));
    // endpoints interpolate the first/last control point (clamped knots)
    close3(c.eval(0.0), [1.0, 1.0, 1.0], 1e-12);
    close3(c.eval(1.0), [3.0, 5.0, 7.0], 1e-9);
    for k in 0..11 {
        let t = k as f64 / 10.0;
        let p = c.eval(t);
        // distance from the line through (1,1,1) with direction dir
        let q = sub(p, [1.0, 1.0, 1.0]);
        let dl = norm(dir);
        let along = dot(q, [dir[0] / dl, dir[1] / dl, dir[2] / dl]);
        let perp = norm(sub(
            q,
            [
                dir[0] / dl * along,
                dir[1] / dl * along,
                dir[2] / dl * along,
            ],
        ));
        close(perp, 0.0, 1e-12);
        // inversion recovers the parameter
        close(c.inv(p), t, 1e-6);
    }
}

/// Rational B_CURVE: the classic 90-degree arc, verts stored as (wx,wy,wz,w).
#[test]
fn bcurve_rational_arc() {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let verts = vec![
        1.0, 0.0, 0.0, 1.0, // (1,0,0) w=1
        w, w, 0.0, w, // (1,1,0) w=1/sqrt2
        0.0, 1.0, 0.0, 1.0, // (0,1,0) w=1
    ];
    let nodes = vec![
        node(1, "B_CURVE", vec![("nurbs", ptr(2))]),
        node(
            2,
            "NURBS_CURVE",
            vec![
                ("degree", i(2)),
                ("n_vertices", i(3)),
                ("vertex_dim", i(4)),
                ("rational", b(true)),
                ("closed", b(false)),
                ("bspline_vertices", ptr(3)),
                ("knots", ptr(4)),
                ("knot_mult", ptr(5)),
            ],
        ),
        node(3, "BSPLINE_VERTICES", vec![("vertices", arr_f(&verts))]),
        node(4, "KNOT_SET", vec![("knots", arr_f(&[0.0, 1.0]))]),
        node(5, "KNOT_MULT", vec![("mult", arr_i(&[3, 3]))]),
    ];
    let (_g, c) = curve_of(nodes, 1);
    for k in 0..=10 {
        let t = k as f64 / 10.0;
        let p = c.eval(t);
        close(norm(p), 1.0, 1e-12); // exact unit circle
        close(p[2], 0.0, 1e-15);
    }
    close3(c.eval(0.5), [w, w, 0.0], 1e-12);
}

#[test]
fn trimmed_curve_range_and_delegation() {
    let nodes = vec![
        node(
            1,
            "TRIMMED_CURVE",
            vec![
                ("basis_curve", ptr(2)),
                ("parm_1", f(0.5)),
                ("parm_2", f(2.0)),
                ("point_1", v3(0.5, 0.0, 0.0)),
                ("point_2", v3(2.0, 0.0, 0.0)),
            ],
        ),
        node(
            2,
            "LINE",
            vec![
                ("pvec", v3(0.0, 0.0, 0.0)),
                ("direction", v3(1.0, 0.0, 0.0)),
            ],
        ),
    ];
    let (_g, c) = curve_of(nodes, 1);
    assert_eq!(c.full_range(), Some((0.5, 2.0)));
    close3(c.eval(1.25), [1.25, 0.0, 0.0], 1e-12);
    close(c.inv([1.75, 0.0, 0.0]), 1.75, 1e-12);
}

#[test]
fn intersection_polyline() {
    let hvec = Value::Array(vec![
        v3(0.0, 0.0, 0.0),
        v3(1.0, 0.0, 0.0),
        v3(1.0, 2.0, 0.0),
        v3(1.0, 2.0, 2.0),
    ]);
    let nodes = vec![
        node(1, "INTERSECTION", vec![("chart", ptr(2))]),
        node(2, "CHART", vec![("hvec", hvec)]),
    ];
    let (_g, c) = curve_of(nodes, 1);
    // arc-length parameterization: total length 1 + 2 + 2 = 5
    assert_eq!(c.full_range(), Some((0.0, 5.0)));
    close3(c.eval(0.0), [0.0, 0.0, 0.0], 1e-12);
    close3(c.eval(0.5), [0.5, 0.0, 0.0], 1e-12);
    close3(c.eval(2.0), [1.0, 1.0, 0.0], 1e-12);
    close3(c.eval(5.0), [1.0, 2.0, 2.0], 1e-12);
    close3(c.eval(9.0), [1.0, 2.0, 2.0], 1e-12); // clamped

    // inv projects onto the nearest segment, not the nearest stored sample.
    // A point 0.4 along the second leg must come back as s = 1 + 0.4*2 = 1.8;
    // snapping to samples would have returned 1.0 or 3.0.
    close(c.inv([1.05, 0.4, 0.0]), 1.4, 1e-12);
    close3(c.eval(c.inv([1.0, 1.5, 0.0])), [1.0, 1.5, 0.0], 1e-12);
}

#[test]
fn unsupported_and_broken_nodes_return_none() {
    let nodes = vec![
        node(1, "PE_CURVE", vec![("pvec", v3(0.0, 0.0, 0.0))]),
        node(2, "LINE", vec![("pvec", v3(0.0, 0.0, 0.0))]), // missing direction
        node(3, "B_CURVE", vec![("nurbs", Value::Ptr(None))]),
        node(4, "CIRCLE", vec![("centre", v3(0.0, 0.0, 0.0))]),
        node(5, "SPHERE", vec![("radius", f(1.0))]),
        node(6, "PLANE", vec![("pvec", v3(0.0, 0.0, 0.0))]),
        node(7, "FOO_SURF", vec![]),
        node(
            8,
            "B_SURFACE",
            vec![("nurbs", ptr(99))], // dangling
        ),
    ];
    let g = Graph::new(nodes);
    for id in [1, 2, 3, 4] {
        assert!(make_curve(&g, g.get(id).unwrap()).is_none(), "curve {id}");
    }
    for id in [5, 6, 7, 8] {
        assert!(
            make_surface(&g, g.get(id).unwrap()).is_none(),
            "surface {id}"
        );
    }
}

// ── surfaces ────────────────────────────────────────────────────────────────

fn plane_node(id: NodeId, p: [f64; 3], n: [f64; 3], x: [f64; 3]) -> Node {
    node(
        id,
        "PLANE",
        vec![
            ("pvec", v3(p[0], p[1], p[2])),
            ("normal", v3(n[0], n[1], n[2])),
            ("x_axis", v3(x[0], x[1], x[2])),
        ],
    )
}

#[test]
fn plane_normal_and_roundtrip() {
    let (_g, s) = surface_of(
        vec![plane_node(
            1,
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 5.0],
            [2.0, 0.0, 0.0],
        )],
        1,
    );
    close3(s.normal([0.0, 0.0]), [0.0, 0.0, 1.0], 1e-15);
    close3(s.normal([3.0, -7.0]), [0.0, 0.0, 1.0], 1e-15);
    close3(fd_normal(s.as_ref(), [1.0, 1.0]), [0.0, 0.0, 1.0], 1e-9);
    for uv in [[0.0, 0.0], [1.5, -2.5], [10.0, 4.0]] {
        let p = s.eval(uv);
        close(p[2], 3.0, 1e-12);
        let back = s.inv(p);
        close(back[0], uv[0], 1e-12);
        close(back[1], uv[1], 1e-12);
    }
    let uv0 = s.inv([1.0, 2.0, 3.0]);
    close(uv0[0], 0.0, 1e-12);
    close(uv0[1], 0.0, 1e-12);
    assert_eq!(s.sense_sign(), 1.0);
    assert!(s.period_u().is_none() && s.period_v().is_none());
}

#[test]
fn plane_sense_flag() {
    let mut n = plane_node(1, [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    n.fields.push(("sense".into(), Value::Char("-".into())));
    let (_g, s) = surface_of(vec![n], 1);
    assert_eq!(s.sense_sign(), -1.0);
}

fn cylinder_node(id: NodeId, r: f64) -> Node {
    node(
        id,
        "CYLINDER",
        vec![
            ("pvec", v3(1.0, -1.0, 0.5)),
            ("axis", v3(0.0, 0.0, 2.0)),
            ("radius", f(r)),
            ("x_axis", v3(1.0, 0.0, 0.0)),
        ],
    )
}

#[test]
fn cylinder_geometry_roundtrip_normal() {
    let (_g, s) = surface_of(vec![cylinder_node(1, 3.0)], 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    assert!(s.period_v().is_none());
    for k in 0..12 {
        let uv = [k as f64 * 0.5, k as f64 * 0.25 - 1.0];
        let p = s.eval(uv);
        // distance from the axis line through pvec is exactly r
        let q = sub(p, [1.0, -1.0, 0.5]);
        let along = q[2];
        close(norm([q[0], q[1], 0.0]), 3.0, 1e-12);
        close(along, uv[1], 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-12);
        close(back[1], uv[1], 1e-12);
        // analytic normal agrees with central differences and is radial
        close3(s.normal(uv), fd_normal(s.as_ref(), uv), 1e-8);
        close3(s.normal(uv), [q[0] / 3.0, q[1] / 3.0, 0.0], 1e-12);
    }
}

#[test]
fn cone_apex_bounds_and_radius() {
    let half = 0.3_f64; // half angle
    let n = node(
        1,
        "CONE",
        vec![
            ("pvec", v3(0.0, 0.0, 0.0)),
            ("axis", v3(0.0, 0.0, 1.0)),
            ("radius", f(2.0)),
            ("sin_half_angle", f(half.sin())),
            ("cos_half_angle", f(half.cos())),
            ("x_axis", v3(1.0, 0.0, 0.0)),
        ],
    );
    let (_g, s) = surface_of(vec![n], 1);
    let tan = half.tan();
    let (lo, hi) = s.v_bounds().unwrap();
    close(lo.unwrap(), -2.0 / tan, 1e-9);
    assert!(hi.is_none());
    for k in 0..10 {
        let uv = [k as f64 * 0.7, k as f64 * 0.3 - 1.0];
        let p = s.eval(uv);
        close(norm([p[0], p[1], 0.0]), 2.0 + uv[1] * tan, 1e-12);
        close(p[2], uv[1], 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-12);
        close(back[1], uv[1], 1e-12);
        close3(s.normal(uv), fd_normal(s.as_ref(), uv), 1e-7);
    }
    // apex: zero radius
    let apex = s.eval([1.0, -2.0 / tan]);
    close(norm([apex[0], apex[1], 0.0]), 0.0, 1e-12);
}

#[test]
fn sphere_geometry_roundtrip_normal() {
    let n = node(
        1,
        "SPHERE",
        vec![
            ("centre", v3(-1.0, 2.0, 0.0)),
            ("radius", f(4.0)),
            ("axis", v3(0.0, 0.0, 1.0)),
            ("x_axis", v3(1.0, 0.0, 0.0)),
        ],
    );
    let (_g, s) = surface_of(vec![n], 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    assert_eq!(
        s.v_bounds(),
        Some((
            Some(-std::f64::consts::FRAC_PI_2),
            Some(std::f64::consts::FRAC_PI_2)
        ))
    );
    for k in 0..15 {
        let uv = [k as f64 * 0.41, (k as f64 * 0.2) - 1.4];
        let p = s.eval(uv);
        close(dist(p, [-1.0, 2.0, 0.0]), 4.0, 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-12);
        close(back[1], uv[1], 1e-12);
        close3(s.normal(uv), fd_normal(s.as_ref(), uv), 1e-7);
        // eval(inv(p)) == p for points known to lie on the sphere
        close3(s.eval(s.inv(p)), p, 1e-12);
    }
}

#[test]
fn torus_tube_radius_and_periods() {
    let n = node(
        1,
        "TORUS",
        vec![
            ("centre", v3(0.0, 0.0, 1.0)),
            ("axis", v3(0.0, 0.0, 1.0)),
            ("major_radius", f(5.0)),
            ("minor_radius", f(1.5)),
            ("x_axis", v3(1.0, 0.0, 0.0)),
        ],
    );
    let (_g, s) = surface_of(vec![n], 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    assert_eq!(s.period_v(), Some(TWO_PI));
    for k in 0..15 {
        let uv = [k as f64 * 0.41, k as f64 * 0.77];
        let p = s.eval(uv);
        // distance to the tube centre circle is the minor radius
        let q = sub(p, [0.0, 0.0, 1.0]);
        let h = q[2];
        let rad = norm([q[0], q[1], 0.0]);
        close(((rad - 5.0).powi(2) + h * h).sqrt(), 1.5, 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-12);
        close(wrap_diff(back[1], uv[1], TWO_PI), 0.0, 1e-12);
        close3(s.normal(uv), fd_normal(s.as_ref(), uv), 1e-7);
    }
}

#[test]
fn swept_surf_is_a_cylinder_when_section_is_a_circle() {
    let nodes = vec![
        node(
            1,
            "SWEPT_SURF",
            vec![("section", ptr(2)), ("sweep", v3(0.0, 0.0, 3.0))],
        ),
        node(
            2,
            "CIRCLE",
            vec![
                ("centre", v3(0.0, 0.0, 0.0)),
                ("normal", v3(0.0, 0.0, 1.0)),
                ("x_axis", v3(1.0, 0.0, 0.0)),
                ("radius", f(2.0)),
            ],
        ),
    ];
    let (_g, s) = surface_of(nodes, 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    for k in 0..12 {
        let uv = [k as f64 * 0.5, k as f64 * 0.3 - 1.0];
        let p = s.eval(uv);
        close(norm([p[0], p[1], 0.0]), 2.0, 1e-12);
        close(p[2], uv[1], 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-6);
        close(back[1], uv[1], 1e-9);
        close3(s.eval(back), p, 1e-6);
    }
}

#[test]
fn spun_surf_line_profile_is_a_cylinder() {
    // profile: a line parallel to the axis at radius 3
    let nodes = vec![
        node(
            1,
            "SPUN_SURF",
            vec![
                ("profile", ptr(2)),
                ("base", v3(0.0, 0.0, 0.0)),
                ("axis", v3(0.0, 0.0, 1.0)),
                ("x_axis", v3(1.0, 0.0, 0.0)),
            ],
        ),
        node(
            2,
            "LINE",
            vec![
                ("pvec", v3(3.0, 0.0, 0.0)),
                ("direction", v3(0.0, 0.0, 1.0)),
            ],
        ),
    ];
    let (_g, s) = surface_of(nodes, 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    for k in 0..10 {
        let uv = [k as f64 * 0.6, k as f64 / 10.0];
        let p = s.eval(uv);
        close(norm([p[0], p[1], 0.0]), 3.0, 1e-12);
        close(p[2], uv[1], 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-9);
        // v is snapped to the 256-sample profile cache (see report)
        close(back[1], uv[1], 5e-3);
        close3(s.eval(back), p, 5e-3);
    }
}

#[test]
fn spun_surf_accepts_both_field_spellings() {
    // Some files spell these `section`/`pvec`; the 13006 schema says
    // `profile`/`base`. Both must work.
    let nodes = vec![
        node(
            1,
            "SPUN_SURF",
            vec![
                ("section", ptr(2)),
                ("pvec", v3(0.0, 0.0, 0.0)),
                ("axis", v3(0.0, 0.0, 1.0)),
            ],
        ),
        node(
            2,
            "LINE",
            vec![
                ("pvec", v3(3.0, 0.0, 0.0)),
                ("direction", v3(0.0, 0.0, 1.0)),
            ],
        ),
    ];
    let (_g, s) = surface_of(nodes, 1);
    let p = s.eval([0.0, 0.5]);
    close3(p, [3.0, 0.0, 0.5], 1e-12);
}

#[test]
fn offset_surf_grows_cylinder_radius() {
    let nodes = vec![
        node(
            1,
            "OFFSET_SURF",
            vec![("surface", ptr(2)), ("offset", f(0.5))],
        ),
        cylinder_node(2, 1.0),
    ];
    let (_g, s) = surface_of(nodes, 1);
    assert_eq!(s.period_u(), Some(TWO_PI));
    for k in 0..8 {
        let uv = [k as f64 * 0.8, k as f64 * 0.5];
        let p = s.eval(uv);
        let q = sub(p, [1.0, -1.0, 0.5]);
        close(norm([q[0], q[1], 0.0]), 1.5, 1e-12);
        let back = s.inv(p);
        close(wrap_diff(back[0], uv[0], TWO_PI), 0.0, 1e-12);
        close(back[1], uv[1], 1e-12);
    }
}

#[test]
fn offset_surf_negative_sense_shrinks() {
    let nodes = vec![
        node(
            1,
            "OFFSET_SURF",
            vec![
                ("surface", ptr(2)),
                ("offset", f(0.5)),
                ("sense", Value::Char("-".into())),
            ],
        ),
        cylinder_node(2, 1.0),
    ];
    let (_g, s) = surface_of(nodes, 1);
    let p = s.eval([0.0, 0.0]);
    let q = sub(p, [1.0, -1.0, 0.5]);
    close(norm([q[0], q[1], 0.0]), 0.5, 1e-12);
    assert_eq!(s.sense_sign(), -1.0);
}

// ── B_SURFACE ───────────────────────────────────────────────────────────────

/// nu x nv control net, stored v-fastest, z = 2x + 3y + 1 (a plane).
fn planar_bsurface_nodes() -> Vec<Node> {
    let (nu, nv) = (3usize, 4usize);
    let mut verts = Vec::new();
    for iu in 0..nu {
        for iv in 0..nv {
            let (x, y) = (iu as f64, iv as f64);
            verts.extend_from_slice(&[x, y, 2.0 * x + 3.0 * y + 1.0]);
        }
    }
    vec![
        node(1, "B_SURFACE", vec![("nurbs", ptr(2))]),
        node(
            2,
            "NURBS_SURF",
            vec![
                ("u_degree", i(2)),
                ("v_degree", i(2)),
                ("n_u_vertices", i(nu as i64)),
                ("n_v_vertices", i(nv as i64)),
                ("vertex_dim", i(3)),
                ("rational", b(false)),
                ("u_periodic", b(false)),
                ("v_periodic", b(false)),
                ("u_closed", b(false)),
                ("v_closed", b(false)),
                ("bspline_vertices", ptr(3)),
                ("u_knots", ptr(4)),
                ("u_knot_mult", ptr(5)),
                ("v_knots", ptr(6)),
                ("v_knot_mult", ptr(7)),
            ],
        ),
        node(3, "BSPLINE_VERTICES", vec![("vertices", arr_f(&verts))]),
        node(4, "KNOT_SET", vec![("knots", arr_f(&[0.0, 1.0]))]),
        node(5, "KNOT_MULT", vec![("mult", arr_i(&[3, 3]))]),
        node(6, "KNOT_SET", vec![("knots", arr_f(&[0.0, 0.5, 1.0]))]),
        node(7, "KNOT_MULT", vec![("mult", arr_i(&[3, 1, 3]))]),
    ]
}

#[test]
fn bsurface_planar_net_stays_planar_and_inverts() {
    let (_g, s) = surface_of(planar_bsurface_nodes(), 1);
    assert!(s.period_u().is_none() && s.period_v().is_none());
    // corners interpolate the corner control points — this also pins the
    // v-fastest control net ordering (a u-fastest read gives (0,0)/(3,0)/(0,2))
    close3(s.eval([0.0, 0.0]), [0.0, 0.0, 1.0], 1e-12);
    close3(s.eval([1.0, 0.0]), [2.0, 0.0, 5.0], 1e-9);
    close3(s.eval([0.0, 1.0]), [0.0, 3.0, 10.0], 1e-9);
    close3(s.eval([1.0, 1.0]), [2.0, 3.0, 14.0], 1e-9);
    for k in 0..11 {
        let uv = [k as f64 / 10.0, ((k * 3) % 11) as f64 / 10.0];
        let p = s.eval(uv);
        close(p[2], 2.0 * p[0] + 3.0 * p[1] + 1.0, 1e-9); // in the plane
        let back = s.inv(p);
        close(back[0], uv[0], 1e-6);
        close(back[1], uv[1], 1e-6);
        close3(s.eval(back), p, 1e-9);
    }
    // the parametric normal is the plane normal, up to sign
    let n = s.normal([0.4, 0.6]);
    let expect = {
        let v = [-2.0, -3.0, 1.0_f64];
        let l = norm(v);
        [v[0] / l, v[1] / l, v[2] / l]
    };
    close(dot(n, expect).abs(), 1.0, 1e-6);
}

#[test]
fn bsurface_curved_net_inverts() {
    // bump the middle row out of plane so the surface is genuinely curved
    let (nu, nv) = (3usize, 4usize);
    let mut verts = Vec::new();
    for iu in 0..nu {
        for iv in 0..nv {
            let (x, y) = (iu as f64, iv as f64);
            let z = if iu == 1 { 1.5 } else { 0.0 };
            verts.extend_from_slice(&[x, y, z]);
        }
    }
    let mut nodes = planar_bsurface_nodes();
    nodes[2] = node(3, "BSPLINE_VERTICES", vec![("vertices", arr_f(&verts))]);
    let (_g, s) = surface_of(nodes, 1);
    for k in 0..=10 {
        for j in 0..=4 {
            let uv = [k as f64 / 10.0, j as f64 / 4.0];
            let p = s.eval(uv);
            let back = s.inv(p);
            close3(s.eval(back), p, 1e-8);
            close(back[0], uv[0], 1e-5);
            close(back[1], uv[1], 1e-5);
        }
    }
}

#[test]
fn bsurface_rational_weights() {
    // a rational B_SURFACE whose u direction is a quarter unit circle,
    // extruded along z: (wx,wy,wz,w) storage, v-fastest net.
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let cps: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
    let ws = [1.0, w, 1.0];
    let mut verts = Vec::new();
    for iu in 0..3 {
        for iv in 0..2 {
            let wt = ws[iu];
            let c = cps[iu];
            verts.extend_from_slice(&[
                wt * c[0],
                wt * c[1],
                wt * (c[2] + iv as f64), // z = 0 or 1
                wt,
            ]);
        }
    }
    let mut nodes = planar_bsurface_nodes();
    nodes[1] = node(
        2,
        "NURBS_SURF",
        vec![
            ("u_degree", i(2)),
            ("v_degree", i(1)),
            ("n_u_vertices", i(3)),
            ("n_v_vertices", i(2)),
            ("vertex_dim", i(4)),
            ("rational", b(true)),
            ("bspline_vertices", ptr(3)),
            ("u_knots", ptr(4)),
            ("u_knot_mult", ptr(5)),
            ("v_knots", ptr(6)),
            ("v_knot_mult", ptr(7)),
        ],
    );
    nodes[2] = node(3, "BSPLINE_VERTICES", vec![("vertices", arr_f(&verts))]);
    nodes[5] = node(6, "KNOT_SET", vec![("knots", arr_f(&[0.0, 1.0]))]);
    nodes[6] = node(7, "KNOT_MULT", vec![("mult", arr_i(&[2, 2]))]);
    let (_g, s) = surface_of(nodes, 1);
    for k in 0..=8 {
        let uv = [k as f64 / 8.0, 0.25];
        let p = s.eval(uv);
        close(norm([p[0], p[1], 0.0]), 1.0, 1e-12); // exact circle
        close(p[2], 0.25, 1e-12);
        let back = s.inv(p);
        close3(s.eval(back), p, 1e-9);
    }
}

// ── BLENDED_EDGE ────────────────────────────────────────────────────────────

/// Rolling ball of radius 1 in the concave corner of the planes y=0 and x=0,
/// spine running up z at (1,1,z).
fn blend_nodes(support_a: NodeId) -> Vec<Node> {
    vec![
        node(
            1,
            "BLENDED_EDGE",
            vec![
                ("blend_type", Value::Char("R".into())),
                ("range", arr_f(&[1.0, 1.0])),
                ("spine", ptr(2)),
                ("surface", Value::Array(vec![ptr(support_a), ptr(4)])),
            ],
        ),
        node(
            2,
            "LINE",
            vec![
                ("pvec", v3(1.0, 1.0, 0.0)),
                ("direction", v3(0.0, 0.0, 1.0)),
            ],
        ),
        // wall 1: the plane y = 0
        plane_node(3, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        // wall 2: the plane x = 0
        plane_node(4, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        // centre-locus form of wall 1: y = 0 offset by the blend radius
        node(
            5,
            "OFFSET_SURF",
            vec![("surface", ptr(3)), ("offset", f(1.0))],
        ),
    ]
}

#[test]
fn blended_edge_rolling_ball_arc() {
    let (_g, s) = surface_of(blend_nodes(3), 1);
    // the spine here is an unbounded LINE, so its sampling cache (and hence
    // inv's u range) defaults to [0, 1] — keep the test inside that window
    for k in 0..5 {
        let z = k as f64 * 0.25;
        let c = [1.0, 1.0, z];
        // v = 0 touches wall 1 (y = 0), v = 1 touches wall 2 (x = 0)
        close3(s.eval([z, 0.0]), [1.0, 0.0, z], 1e-9);
        close3(s.eval([z, 1.0]), [0.0, 1.0, z], 1e-9);
        let mid = s.eval([z, 0.5]);
        let r2 = std::f64::consts::FRAC_1_SQRT_2;
        close3(mid, [1.0 - r2, 1.0 - r2, z], 1e-9);
        // every section point is exactly the ball radius from the spine
        for j in 0..=8 {
            let v = j as f64 / 8.0;
            let p = s.eval([z, v]);
            close(dist(p, c), 1.0, 1e-9);
            let back = s.inv(p);
            close(back[0], z, 1e-6);
            close(back[1], v, 1e-6);
        }
    }
}

#[test]
fn blended_edge_uses_offset_support_base() {
    // support given as the centre-locus OFFSET_SURF: identical geometry
    let (_g, s) = surface_of(blend_nodes(5), 1);
    close3(s.eval([0.0, 0.0]), [1.0, 0.0, 0.0], 1e-9);
    close3(s.eval([0.0, 1.0]), [0.0, 1.0, 0.0], 1e-9);
}

/// A negative `range[0]` (seen in bbox-precision.SLDPRT) encodes convexity,
/// not a side flip: the radius is its magnitude.
#[test]
fn blended_edge_negative_range_is_a_magnitude() {
    let mut nodes = blend_nodes(3);
    if let Some(fld) = nodes[0].fields.iter_mut().find(|(k, _)| k == "range") {
        fld.1 = arr_f(&[-1.0, 1.0]);
    }
    let (_g, s) = surface_of(nodes, 1);
    close3(s.eval([0.0, 0.0]), [1.0, 0.0, 0.0], 1e-9);
    close3(s.eval([0.0, 1.0]), [0.0, 1.0, 0.0], 1e-9);
}

#[test]
fn blended_edge_requires_two_supports() {
    let mut nodes = blend_nodes(3);
    nodes[0] = node(
        1,
        "BLENDED_EDGE",
        vec![
            ("range", arr_f(&[1.0, 1.0])),
            ("spine", ptr(2)),
            ("surface", ptr(3)),
        ],
    );
    let g = Graph::new(nodes);
    assert!(make_surface(&g, g.get(1).unwrap()).is_none());
}

// ── de Boor basis ───────────────────────────────────────────────────────────

#[test]
fn deboor_partition_of_unity() {
    use solid_diff::geom::curves::deboor_basis;
    let knots = vec![0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0];
    let ncp = 7;
    for k in 0..=40 {
        let t = k as f64 / 40.0;
        let (span, basis) = deboor_basis(&knots, 3, ncp, t);
        assert!((3..ncp).contains(&span), "span {span} at t={t}");
        let sum: f64 = basis.iter().sum();
        close(sum, 1.0, 1e-12);
        assert!(basis.iter().all(|&b| (-1e-15..=1.0 + 1e-15).contains(&b)));
    }
    // out-of-range parameters clamp instead of panicking
    let (_s, b) = deboor_basis(&knots, 3, ncp, -5.0);
    close(b.iter().sum::<f64>(), 1.0, 1e-12);
    let (_s, b) = deboor_basis(&knots, 3, ncp, 5.0);
    close(b.iter().sum::<f64>(), 1.0, 1e-12);
}

/// A circular profile revolved about an axis — a tube bend, as found on the
/// `11245A82` pull handle. `v` runs around the circular section, so it must be
/// reported as periodic: without that, the tessellator places its "provably
/// outside" trimming anchor just below the smallest `v`, which on a wrapped
/// parameter is actually *inside* the tube, and the face collapses.
#[test]
fn spun_surf_with_a_closed_profile_is_periodic_in_v() {
    let nodes = vec![
        node(
            1,
            "SPUN_SURF",
            vec![
                ("profile", ptr(2)),
                ("base", v3(0.0, 0.0, 0.0)),
                ("axis", v3(0.0, 0.0, 1.0)),
                ("x_axis", v3(1.0, 0.0, 0.0)),
            ],
        ),
        // circle of radius 0.25 centred 2.0 out along x, in the xz plane
        node(
            2,
            "CIRCLE",
            vec![
                ("centre", v3(2.0, 0.0, 0.0)),
                ("normal", v3(0.0, 1.0, 0.0)),
                ("x_axis", v3(1.0, 0.0, 0.0)),
                ("radius", f(0.25)),
            ],
        ),
    ];
    let (_g, s) = surface_of(nodes, 1);
    assert_eq!(s.period_u(), Some(TWO_PI), "revolve angle wraps");
    assert_eq!(
        s.period_v(),
        Some(TWO_PI),
        "a closed profile makes v wrap too"
    );
    // One full period along v returns to the same point.
    for k in 0..6 {
        let uv = [k as f64 * 0.9, k as f64 * 0.7];
        close3(s.eval([uv[0], uv[1] + TWO_PI]), s.eval(uv), 1e-9);
    }
    // Every point sits on the tube: distance to the spine circle is the
    // section radius.
    for k in 0..12 {
        let uv = [k as f64 * 0.5, k as f64 * 0.4];
        let p = s.eval(uv);
        let spine_r = norm([p[0], p[1], 0.0]);
        let d = ((spine_r - 2.0).powi(2) + p[2] * p[2]).sqrt();
        close(d, 0.25, 1e-9);
    }
}

// ── INTERSECTION curves refined against their two surfaces (#23) ────────────

/// A chart deliberately stored at only four samples, for a curve whose true
/// shape is known exactly: a cylinder of radius R about the z axis cut by the
/// plane z = 0 is the circle x^2 + y^2 = R^2.
fn coarse_circle_intersection(r: f64, samples: usize) -> Vec<Node> {
    let mut hvec = Vec::new();
    for k in 0..=samples {
        let a = TWO_PI * (k as f64) / (samples as f64);
        hvec.extend_from_slice(&[r * a.cos(), r * a.sin(), 0.0]);
    }
    vec![
        node(
            1,
            "INTERSECTION",
            vec![
                (
                    "surface",
                    Value::Array(vec![Value::Ptr(Some(2)), Value::Ptr(Some(3))]),
                ),
                ("chart", Value::Ptr(Some(4))),
            ],
        ),
        node(
            2,
            "CYLINDER",
            vec![
                ("pvec", v3(0.0, 0.0, 0.0)),
                ("axis", v3(0.0, 0.0, 1.0)),
                ("radius", f(r)),
                ("x_axis", v3(1.0, 0.0, 0.0)),
            ],
        ),
        plane_node(3, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        node(
            4,
            "CHART",
            vec![("hvec", arr_f(&hvec)), ("chordal_error", f(1e-6))],
        ),
    ]
}

#[test]
fn intersection_curve_is_pulled_onto_both_surfaces() {
    let r = 2.0;
    // Four samples means the raw chart is a square inscribed in the circle:
    // its edge midpoints sit at radius r/sqrt(2), a 29% error.
    let (_g, c) = curve_of(coarse_circle_intersection(r, 4), 1);
    let (t0, t1) = c.full_range().expect("range");

    let mut worst: f64 = 0.0;
    for k in 0..=400 {
        let p = c.eval(t0 + (t1 - t0) * (k as f64 / 400.0));
        let radial = (p[0] * p[0] + p[1] * p[1]).sqrt();
        worst = worst.max((radial - r).abs()).max(p[2].abs());
    }
    // Every evaluated point must lie on the true circle, not on the chords.
    assert!(
        worst < 1e-4,
        "refined curve deviates from the analytic circle by {worst:.3e}"
    );
}

#[test]
fn unrefined_chart_would_have_been_far_off() {
    // Guards the test above from passing vacuously: with the same four samples
    // and no surfaces to refine against, the midpoints really are 29% out.
    let mut nodes = coarse_circle_intersection(2.0, 4);
    nodes[0].fields.retain(|(k, _)| k != "surface");
    let (_g, c) = curve_of(nodes, 1);
    let (t0, t1) = c.full_range().expect("range");
    let p = c.eval(t0 + (t1 - t0) * 0.125); // midpoint of the first chord
    let radial = (p[0] * p[0] + p[1] * p[1]).sqrt();
    assert!(
        (radial - 2.0).abs() > 0.5,
        "expected a large chord error without refinement, got {radial}"
    );
}

#[test]
fn polyline_inv_projects_onto_segments_not_samples() {
    // Straight polyline with one long segment: inv must return a parameter in
    // the segment's interior, not snap to whichever endpoint is nearest.
    let nodes = vec![
        node(1, "INTERSECTION", vec![("chart", Value::Ptr(Some(2)))]),
        node(
            2,
            "CHART",
            vec![("hvec", arr_f(&[0.0, 0.0, 0.0, 10.0, 0.0, 0.0]))],
        ),
    ];
    let (_g, c) = curve_of(nodes, 1);
    let probe = [3.7, 0.0, 0.0];
    let back = c.eval(c.inv(probe));
    assert!(
        dist3(back, probe) < 1e-9,
        "inv/eval round trip landed at {back:?}, not {probe:?}"
    );
}

fn dist3(a: P3, b: P3) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
