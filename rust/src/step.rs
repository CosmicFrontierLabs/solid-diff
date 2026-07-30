//! STEP (AP214) export of the Parasolid B-rep.
//!
//! This exists so OCCT can do the tessellation. A season of writing our own
//! trimmer established that the hard part is not evaluating surfaces -- ours
//! are exact to 3e-10 -- but closing trimmed regions in parameter space, which
//! is precisely the machinery a mature kernel already has (pcurve projection,
//! `ShapeFix`, seam insertion; the documented motivating case for OCCT's
//! `FixMissingSeam` is a cylinder from a Parasolid-kernel system).
//!
//! Two deliberate choices shape the output:
//!
//! * **Surfaces are exact** wherever STEP has the type: plane, cylinder, cone,
//!   sphere, torus, NURBS, extrusion, revolution, offset. Only `BLENDED_EDGE`
//!   (our reconstruction, not the file's) is sampled.
//! * **Edges are sampled polylines**, not analytic curves, taken from the same
//!   shared `EdgeSampler` the native tessellator uses. That sidesteps in one
//!   move every edge pathology this project has hit: the two-arcs ambiguity on
//!   periodic curves (XT stores no direction flag), `INTERSECTION` curves with
//!   no closed form, `SP_CURVE`, and edges with a null curve pointer. Both
//!   faces of an edge reference one polyline, so the shell sews.
//!
//! OCCT computes pcurves and heals the topology on import; we do not emit
//! pcurves at all.

use std::collections::HashMap;
use std::path::Path;

use crate::geom::{curves::make_curve, P3};
use crate::graph::Graph;
use crate::tess::EdgeSampler;
use crate::value::{Node, NodeId};

/// A finished export: the STEP text plus the FACE node ids actually written,
/// in emission order, for whoever needs to map OCCT faces back to ours.
pub struct StepExport {
    pub text: String,
    pub faces: Vec<NodeId>,
    pub skipped: usize,
}

/// Format a real the way ISO 10303-21 wants: always with a decimal point,
/// even in exponent form (`1.0E-7`, never `1E-7`).
fn fr(x: f64) -> String {
    if !x.is_finite() {
        return "0.0".to_string();
    }
    let s = format!("{x:E}");
    match s.split_once('E') {
        Some((m, e)) => {
            let m = if m.contains('.') {
                m.to_string()
            } else {
                format!("{m}.0")
            };
            // Small exponents read better and parse everywhere.
            if let Ok(exp) = e.parse::<i32>() {
                if (-4..=6).contains(&exp) {
                    return format!("{x:?}");
                }
            }
            format!("{m}E{e}")
        }
        None => s,
    }
}

struct W {
    body: String,
    n: u32,
}

impl W {
    fn add(&mut self, s: &str) -> u32 {
        self.n += 1;
        self.body.push('#');
        self.body.push_str(&self.n.to_string());
        self.body.push('=');
        self.body.push_str(s);
        self.body.push_str(";\n");
        self.n
    }

    fn cart(&mut self, p: P3) -> u32 {
        self.add(&format!(
            "CARTESIAN_POINT('',({},{},{}))",
            fr(p[0]),
            fr(p[1]),
            fr(p[2])
        ))
    }

    fn dir(&mut self, v: P3) -> u32 {
        self.add(&format!(
            "DIRECTION('',({},{},{}))",
            fr(v[0]),
            fr(v[1]),
            fr(v[2])
        ))
    }

    fn ax2(&mut self, origin: P3, z: P3, x: P3) -> u32 {
        let c = self.cart(origin);
        let dz = self.dir(z);
        let dx = self.dir(x);
        self.add(&format!("AXIS2_PLACEMENT_3D('',#{c},#{dz},#{dx})"))
    }
}

/// Compress an expanded knot vector into (distinct knots, multiplicities).
fn compress_knots(expanded: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut knots: Vec<f64> = Vec::new();
    let mut mults = Vec::new();
    for &k in expanded {
        match knots.last() {
            Some(&last) if (k - last).abs() <= 1e-12 * (1.0 + k.abs()) => {
                *mults.last_mut().unwrap() += 1;
            }
            _ => {
                knots.push(k);
                mults.push(1usize);
            }
        }
    }
    (knots, mults)
}

fn list_u32(ids: &[u32]) -> String {
    let inner: Vec<String> = ids.iter().map(|i| format!("#{i}")).collect();
    format!("({})", inner.join(","))
}

fn list_f(xs: &[f64]) -> String {
    let inner: Vec<String> = xs.iter().map(|x| fr(*x)).collect();
    format!("({})", inner.join(","))
}

fn list_i(xs: &[usize]) -> String {
    let inner: Vec<String> = xs.iter().map(|x| x.to_string()).collect();
    format!("({})", inner.join(","))
}

/// Degree-1 B-spline through the given points, chord-length parameterized.
/// This is how every sampled polyline (edge or sweep section) is carried.
fn polyline_bspline(w: &mut W, pts: &[P3]) -> Option<u32> {
    if pts.len() < 2 {
        return None;
    }
    let ids: Vec<u32> = pts.iter().map(|p| w.cart(*p)).collect();
    let mut knots = Vec::with_capacity(pts.len());
    let mut acc = 0.0;
    knots.push(0.0);
    for i in 1..pts.len() {
        let d = crate::geom::dist(pts[i], pts[i - 1]).max(1e-12);
        acc += d;
        knots.push(acc);
    }
    let mut mults = vec![1usize; pts.len()];
    mults[0] = 2;
    *mults.last_mut().unwrap() = 2;
    Some(w.add(&format!(
        "B_SPLINE_CURVE_WITH_KNOTS('',1,{},.POLYLINE_FORM.,.F.,.F.,{},{},.UNSPECIFIED.)",
        list_u32(&ids),
        list_i(&mults),
        list_f(&knots),
    )))
}

/// A curve entity for sweep sections and revolution profiles: exact where
/// STEP has the type, sampled otherwise.
fn curve_entity(w: &mut W, graph: &Graph, node: &Node) -> Option<u32> {
    match node.name.as_str() {
        "LINE" => {
            let p = node.vec3("pvec")?;
            let d = node.vec3("direction")?;
            let c = w.cart(p);
            let dd = w.dir(d);
            let v = w.add(&format!("VECTOR('',#{dd},1.0)"));
            Some(w.add(&format!("LINE('',#{c},#{v})")))
        }
        "CIRCLE" => {
            let c = node.vec3("centre")?;
            let n = node.vec3("normal")?;
            let x = node.vec3("x_axis")?;
            let r = node.f64("radius")?;
            let ax = w.ax2(c, n, x);
            Some(w.add(&format!("CIRCLE('',#{ax},{})", fr(r))))
        }
        "ELLIPSE" => {
            let c = node.vec3("centre")?;
            let n = node.vec3("normal")?;
            let x = node.vec3("x_axis")?;
            let r1 = node.f64("major_radius")?;
            let r2 = node.f64("minor_radius")?;
            let ax = w.ax2(c, n, x);
            Some(w.add(&format!("ELLIPSE('',#{ax},{},{})", fr(r1), fr(r2))))
        }
        _ => {
            // Sampled: B_CURVE could be exact but sections are rare enough
            // that one code path beats two.
            let curve = make_curve(graph, node)?;
            let (t0, t1) = curve.full_range()?;
            let n = 64;
            let pts: Vec<P3> = (0..=n)
                .map(|i| curve.eval(t0 + (t1 - t0) * (i as f64 / n as f64)))
                .collect();
            polyline_bspline(w, &pts)
        }
    }
}

/// The exact NURBS surface, rational or not.
fn nurbs_surface(w: &mut W, graph: &Graph, bsurf: &Node) -> Option<u32> {
    let ns = graph.deref(bsurf, "nurbs")?;
    let pu = usize::try_from(ns.i64("u_degree")?).ok()?;
    let pv = usize::try_from(ns.i64("v_degree")?).ok()?;
    let nu = usize::try_from(ns.i64("n_u_vertices")?).ok()?;
    let nv = usize::try_from(ns.i64("n_v_vertices")?).ok()?;
    let dim = usize::try_from(ns.i64("vertex_dim")?).ok()?;
    let rational = ns.bool("rational");
    let verts = graph.deref(ns, "bspline_vertices")?.f64_vec("vertices")?;
    if verts.len() < nu * nv * dim || dim < 3 {
        return None;
    }
    let (uk, um) = {
        let knots = graph.deref(ns, "u_knots")?.f64_vec("knots")?;
        let mult = graph.deref(ns, "u_knot_mult")?.i64_vec("mult")?;
        let expanded: Vec<f64> = knots
            .iter()
            .zip(&mult)
            .flat_map(|(k, m)| std::iter::repeat_n(*k, *m as usize))
            .collect();
        compress_knots(&expanded)
    };
    let (vk, vm) = {
        let knots = graph.deref(ns, "v_knots")?.f64_vec("knots")?;
        let mult = graph.deref(ns, "v_knot_mult")?.i64_vec("mult")?;
        let expanded: Vec<f64> = knots
            .iter()
            .zip(&mult)
            .flat_map(|(k, m)| std::iter::repeat_n(*k, *m as usize))
            .collect();
        compress_knots(&expanded)
    };

    // Control net rows (u-major), STEP's ordering. Rational verts are stored
    // as (wx, wy, wz, w).
    let mut rows: Vec<String> = Vec::with_capacity(nu);
    let mut wrows: Vec<String> = Vec::with_capacity(nu);
    for iu in 0..nu {
        let mut ids = Vec::with_capacity(nv);
        let mut ws = Vec::with_capacity(nv);
        for iv in 0..nv {
            let row = &verts[(iu * nv + iv) * dim..(iu * nv + iv) * dim + dim];
            let (p, wt) = if rational && dim >= 4 {
                let wt = row[3];
                if wt == 0.0 || !wt.is_finite() {
                    return None;
                }
                ([row[0] / wt, row[1] / wt, row[2] / wt], wt)
            } else {
                ([row[0], row[1], row[2]], 1.0)
            };
            ids.push(w.cart(p));
            ws.push(wt);
        }
        rows.push(list_u32(&ids));
        wrows.push(list_f(&ws));
    }
    let net = format!("({})", rows.join(","));

    if rational {
        let weights = format!("({})", wrows.join(","));
        Some(w.add(&format!(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE({pu},{pv},{net},.UNSPECIFIED.,.F.,.F.,.F.)\
             B_SPLINE_SURFACE_WITH_KNOTS({},{},{},{},.UNSPECIFIED.)\
             GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE({weights})\
             REPRESENTATION_ITEM('')SURFACE())",
            list_i(&um),
            list_i(&vm),
            list_f(&uk),
            list_f(&vk),
        )))
    } else {
        Some(w.add(&format!(
            "B_SPLINE_SURFACE_WITH_KNOTS('',{pu},{pv},{net},.UNSPECIFIED.,.F.,.F.,.F.,{},{},{},{},.UNSPECIFIED.)",
            list_i(&um),
            list_i(&vm),
            list_f(&uk),
            list_f(&vk),
        )))
    }
}

/// A surface no STEP type fits (`BLENDED_EDGE`): sample a degree-1 grid over
/// the region the face's own boundary occupies, padded.
fn sampled_surface(
    w: &mut W,
    graph: &Graph,
    face: &Node,
    snode: &Node,
    sampler: &mut EdgeSampler,
) -> Option<u32> {
    let surf = crate::geom::surfaces::make_surface(graph, snode)?;
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for lp in graph.face_loops(face) {
        for he in graph.loop_halfedges(lp) {
            let Some(edge) = graph.deref(he, "edge") else {
                continue;
            };
            let Some(pts) = sampler.get(graph, edge) else {
                continue;
            };
            let mut uvs: Vec<[f64; 2]> = pts.iter().map(|p| surf.inv(*p)).collect();
            // Keep periodic coordinates continuous so the extent is sane.
            for dim in 0..2 {
                let period = if dim == 0 {
                    surf.period_u()
                } else {
                    surf.period_v()
                };
                let Some(p) = period else { continue };
                for i in 1..uvs.len() {
                    let prev = uvs[i - 1][dim];
                    uvs[i][dim] -= p * ((uvs[i][dim] - prev) / p).round();
                }
            }
            for q in &uvs {
                for d in 0..2 {
                    lo[d] = lo[d].min(q[d]);
                    hi[d] = hi[d].max(q[d]);
                }
            }
        }
    }
    if !lo[0].is_finite() || hi[0] <= lo[0] || hi[1] <= lo[1] {
        return None;
    }
    for d in 0..2 {
        let pad = (hi[d] - lo[d]) * 0.05;
        lo[d] -= pad;
        hi[d] += pad;
    }
    const NU: usize = 24;
    const NV: usize = 8;
    let mut rows = Vec::with_capacity(NU + 1);
    for iu in 0..=NU {
        let u = lo[0] + (hi[0] - lo[0]) * (iu as f64 / NU as f64);
        let mut ids = Vec::with_capacity(NV + 1);
        for iv in 0..=NV {
            let v = lo[1] + (hi[1] - lo[1]) * (iv as f64 / NV as f64);
            ids.push(w.cart(surf.eval([u, v])));
        }
        rows.push(list_u32(&ids));
    }
    let net = format!("({})", rows.join(","));
    let uk: Vec<f64> = (0..=NU).map(|i| i as f64).collect();
    let vk: Vec<f64> = (0..=NV).map(|i| i as f64).collect();
    let mut um = vec![1usize; NU + 1];
    um[0] = 2;
    um[NU] = 2;
    let mut vm = vec![1usize; NV + 1];
    vm[0] = 2;
    vm[NV] = 2;
    Some(w.add(&format!(
        "B_SPLINE_SURFACE_WITH_KNOTS('',1,1,{net},.UNSPECIFIED.,.F.,.F.,.F.,{},{},{},{},.UNSPECIFIED.)",
        list_i(&um),
        list_i(&vm),
        list_f(&uk),
        list_f(&vk),
    )))
}

/// One surface entity for a FACE, exact where possible.
fn surface_entity(
    w: &mut W,
    graph: &Graph,
    face: &Node,
    snode: &Node,
    sampler: &mut EdgeSampler,
) -> Option<u32> {
    match snode.name.as_str() {
        "PLANE" => {
            let p = snode.vec3("pvec")?;
            let n = snode.vec3("normal")?;
            let x = snode.vec3("x_axis")?;
            let ax = w.ax2(p, n, x);
            Some(w.add(&format!("PLANE('',#{ax})")))
        }
        "CYLINDER" => {
            let p = snode.vec3("pvec")?;
            let a = snode.vec3("axis")?;
            let x = snode.vec3("x_axis")?;
            let r = snode.f64("radius")?;
            let ax = w.ax2(p, a, x);
            Some(w.add(&format!("CYLINDRICAL_SURFACE('',#{ax},{})", fr(r))))
        }
        "CONE" => {
            let p = snode.vec3("pvec")?;
            let mut a = snode.vec3("axis")?;
            let x = snode.vec3("x_axis")?;
            let r = snode.f64("radius")?;
            let sin_ha = snode.f64("sin_half_angle")?;
            let cos_ha = snode.f64("cos_half_angle")?;
            if cos_ha == 0.0 {
                return None;
            }
            let mut semi = (sin_ha / cos_ha).atan();
            // STEP's radius grows along +Z; ours grows along +axis when
            // tan > 0. When it shrinks, present the flipped axis instead.
            if semi < 0.0 {
                semi = -semi;
                a = [-a[0], -a[1], -a[2]];
            }
            let ax = w.ax2(p, a, x);
            Some(w.add(&format!("CONICAL_SURFACE('',#{ax},{},{})", fr(r), fr(semi))))
        }
        "SPHERE" => {
            let c = snode.vec3("centre")?;
            let a = snode.vec3("axis")?;
            let x = snode.vec3("x_axis")?;
            let r = snode.f64("radius")?;
            let ax = w.ax2(c, a, x);
            Some(w.add(&format!("SPHERICAL_SURFACE('',#{ax},{})", fr(r))))
        }
        "TORUS" => {
            let c = snode.vec3("centre")?;
            let a = snode.vec3("axis")?;
            let x = snode.vec3("x_axis")?;
            let major = snode.f64("major_radius")?;
            let minor = snode.f64("minor_radius")?;
            let ax = w.ax2(c, a, x);
            Some(w.add(&format!(
                "TOROIDAL_SURFACE('',#{ax},{},{})",
                fr(major),
                fr(minor)
            )))
        }
        "SWEPT_SURF" => {
            let section = graph.deref(snode, "section")?;
            let d = snode.vec3("sweep")?;
            let cid = curve_entity(w, graph, section)?;
            let dd = w.dir(d);
            let v = w.add(&format!("VECTOR('',#{dd},1.0)"));
            Some(w.add(&format!("SURFACE_OF_LINEAR_EXTRUSION('',#{cid},#{v})")))
        }
        "SPUN_SURF" => {
            let profile = graph.deref(snode, "profile")?;
            let p0 = snode.vec3("base").or_else(|| snode.vec3("pvec"))?;
            let a = snode.vec3("axis")?;
            let cid = curve_entity(w, graph, profile)?;
            let c = w.cart(p0);
            let dz = w.dir(a);
            let ax1 = w.add(&format!("AXIS1_PLACEMENT('',#{c},#{dz})"));
            Some(w.add(&format!("SURFACE_OF_REVOLUTION('',#{cid},#{ax1})")))
        }
        "OFFSET_SURF" => {
            let base = graph.deref(snode, "surface")?;
            let mut o = snode.f64("offset")?;
            if !snode.sense_positive() {
                o = -o;
            }
            let bid = surface_entity(w, graph, face, base, sampler)?;
            Some(w.add(&format!("OFFSET_SURFACE('',#{bid},{},.F.)", fr(o))))
        }
        "B_SURFACE" => nurbs_surface(w, graph, snode),
        // BLENDED_EDGE and anything else we can evaluate but STEP cannot name.
        _ => sampled_surface(w, graph, face, snode, sampler),
    }
}

/// Emit the whole body. `tol` is the edge-sampling chord tolerance in model
/// units; `None` picks 5e-4 of part size (finer than the native tessellator,
/// because these polylines *are* the boundary geometry downstream).
pub fn export(graph: &Graph, tol: Option<f64>) -> StepExport {
    export_faces(graph, tol, None)
}

/// As [`export`], restricted to the given FACE node ids (debugging aid).
pub fn export_faces(graph: &Graph, tol: Option<f64>, only: Option<&[NodeId]>) -> StepExport {
    let scale = graph.model_scale();
    let tol = tol.unwrap_or(5e-4 * scale).max(1e-9);
    let mut sampler = EdgeSampler::new(tol);
    let mut w = W {
        body: String::new(),
        n: 0,
    };

    // Shared vertex and edge entities.
    #[derive(Clone, Copy)]
    struct EdgeEnt {
        ec: u32,
        v1: u32,
        v2: u32,
        first: P3,
        last: P3,
    }
    let mut vert_ids: HashMap<NodeId, u32> = HashMap::new();
    let mut edge_ids: HashMap<NodeId, EdgeEnt> = HashMap::new();

    let faces = graph.by_type("FACE");
    let mut face_entities: Vec<u32> = Vec::new();
    let mut face_nodes: Vec<NodeId> = Vec::new();
    let mut skipped = 0usize;

    for face in &faces {
        if let Some(only) = only {
            if !only.contains(&face.id) {
                continue;
            }
        }
        let Some(snode) = graph.deref(face, "surface") else {
            skipped += 1;
            continue;
        };
        let Some(surf_id) = surface_entity(&mut w, graph, face, snode, &mut sampler) else {
            skipped += 1;
            continue;
        };

        // Gather each loop first: winding loops on a periodic surface must be
        // joined through a seam before they can bound anything.
        struct LoopRec {
            /// (EDGE_CURVE id, traversed forward)
            run: Vec<(u32, bool)>,
            /// Vertex entity + point where the traversal starts.
            anchor: Option<(u32, P3)>,
            /// (period dimension, direction) when the loop winds a period.
            winding: Option<(usize, f64)>,
            /// Unwrapped UV bounding box of the loop's samples.
            uv_lo: [f64; 2],
            uv_hi: [f64; 2],
        }
        let surf_eval = crate::geom::surfaces::make_surface(graph, snode);
        let mut recs: Vec<LoopRec> = Vec::new();
        for lp in graph.face_loops(face) {
            let mut run: Vec<(u32, bool)> = Vec::new();
            let mut chain: Vec<P3> = Vec::new();
            let mut anchor: Option<(u32, P3)> = None;
            for he in graph.loop_halfedges(lp) {
                let Some(edge) = graph.deref(he, "edge") else {
                    continue; // vertex loop: no edge to walk
                };
                let ent = match edge_ids.get(&edge.id) {
                    Some(e) => *e,
                    None => {
                        let Some(pts) = sampler.get(graph, edge) else {
                            continue;
                        };
                        let Some(curve_id) = polyline_bspline(&mut w, &pts) else {
                            continue;
                        };
                        let vid = |w: &mut W,
                                   vert_ids: &mut HashMap<NodeId, u32>,
                                   hnode: Option<&Node>,
                                   at: P3|
                         -> u32 {
                            let key = hnode.and_then(|h| graph.deref(h, "vertex")).map(|v| v.id);
                            if let Some(k) = key {
                                if let Some(id) = vert_ids.get(&k) {
                                    return *id;
                                }
                            }
                            let c = w.cart(at);
                            let id = w.add(&format!("VERTEX_POINT('',#{c})"));
                            if let Some(k) = key {
                                vert_ids.insert(k, id);
                            }
                            id
                        };
                        let hp = graph.deref(edge, "halfedge");
                        let hp = match hp {
                            Some(h) if h.sense_positive() => Some(h),
                            Some(h) => graph.deref(h, "other"),
                            None => None,
                        };
                        let hm = hp.and_then(|h| graph.deref(h, "other"));
                        // Closed is a fact of the topology (a rim carries no
                        // vertices, or one vertex at both ends); proximity is
                        // not evidence, or genuinely short edges lose a vertex
                        // and their wire acquires a gap.
                        let start_vertex = hp.and_then(|h| graph.deref(h, "vertex"));
                        let end_vertex = hm.and_then(|h| graph.deref(h, "vertex"));
                        let closed = match (start_vertex, end_vertex) {
                            (None, _) | (_, None) => true,
                            (Some(a), Some(b)) => a.id == b.id,
                        };
                        let v1 = vid(&mut w, &mut vert_ids, hp, pts[0]);
                        let v2 = if closed {
                            v1
                        } else {
                            vid(&mut w, &mut vert_ids, hm, *pts.last().unwrap())
                        };
                        let ec = w.add(&format!("EDGE_CURVE('',#{v1},#{v2},#{curve_id},.T.)"));
                        let ent = EdgeEnt {
                            ec,
                            v1,
                            v2,
                            first: pts[0],
                            last: *pts.last().unwrap(),
                        };
                        edge_ids.insert(edge.id, ent);
                        ent
                    }
                };
                let fwd = he.sense_positive();
                run.push((ent.ec, fwd));
                if anchor.is_none() {
                    anchor = Some(if fwd {
                        (ent.v1, ent.first)
                    } else {
                        (ent.v2, ent.last)
                    });
                }
                if let Some(pts) = sampler.get(graph, edge) {
                    if fwd {
                        chain.extend(pts);
                    } else {
                        chain.extend(pts.into_iter().rev());
                    }
                }
            }
            if run.is_empty() {
                continue;
            }
            // Does this loop wind a period? Unwrap its UV trace and take the
            // net travel: a rim accumulates a full period, a hole nets out to
            // zero. Geometry decides; the file's orientation flags do not.
            let mut winding = None;
            let mut uv_lo = [f64::INFINITY; 2];
            let mut uv_hi = [f64::NEG_INFINITY; 2];
            if let Some(surf) = surf_eval.as_ref() {
                // One continuous unwrapped trace serves the winding test and
                // the bbox both.
                let mut prev: Option<[f64; 2]> = None;
                let mut acc = [0.0f64; 2];
                for q3 in &chain {
                    let mut c = surf.inv(*q3);
                    for (dim, period) in [(0usize, surf.period_u()), (1usize, surf.period_v())] {
                        let Some(p) = period else { continue };
                        if let Some(pr) = prev {
                            let mut d = c[dim] - pr[dim];
                            d -= p * (d / p).round();
                            acc[dim] += d;
                            c[dim] = pr[dim] + d;
                        }
                    }
                    for dim in 0..2 {
                        uv_lo[dim] = uv_lo[dim].min(c[dim]);
                        uv_hi[dim] = uv_hi[dim].max(c[dim]);
                    }
                    prev = Some(c);
                }
                for (dim, period) in [(0usize, surf.period_u()), (1usize, surf.period_v())] {
                    let Some(p) = period else { continue };
                    if acc[dim].abs() > p * 0.75 {
                        winding = Some((dim, acc[dim].signum()));
                        break;
                    }
                }
            }
            recs.push(LoopRec {
                run,
                anchor,
                winding,
                uv_lo,
                uv_hi,
            });
        }

        // Join pairs of winding loops through a seam. The seam edge appears
        // twice in the combined wire with opposite orientations, which is how
        // STEP natively represents a face on a closed surface; without it the
        // reader drops one of the rims and the trim collapses -- measured on a
        // plate, whose drilled holes came back filled.
        let oe = |w: &mut W, ec: u32, fwd: bool| -> u32 {
            let flag = if fwd { ".T." } else { ".F." };
            w.add(&format!("ORIENTED_EDGE('',*,*,#{ec},{flag})"))
        };
        let mut bounds: Vec<u32> = Vec::new();
        let mut used = vec![false; recs.len()];
        for i in 0..recs.len() {
            if used[i] {
                continue;
            }
            let Some((dim, sign_i)) = recs[i].winding else {
                continue;
            };
            // Partner: the *nearest* unused winding loop in the same period
            // direction. Nearest in the other coordinate, because a face can
            // carry several bands and pairing across bands ties the boundary
            // in a knot.
            let anchor_i = recs[i].anchor.map(|(_, p)| p);
            let Some(j) = (i + 1..recs.len())
                .filter(|j| !used[*j] && recs[*j].winding.map(|(d, _)| d == dim).unwrap_or(false))
                .min_by(|a, b| {
                    let d = |k: usize| -> f64 {
                        match (anchor_i, recs[k].anchor) {
                            (Some(pa), Some((_, pb))) => crate::geom::dist(pa, pb),
                            _ => f64::INFINITY,
                        }
                    };
                    d(*a)
                        .partial_cmp(&d(*b))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
            else {
                continue;
            };
            let (Some((va, pa)), Some((vb, pb))) = (recs[i].anchor, recs[j].anchor) else {
                continue;
            };
            let Some(surf) = surf_eval.as_ref() else {
                continue;
            };
            // The seam must not cross any other loop, or its wire intersects
            // that loop's wire and the face mistrims. Check the straight UV
            // path between the anchors against every other loop's box; when
            // something is in the way, leave the face to the gate rather
            // than guess.
            let period = if dim == 0 {
                surf.period_u()
            } else {
                surf.period_v()
            }
            .unwrap_or(f64::INFINITY);
            let ua = surf.inv(pa);
            let mut ub = surf.inv(pb);
            for (d, per) in [(0usize, surf.period_u()), (1usize, surf.period_v())] {
                let Some(p) = per else { continue };
                ub[d] -= p * ((ub[d] - ua[d]) / p).round();
            }
            let margin = period * 0.02;
            let crosses = recs.iter().enumerate().any(|(k, r)| {
                if k == i || k == j || r.run.is_empty() || !r.uv_lo[dim].is_finite() {
                    return false;
                }
                // Compare in the same period window as the anchor.
                let shift =
                    period * (((r.uv_lo[dim] + r.uv_hi[dim]) * 0.5 - ua[dim]) / period).round();
                let (lo, hi) = (r.uv_lo[dim] - shift, r.uv_hi[dim] - shift);
                ua[dim] >= lo - margin && ua[dim] <= hi + margin
            });
            if crosses {
                continue;
            }
            const SEAM_STEPS: usize = 16;
            let mut seam_pts: Vec<P3> = Vec::with_capacity(SEAM_STEPS + 1);
            seam_pts.push(pa);
            for k in 1..SEAM_STEPS {
                let t = k as f64 / SEAM_STEPS as f64;
                seam_pts
                    .push(surf.eval([ua[0] + (ub[0] - ua[0]) * t, ua[1] + (ub[1] - ua[1]) * t]));
            }
            seam_pts.push(pb);
            let Some(seam_curve) = polyline_bspline(&mut w, &seam_pts) else {
                continue;
            };
            let seam_ec = w.add(&format!("EDGE_CURVE('',#{va},#{vb},#{seam_curve},.T.)"));

            // The rims must travel opposite ways round the period to bound a
            // band between them; reverse B's run when it agrees with A.
            let sign_j = recs[j].winding.unwrap().1;
            let b_run: Vec<(u32, bool)> = if sign_j == sign_i {
                recs[j].run.iter().rev().map(|(ec, f)| (*ec, !*f)).collect()
            } else {
                recs[j].run.clone()
            };
            let mut circuit: Vec<u32> = recs[i]
                .run
                .iter()
                .map(|(ec, f)| oe(&mut w, *ec, *f))
                .collect();
            circuit.push(oe(&mut w, seam_ec, true));
            circuit.extend(b_run.iter().map(|(ec, f)| oe(&mut w, *ec, *f)));
            circuit.push(oe(&mut w, seam_ec, false));
            let el = w.add(&format!("EDGE_LOOP('',{})", list_u32(&circuit)));
            bounds.push(w.add(&format!("FACE_BOUND('',#{el},.T.)")));
            used[i] = true;
            used[j] = true;
        }
        for (i, rec) in recs.iter().enumerate() {
            if used[i] || rec.run.is_empty() {
                continue;
            }
            let ids: Vec<u32> = rec.run.iter().map(|(ec, f)| oe(&mut w, *ec, *f)).collect();
            let el = w.add(&format!("EDGE_LOOP('',{})", list_u32(&ids)));
            bounds.push(w.add(&format!("FACE_BOUND('',#{el},.T.)")));
        }

        // Outward normal = parametric normal x surface sense x face sense; the
        // STEP surface reproduces the parametric normal, so same_sense is the
        // product of the two senses.
        let same = if face.sense_positive() == snode.sense_positive() {
            ".T."
        } else {
            ".F."
        };
        let af = w.add(&format!(
            "ADVANCED_FACE('f{}',{},#{surf_id},{same})",
            face.id,
            list_u32(&bounds)
        ));
        face_entities.push(af);
        face_nodes.push(face.id);
    }

    // Solid bodies get a closed shell; everything else is a sheet.
    let solid = graph
        .by_type("BODY")
        .first()
        .and_then(|b| b.i64("body_type"))
        .map(|t| t == 1)
        .unwrap_or(true);

    let (shape_id, shape_kind) = if solid {
        let shell = w.add(&format!("CLOSED_SHELL('',{})", list_u32(&face_entities)));
        let s = w.add(&format!("MANIFOLD_SOLID_BREP('',#{shell})"));
        (s, "ADVANCED_BREP_SHAPE_REPRESENTATION")
    } else {
        let shell = w.add(&format!("OPEN_SHELL('',{})", list_u32(&face_entities)));
        let s = w.add(&format!("SHELL_BASED_SURFACE_MODEL('',(#{shell}))"));
        (s, "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
    };

    // Context boilerplate: metres, radians, 1e-6 uncertainty.
    let app = w.add("APPLICATION_CONTEXT('automotive design')");
    w.add(&format!(
        "APPLICATION_PROTOCOL_DEFINITION('','automotive_design',2010,#{app})"
    ));
    let pctx = w.add(&format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let prod = w.add(&format!("PRODUCT('part','part','',(#{pctx}))"));
    let pdf = w.add(&format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let pdc = w.add(&format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"
    ));
    let pd = w.add(&format!("PRODUCT_DEFINITION('design','',#{pdf},#{pdc})"));
    let pds = w.add(&format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));

    let lu = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))");
    let au = w.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let su = w.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let unc = w.add(&format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-6),#{lu},'distance_accuracy_value','')"
    ));
    let ctx = w.add(&format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)\
         GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{su}))REPRESENTATION_CONTEXT('',''))"
    ));

    let origin = w.ax2([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let rep = w.add(&format!("{shape_kind}('',(#{origin},#{shape_id}),#{ctx})"));
    w.add(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));

    let text = format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n\
         FILE_NAME('solid-diff.step','',(''),(''),'solid-diff','solid-diff','');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
        w.body
    );

    StepExport {
        text,
        faces: face_nodes,
        skipped,
    }
}

/// Write a part's best body to a STEP file.
pub fn write_step(graph: &Graph, path: &Path, tol: Option<f64>) -> std::io::Result<StepExport> {
    let ex = export(graph, tol);
    std::fs::write(path, &ex.text)?;
    Ok(ex)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reals_always_carry_a_decimal_point() {
        // STEP parsers reject `1E-7`; the mantissa must contain '.'.
        for (x, want_dot) in [
            (1.0, true),
            (0.5, true),
            (1e-7, true),
            (1e12, true),
            (-3.25e-9, true),
            (std::f64::consts::TAU, true),
        ] {
            let s = fr(x);
            assert!(
                s.contains('.'),
                "{x} formatted as {s:?} which has no decimal point"
            );
            let _ = want_dot;
            // And it must round-trip.
            let back: f64 = s.parse().unwrap();
            assert!(
                (back - x).abs() <= 1e-15 * (1.0 + x.abs()),
                "{x} -> {s} -> {back}"
            );
        }
    }

    #[test]
    fn knot_compression_round_trips() {
        let expanded = [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
        let (k, m) = compress_knots(&expanded);
        assert_eq!(k, vec![0.0, 0.5, 1.0]);
        assert_eq!(m, vec![3, 1, 3]);
        assert_eq!(m.iter().sum::<usize>(), expanded.len());
    }
}
