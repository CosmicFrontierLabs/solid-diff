//! Attribute mesh defects back to the B-rep faces that caused them.
//!
//!   cargo run --release --example diag -- FILE...
//!
//! Reports, per part: which stream the body came from, the surface and curve
//! mix, faces that produced no triangles, open and flipped edges grouped by
//! the surface types either side, and -- most usefully -- faces whose
//! triangles escape their own boundary curves, which is what a trimming
//! failure looks like from the outside.
//!
//! Bound faces by their sampled boundary *curves*, never by vertices alone:
//! ring edges (full circles) carry no vertices, so a vertex-only box
//! under-covers them and reports leaks that are not there.

use std::collections::{HashMap, HashSet};

use solid_diff::value::NodeId;

fn main() {
    for path in std::env::args().skip(1) {
        let p = std::path::Path::new(&path);
        println!("\n=== {}", p.file_name().unwrap().to_string_lossy());

        let bodies = match solid_diff::body_graphs(p) {
            Ok(b) => b,
            Err(e) => {
                println!("  no geometry: {e}");
                continue;
            }
        };
        for (i, b) in bodies.iter().enumerate() {
            println!(
                "  candidate {i}: {:<45} {} faces{}",
                b.stream,
                b.graph.by_type("FACE").len(),
                if i == 0 { "   <-- used" } else { "" }
            );
        }
        let g = &bodies[0].graph;

        // BODY nodes and their shells/regions.
        for b in g.by_type("BODY") {
            println!(
                "  BODY {} type={:?} shells={}",
                b.id,
                b.i64("body_type"),
                g.chain(b.ptr("shell"), "next").len()
            );
        }

        let mut surf_hist: HashMap<String, usize> = HashMap::new();
        for f in g.by_type("FACE") {
            let n = g
                .deref(f, "surface")
                .map(|s| s.name.clone())
                .unwrap_or_else(|| "<none>".into());
            *surf_hist.entry(n).or_default() += 1;
        }
        let mut sv: Vec<_> = surf_hist.into_iter().collect();
        sv.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        println!(
            "  surfaces: {}",
            sv.iter()
                .map(|(k, n)| format!("{k}:{n}"))
                .collect::<Vec<_>>()
                .join(" ")
        );

        let mut curve_hist: HashMap<String, usize> = HashMap::new();
        for e in g.by_type("EDGE") {
            let n = g
                .deref(e, "curve")
                .map(|c| c.name.clone())
                .unwrap_or_else(|| "<none>".into());
            *curve_hist.entry(n).or_default() += 1;
        }
        let mut cv: Vec<_> = curve_hist.into_iter().collect();
        cv.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        println!(
            "  curves:   {}",
            cv.iter()
                .map(|(k, n)| format!("{k}:{n}"))
                .collect::<Vec<_>>()
                .join(" ")
        );

        let mesh = solid_diff::tess::tessellate(g, None);
        println!(
            "  mesh: {} tris, {} verts",
            mesh.triangles.len(),
            mesh.vertices.len()
        );

        // Faces that produced no triangles at all.
        let meshed: HashSet<NodeId> = mesh.face_ids.iter().copied().collect();
        let mut dropped: HashMap<String, Vec<NodeId>> = HashMap::new();
        for f in g.by_type("FACE") {
            if !meshed.contains(&f.id) {
                let sn = g
                    .deref(f, "surface")
                    .map(|s| s.name.clone())
                    .unwrap_or_else(|| "<none>".into());
                dropped.entry(sn).or_default().push(f.id);
            }
        }
        if dropped.is_empty() {
            println!("  dropped faces: none");
        } else {
            for (k, ids) in &dropped {
                println!(
                    "  DROPPED {} face(s) on {k}: {:?}",
                    ids.len(),
                    &ids[..ids.len().min(12)]
                );
            }
        }

        // Attribute open/flipped edges to the surface types either side.
        let surf_of: HashMap<NodeId, String> = g
            .by_type("FACE")
            .iter()
            .map(|f| {
                (
                    f.id,
                    g.deref(f, "surface")
                        .map(|s| s.name.clone())
                        .unwrap_or_else(|| "?".into()),
                )
            })
            .collect();

        let mut users: HashMap<(u32, u32), Vec<(usize, bool)>> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                users.entry(key).or_default().push((ti, a < b));
            }
        }
        let mut open: HashMap<String, usize> = HashMap::new();
        let mut flip: HashMap<String, usize> = HashMap::new();
        for (_, us) in users.iter() {
            let label = |ti: usize| -> String {
                mesh.face_ids
                    .get(ti)
                    .and_then(|f| surf_of.get(f))
                    .cloned()
                    .unwrap_or_else(|| "?".into())
            };
            if us.len() == 1 {
                *open.entry(label(us[0].0)).or_default() += 1;
            } else if us.len() == 2 && us[0].1 == us[1].1 {
                let (mut x, mut y) = (label(us[0].0), label(us[1].0));
                if x > y {
                    std::mem::swap(&mut x, &mut y);
                }
                *flip.entry(format!("{x}|{y}")).or_default() += 1;
            }
        }
        let mut ov: Vec<_> = open.into_iter().collect();
        ov.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        let mut fv: Vec<_> = flip.into_iter().collect();
        fv.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        println!(
            "  open edges by surface:    {}",
            ov.iter()
                .map(|(k, n)| format!("{k}:{n}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
        println!(
            "  flipped edges by pairing: {}",
            fv.iter()
                .take(8)
                .map(|(k, n)| format!("{k}:{n}"))
                .collect::<Vec<_>>()
                .join(" ")
        );

        // Per-face: do the triangles stay inside the face's OWN boundary loops?
        // A face's loops bound it exactly, so triangles reaching well outside
        // that box mean trimming failed and the parametric domain leaked.
        let mut tris_of: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (ti, _) in mesh.triangles.iter().enumerate() {
            if let Some(f) = mesh.face_ids.get(ti) {
                tris_of.entry(*f).or_default().push(ti);
            }
        }
        // True bound of a face: sample the CURVES of its boundary edges, not
        // just their vertices. Ring edges (full circles) carry no vertices at
        // all, so a vertex-only box silently under-covers them.
        let curve_bbox = |ids: &[NodeId]| -> Option<([f64; 3], [f64; 3])> {
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            let mut any = false;
            for eid in ids {
                let Some(edge) = g.get(*eid) else { continue };
                let Some(cn) = g.deref(edge, "curve") else {
                    continue;
                };
                let Some(curve) = solid_diff::geom::curves::make_curve(g, cn) else {
                    continue;
                };
                let ends: Vec<[f64; 3]> = [edge]
                    .iter()
                    .filter_map(|e| g.deref(e, "halfedge"))
                    .flat_map(|h| [Some(h), g.deref(h, "other")])
                    .flatten()
                    .filter_map(|h| {
                        g.deref(h, "vertex")
                            .and_then(|v| g.deref(v, "point"))
                            .and_then(|p| p.vec3("pvec"))
                    })
                    .collect();
                let (t0, t1) = if ends.len() >= 2 {
                    let (a, b) = (curve.inv(ends[0]), curve.inv(ends[1]));
                    if let Some(per) = curve.period() {
                        let fwd = if b >= a { b } else { b + per };
                        let back = if b <= a { b } else { b - per };
                        if (fwd - a).abs() <= (a - back).abs() {
                            (a, fwd)
                        } else {
                            (a, back)
                        }
                    } else {
                        (a, b)
                    }
                } else if let Some((a, b)) = curve.full_range() {
                    (a, b)
                } else {
                    continue;
                };
                for i in 0..=24 {
                    let v = curve.eval(t0 + (t1 - t0) * (i as f64 / 24.0));
                    any = true;
                    for k in 0..3 {
                        lo[k] = lo[k].min(v[k]);
                        hi[k] = hi[k].max(v[k]);
                    }
                }
            }
            any.then_some((lo, hi))
        };

        let mut leaks: Vec<(f64, String)> = Vec::new();
        let mut all_edges: Vec<NodeId> = Vec::new();
        for f in g.by_type("FACE") {
            let mut eids: Vec<NodeId> = Vec::new();
            for lp in g.face_loops(f) {
                for he in g.loop_halfedges(lp) {
                    if let Some(e) = g.deref(he, "edge") {
                        eids.push(e.id);
                    }
                }
            }
            all_edges.extend(eids.iter().copied());
            let Some((blo, bhi)) = curve_bbox(&eids) else {
                continue;
            };
            let span = (0..3).map(|i| bhi[i] - blo[i]).fold(0.0, f64::max);
            if span <= 0.0 {
                continue;
            }
            let slack = span * 0.02;
            let mut worst = 0.0f64;
            for ti in tris_of.get(&f.id).into_iter().flatten() {
                for vi in mesh.triangles[*ti] {
                    let v = mesh.vertices[vi as usize];
                    for i in 0..3 {
                        worst = worst.max(blo[i] - v[i]).max(v[i] - bhi[i]);
                    }
                }
            }
            if worst > slack {
                let sn = g
                    .deref(f, "surface")
                    .map(|s| s.name.clone())
                    .unwrap_or_default();
                leaks.push((
                    worst / span,
                    format!(
                        "face {:<5} {sn:<10} leaks {:>6.2}x its {:.4} m span ({} tris, {} loops)",
                        f.id,
                        worst / span,
                        span,
                        tris_of.get(&f.id).map(|v| v.len()).unwrap_or(0),
                        g.face_loops(f).len(),
                    ),
                ));
            }
        }
        leaks.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        if leaks.is_empty() {
            println!("  no faces leak past their boundary curves");
        } else {
            println!(
                "  LEAKING FACES ({} of {}):",
                leaks.len(),
                g.by_type("FACE").len()
            );
            for (_, l) in leaks.iter().take(10) {
                println!("    {l}");
            }
        }
        if let Some((elo, ehi)) = curve_bbox(&all_edges) {
            println!(
                "  edge-curve bbox {:?}..{:?}",
                elo.map(|v| (v * 1e4).round() / 1e4),
                ehi.map(|v| (v * 1e4).round() / 1e4)
            );
        }

        // Sanity: does the mesh bbox match the POINT-node extent?
        let (lo, hi) = mesh.bounds();
        let mut plo = [f64::MAX; 3];
        let mut phi = [f64::MIN; 3];
        for pt in g.by_type("POINT") {
            if let Some(v) = pt.vec3("pvec") {
                for i in 0..3 {
                    plo[i] = plo[i].min(v[i]);
                    phi[i] = phi[i].max(v[i]);
                }
            }
        }
        println!(
            "  mesh bbox {:?}..{:?}\n  POINT bbox {:?}..{:?}",
            lo.map(|v| (v * 1e3).round() / 1e3),
            hi.map(|v| (v * 1e3).round() / 1e3),
            plo.map(|v| (v * 1e3).round() / 1e3),
            phi.map(|v| (v * 1e3).round() / 1e3),
        );
    }
}
