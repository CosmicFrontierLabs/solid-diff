//! Invariants the part files assert about themselves.
//!
//! These tests need no recorded reference output. Every assertion is either a
//! combinatorial identity that any valid B-rep must satisfy, or a redundancy
//! the file already contains -- Parasolid stores a vertex position *and* the
//! curve it sits on, and stores a curve *and* the two surfaces it bounds, so
//! the geometry has to agree with itself. A decoding or evaluation bug shows
//! up as a disagreement.
//!
//! The point of doing it this way: a frozen snapshot of some earlier
//! implementation's output only tells you that behaviour changed, and happily
//! preserves that implementation's mistakes forever. These checks can tell you
//! the answer is *wrong*.
//!
//! Corpus files are optional (samples/*.SLDPRT are fetched, not committed);
//! tests skip themselves when nothing is on disk.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use solid_diff::geom::{curves::make_curve, dist, surfaces::make_surface, Surface};
use solid_diff::graph::Graph;
use solid_diff::value::{Node, NodeId};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn corpus() -> Vec<PathBuf> {
    let root = repo_root();
    let mut out = Vec::new();
    for dir in ["samples", "vault"] {
        let Ok(entries) = std::fs::read_dir(root.join(dir)) else {
            continue;
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().map(|e| e == "SLDPRT").unwrap_or(false))
            .collect();
        files.sort();
        out.extend(files);
    }
    out
}

/// Best body graph per corpus part, skipping files with no readable geometry
/// (pre-2015 OLE2 containers, and the deltas-only part in #6).
fn graphs() -> Vec<(String, Graph)> {
    corpus()
        .into_iter()
        .filter_map(|p| {
            let name = p.file_name()?.to_string_lossy().to_string();
            let mut bodies = solid_diff::body_graphs(&p).ok()?;
            if bodies.is_empty() {
                return None;
            }
            Some((name, bodies.remove(0).graph))
        })
        .collect()
}

/// Absolute tolerance for "these two independently stored things describe the
/// same point", scaled to the part. Parasolid's own linear resolution is
/// typically 1e-8 m; we allow considerably more slack because our inverses are
/// iterative.
fn tol_for(graph: &Graph) -> f64 {
    (graph.model_scale() * 1e-4).max(1e-9)
}

fn vertex_pos(graph: &Graph, holder: &Node, field: &str) -> Option<[f64; 3]> {
    let v = graph.deref(holder, field)?;
    graph.deref(v, "point")?.vec3("pvec")
}

/// The two halfedges of an edge, positive-sense first.
fn edge_halfedges<'a>(graph: &'a Graph, edge: &'a Node) -> Option<(&'a Node, &'a Node)> {
    let h = graph.deref(edge, "halfedge")?;
    let o = graph.deref(h, "other")?;
    if h.sense_positive() {
        Some((h, o))
    } else {
        Some((o, h))
    }
}

/// The face a halfedge belongs to, via its loop.
fn halfedge_face<'a>(graph: &'a Graph, he: &'a Node) -> Option<&'a Node> {
    graph.deref(graph.deref(he, "loop")?, "face")
}

fn face_surface(graph: &Graph, face: &Node) -> Option<Box<dyn Surface>> {
    make_surface(graph, graph.deref(face, "surface")?)
}

/// Curves whose evaluation is closed-form, so the only error is floating point.
const EXACT_CURVES: &[&str] = &["LINE", "CIRCLE", "ELLIPSE", "B_CURVE"];

/// Surfaces whose evaluation is closed-form. `BLENDED_EDGE` is absent
/// deliberately: our rolling-ball reconstruction is a model of the blend, not
/// the blend itself, and the corpus shows it does not hold the boundary (#4).
const EXACT_SURFACES: &[&str] = &[
    "PLANE",
    "CYLINDER",
    "CONE",
    "SPHERE",
    "TORUS",
    "SWEPT_SURF",
    "SPUN_SURF",
    "B_SURFACE",
    "OFFSET_SURF",
];

/// `INTERSECTION` curves carry no closed form: they are interpolated from the
/// sample points their CHART happens to store, so they miss the surfaces they
/// are supposed to lie on by whatever the sampling density allows (#23).
fn is_exact(curve: &str, surface: &str) -> bool {
    EXACT_CURVES.contains(&curve) && EXACT_SURFACES.contains(&surface)
}

/// Report helper: counts a pass/fail population and formats a failure rate.
#[derive(Default)]
struct Tally {
    checked: usize,
    failed: usize,
    worst: f64,
    worst_where: String,
}

impl Tally {
    fn record(&mut self, err: f64, tol: f64, ctx: impl Fn() -> String) {
        self.checked += 1;
        if err > self.worst {
            self.worst = err;
            self.worst_where = ctx();
        }
        if !err.is_finite() || err > tol {
            self.failed += 1;
        }
    }
    fn summary(&self, what: &str) -> String {
        format!(
            "{what}: {}/{} failed ({:.3}%), worst {:.3e} at {}",
            self.failed,
            self.checked,
            100.0 * self.failed as f64 / self.checked.max(1) as f64,
            self.worst,
            self.worst_where
        )
    }
}

// ── Topology: exact combinatorial identities, no tolerance ──────────────────

/// Halfedge pairing is an involution, the pair straddles one edge, and the two
/// senses are opposite. This is what makes "two triangles per edge" meaningful
/// downstream; if it did not hold, watertightness would be unreachable no
/// matter how good the tessellation was.
#[test]
fn halfedge_pairing_is_an_involution() {
    let graphs = graphs();
    if graphs.is_empty() {
        return;
    }
    let mut checked = 0usize;
    let mut problems: Vec<String> = Vec::new();

    for (name, g) in &graphs {
        for he in g.by_type("HALFEDGE") {
            if g.deref(he, "edge").is_none() {
                // A vertex loop: a loop that is a single point rather than a
                // circuit of edges (Parasolid uses these at cone apexes and
                // other degenerate spots). It has one halfedge, no edge, and
                // correctly no partner.
                continue;
            }
            let Some(other) = g.deref(he, "other") else {
                problems.push(format!("{name}: halfedge {} has no partner", he.id));
                continue;
            };
            checked += 1;
            match g.deref(other, "other") {
                Some(back) if back.id == he.id => {}
                Some(back) => problems.push(format!(
                    "{name}: other(other({})) = {}, not {}",
                    he.id, back.id, he.id
                )),
                None => problems.push(format!("{name}: other({}) has no partner", other.id)),
            }
            if he.sense_positive() == other.sense_positive() {
                problems.push(format!(
                    "{name}: halfedges {}/{} share sense {}",
                    he.id,
                    other.id,
                    he.sense_positive()
                ));
            }
            if let (Some(a), Some(b)) = (g.deref(he, "edge"), g.deref(other, "edge")) {
                if a.id != b.id {
                    problems.push(format!(
                        "{name}: halfedges {}/{} straddle different edges {} vs {}",
                        he.id, other.id, a.id, b.id
                    ));
                }
            }
        }
    }
    assert!(checked > 0, "no halfedges found in corpus");
    assert!(
        problems.is_empty(),
        "{} halfedge pairing violations of {checked} checked:\n{}",
        problems.len(),
        problems
            .iter()
            .take(10)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
    eprintln!(
        "halfedge pairing: {checked} halfedges across {} parts",
        graphs.len()
    );
}

/// Loops close when chained through `backward`, and the cycle covers exactly
/// the halfedges that name the loop as their owner.
///
/// This pins a convention that is easy to get backwards and was originally
/// wrong here: `forward` walks the chain in reverse. A comment saying so is
/// not enforcement; this is.
#[test]
fn loops_close_through_the_backward_link() {
    let graphs = graphs();
    if graphs.is_empty() {
        return;
    }
    let mut checked = 0usize;
    let mut problems: Vec<String> = Vec::new();

    for (name, g) in &graphs {
        // Halfedges grouped by the loop they claim to belong to.
        let mut owned: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
        for he in g.by_type("HALFEDGE") {
            if let Some(lp) = g.deref(he, "loop") {
                owned.entry(lp.id).or_default().push(he.id);
            }
        }

        for lp in g.by_type("LOOP") {
            let Some(start) = g.deref(lp, "halfedge") else {
                continue;
            };
            checked += 1;

            let mut seen = Vec::new();
            let mut cur = start;
            loop {
                if seen.contains(&cur.id) {
                    break;
                }
                seen.push(cur.id);
                match g.deref(cur, "backward") {
                    Some(n) => cur = n,
                    None => break,
                }
                if seen.len() > 100_000 {
                    break;
                }
            }
            if cur.id != start.id {
                problems.push(format!(
                    "{name}: loop {} does not close via backward ({} steps, ended at {})",
                    lp.id,
                    seen.len(),
                    cur.id
                ));
                continue;
            }
            let mut want = owned.get(&lp.id).cloned().unwrap_or_default();
            let mut got = seen.clone();
            want.sort();
            got.sort();
            if !want.is_empty() && want != got {
                problems.push(format!(
                    "{name}: loop {} chain visits {} halfedges but {} name it as owner",
                    lp.id,
                    got.len(),
                    want.len()
                ));
            }
        }
    }
    assert!(checked > 0, "no loops found in corpus");
    assert!(
        problems.is_empty(),
        "{} loop-chain violations of {checked} checked:\n{}",
        problems.len(),
        problems
            .iter()
            .take(10)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
    eprintln!("loop chains: {checked} loops across {} parts", graphs.len());
}

// ── Geometry: the file's own redundancy ─────────────────────────────────────

/// Closed-form geometry must agree with the file to floating-point slop.
///
/// Two independent checks, both drawing on redundancy Parasolid already
/// stores:
///
/// * a VERTEX carries explicit coordinates *and* the EDGE it bounds carries a
///   curve -- the point must lie on the curve;
/// * an EDGE's curve is shared by the two FACEs it separates -- points along
///   it must lie on both surfaces.
///
/// Nothing but correct decoding and correct evaluation makes those agree, and
/// no reference output is involved. Restricted here to curve/surface types we
/// evaluate in closed form, where the tolerance can be tight and the expected
/// failure count is flatly zero. Approximate types are covered by the ratchet
/// below.
#[test]
fn closed_form_geometry_agrees_with_the_file() {
    let graphs = graphs();
    if graphs.is_empty() {
        return;
    }
    let mut on_curve = Tally::default();
    let mut on_surface = Tally::default();
    let mut problems: Vec<String> = Vec::new();

    for (name, g) in &graphs {
        let tol = tol_for(g);
        let mut cache: HashMap<NodeId, Option<Box<dyn Surface>>> = HashMap::new();

        for edge in g.by_type("EDGE") {
            let Some(cn) = g.deref(edge, "curve") else {
                continue;
            };
            let Some(curve) = make_curve(g, cn) else {
                continue; // unimplemented curve type (#3 SP_CURVE)
            };
            let Some((hp, hm)) = edge_halfedges(g, edge) else {
                continue;
            };
            let (Some(a), Some(b)) = (vertex_pos(g, hp, "vertex"), vertex_pos(g, hm, "vertex"))
            else {
                continue;
            };

            // Vertex positions must sit on the edge's curve.
            if EXACT_CURVES.contains(&cn.name.as_str()) {
                for (p, which) in [(a, "start"), (b, "end")] {
                    let err = dist(curve.eval(curve.inv(p)), p);
                    on_curve.record(err, tol, || format!("{name} edge {}", edge.id));
                    if !err.is_finite() || err > tol {
                        problems.push(format!(
                            "{name}: edge {} {which} vertex is {err:.3e} off its {} \
                             (tol {tol:.1e})",
                            edge.id, cn.name
                        ));
                    }
                }
            }

            // Interior points of the curve must sit on both adjacent surfaces.
            // Endpoints are excluded: they are already covered above, and the
            // interior is constrained only by the surfaces.
            let (t0, t1) = (curve.inv(a), curve.inv(b));
            let pts: Vec<[f64; 3]> = (1..4)
                .map(|i| curve.eval(t0 + (t1 - t0) * (i as f64 / 4.0)))
                .collect();

            for he in [hp, hm] {
                let Some(face) = halfedge_face(g, he) else {
                    continue;
                };
                let Some(sn) = g.deref(face, "surface") else {
                    continue;
                };
                if !is_exact(&cn.name, &sn.name) {
                    continue;
                }
                let surf = cache
                    .entry(sn.id)
                    .or_insert_with(|| make_surface(g, sn))
                    .as_ref();
                let Some(surf) = surf else { continue };

                for p in &pts {
                    let err = dist(surf.eval(surf.inv(*p)), *p);
                    on_surface.record(err, tol, || {
                        format!("{name} edge {} on {}", edge.id, sn.name)
                    });
                    if !err.is_finite() || err > tol {
                        problems.push(format!(
                            "{name}: edge {} ({}) is {err:.3e} off the {} of face {} \
                             (tol {tol:.1e})",
                            edge.id, cn.name, sn.name, face.id
                        ));
                    }
                }
            }
        }
    }

    eprintln!("{}", on_curve.summary("vertex-on-curve   [exact]"));
    eprintln!("{}", on_surface.summary("curve-on-surface  [exact]"));
    assert!(
        on_curve.checked > 0 && on_surface.checked > 0,
        "corpus produced no checks"
    );
    assert!(
        problems.is_empty(),
        "{} closed-form geometry disagreements ({} vertex + {} surface checks):\n{}",
        problems.len(),
        on_curve.checked,
        on_surface.checked,
        problems
            .iter()
            .take(15)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Budget for geometry we only approximate. These numbers are a record of
/// known defects, not a target: they may only ever be lowered. A rise means a
/// regression; a drop means the comment below is stale and the budget should
/// be tightened in the same commit that earned it.
///
/// * `INTERSECTION` curves are interpolated from stored CHART samples (#23).
/// * `BLENDED_EDGE` fails even for exact CIRCLE boundaries, which says the
///   rolling-ball reconstruction is wrong rather than merely coarse (#4).
const APPROX_BUDGET: &[(&str, usize)] = &[("INTERSECTION", 96), ("BLENDED_EDGE", 32)];

#[test]
fn approximate_geometry_stays_within_budget() {
    let graphs = graphs();
    if graphs.is_empty() {
        return;
    }
    let mut fails: HashMap<&str, usize> = HashMap::new();
    let mut totals: HashMap<&str, usize> = HashMap::new();
    let mut worst = 0.0f64;

    for (name, g) in &graphs {
        let _ = name;
        let tol = tol_for(g);
        let mut cache: HashMap<NodeId, Option<Box<dyn Surface>>> = HashMap::new();

        for edge in g.by_type("EDGE") {
            let Some(cn) = g.deref(edge, "curve") else {
                continue;
            };
            let Some(curve) = make_curve(g, cn) else {
                continue;
            };
            let Some((hp, hm)) = edge_halfedges(g, edge) else {
                continue;
            };
            let (Some(a), Some(b)) = (vertex_pos(g, hp, "vertex"), vertex_pos(g, hm, "vertex"))
            else {
                continue;
            };

            if !EXACT_CURVES.contains(&cn.name.as_str()) {
                for p in [a, b] {
                    let err = dist(curve.eval(curve.inv(p)), p);
                    *totals.entry("INTERSECTION").or_default() += 1;
                    if !err.is_finite() || err > tol {
                        *fails.entry("INTERSECTION").or_default() += 1;
                        worst = worst.max(err / g.model_scale());
                    }
                }
            }

            let (t0, t1) = (curve.inv(a), curve.inv(b));
            for he in [hp, hm] {
                let Some(face) = halfedge_face(g, he) else {
                    continue;
                };
                let Some(sn) = g.deref(face, "surface") else {
                    continue;
                };
                if sn.name != "BLENDED_EDGE" {
                    continue;
                }
                let surf = cache
                    .entry(sn.id)
                    .or_insert_with(|| make_surface(g, sn))
                    .as_ref();
                let Some(surf) = surf else { continue };
                for i in 1..4 {
                    let p = curve.eval(t0 + (t1 - t0) * (i as f64 / 4.0));
                    let err = dist(surf.eval(surf.inv(p)), p);
                    *totals.entry("BLENDED_EDGE").or_default() += 1;
                    if !err.is_finite() || err > tol {
                        *fails.entry("BLENDED_EDGE").or_default() += 1;
                    }
                }
            }
        }
    }

    let mut over = Vec::new();
    for (kind, budget) in APPROX_BUDGET {
        let got = fails.get(kind).copied().unwrap_or(0);
        let total = totals.get(kind).copied().unwrap_or(0);
        eprintln!("{kind:<14} {got:>4}/{total:<6} off-surface (budget {budget})");
        if got > *budget {
            over.push(format!("{kind}: {got} failures, budget {budget}"));
        }
    }
    eprintln!("worst approximate error: {worst:.2e} x model scale");
    assert!(
        over.is_empty(),
        "approximate geometry got worse:\n{}\n\
         If this is expected, say why; otherwise it is a regression.",
        over.join("\n")
    );
}

// ── Mesh contract ───────────────────────────────────────────────────────────

/// Meshing is a pure function of the file. Any run-to-run variation means
/// something is iterating a hash map or racing, and would make every other
/// measurement here unreproducible.
#[test]
fn meshing_is_deterministic() {
    let parts: Vec<PathBuf> = corpus().into_iter().take(6).collect();
    if parts.is_empty() {
        return;
    }
    for p in &parts {
        let Ok(a) = solid_diff::mesh_file(p, None) else {
            continue;
        };
        let b = solid_diff::mesh_file(p, None).expect("second mesh of the same file failed");
        assert_eq!(a.vertices.len(), b.vertices.len(), "{p:?}: vertex count");
        assert_eq!(
            a.triangles.len(),
            b.triangles.len(),
            "{p:?}: triangle count"
        );
        for (i, (x, y)) in a.vertices.iter().zip(b.vertices.iter()).enumerate() {
            assert_eq!(
                x.map(f64::to_bits),
                y.map(f64::to_bits),
                "{p:?}: vertex {i}"
            );
        }
        assert_eq!(a.triangles, b.triangles, "{p:?}: triangles");
    }
}

/// Mesh vertices must lie on the B-rep surface they were generated from.
///
/// Catches the planar fallback silently standing in for a real evaluator, and
/// any UV mapping that is self-consistent but wrong.
#[test]
fn mesh_vertices_lie_on_their_source_surface() {
    let parts: Vec<PathBuf> = corpus().into_iter().take(8).collect();
    if parts.is_empty() {
        return;
    }
    let mut t = Tally::default();

    for p in &parts {
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        let Ok(mut bodies) = solid_diff::body_graphs(p) else {
            continue;
        };
        if bodies.is_empty() {
            continue;
        }
        let g = bodies.remove(0).graph;
        let mesh = solid_diff::tess::tessellate(&g, None);
        if mesh.is_empty() {
            continue;
        }
        let tol = g.model_scale() * 1e-2;

        let mut cache: HashMap<NodeId, Option<Box<dyn Surface>>> = HashMap::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let Some(fid) = mesh.face_ids.get(ti).copied() else {
                continue;
            };
            let surf = cache.entry(fid).or_insert_with(|| {
                g.by_type("FACE")
                    .into_iter()
                    .find(|f| f.id == fid)
                    .and_then(|f| face_surface(&g, f))
            });
            let Some(surf) = surf.as_ref() else { continue };
            for vi in tri {
                let v = mesh.vertices[*vi as usize];
                t.record(dist(surf.eval(surf.inv(v)), v), tol, || {
                    format!("{name} face {fid}")
                });
            }
        }
    }
    eprintln!("{}", t.summary("mesh-vertex-on-surface"));
    assert!(t.checked > 0, "no meshed triangles to check");
    assert_eq!(
        t.failed,
        0,
        "{} mesh vertices are further than 1% of model scale from the surface \
         they were generated on -- the planar fallback is standing in for a real \
         evaluator, or a UV mapping is self-consistent but wrong. {}",
        t.failed,
        t.summary("mesh-vertex-on-surface")
    );
}

/// Whole-corpus mesh quality, as a ratchet.
///
/// The directed-edge manifold report classifies every undirected edge exactly
/// once, so these counts do not overlap. They are budgets of known-bad, not
/// goals: they may only be lowered. This is deliberately not a snapshot of
/// per-part output -- that would fight every legitimate improvement, since
/// fixing a trimming bug changes triangle counts everywhere.
const MESH_BUDGET: &[(&str, usize)] = &[
    ("holes", 4201),
    ("winding mismatches", 10468),
    ("non-manifold", 98),
    ("parts that fail to mesh", 3),
];

#[test]
fn corpus_mesh_quality_does_not_regress() {
    let parts = corpus();
    if parts.is_empty() {
        return;
    }
    let (mut holes, mut flips, mut nonmani, mut failed, mut tris) = (0, 0, 0, 0, 0);

    for p in &parts {
        match solid_diff::mesh_file(p, None) {
            Ok(m) if !m.is_empty() => {
                let r = m.manifold_report();
                holes += r.boundary;
                flips += r.flipped;
                nonmani += r.non_manifold;
                tris += m.triangles.len();
            }
            _ => failed += 1,
        }
    }
    let got = [
        ("holes", holes),
        ("winding mismatches", flips),
        ("non-manifold", nonmani),
        ("parts that fail to mesh", failed),
    ];
    eprintln!("corpus: {} parts, {tris} triangles", parts.len());
    let mut over = Vec::new();
    for ((name, n), (_, budget)) in got.iter().zip(MESH_BUDGET) {
        eprintln!("  {name:<24} {n:>7}  (budget {budget})");
        if n > budget {
            over.push(format!("{name}: {n} > budget {budget}"));
        }
    }
    assert!(
        over.is_empty(),
        "corpus mesh quality regressed:\n{}\n\
         Lower the budget in the same commit if you improved it.",
        over.join("\n")
    );
}
