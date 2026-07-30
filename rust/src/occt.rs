//! Tessellation by OCCT, via STEP.
//!
//! The pipeline is SLDPRT -> Parasolid graph (ours) -> STEP (`step.rs`) ->
//! OCCT import -> `BRepMesh` -> our `Mesh`. OCCT's STEP reader runs shape
//! healing on the way in -- pcurve computation, seam insertion, wire
//! orientation -- which is the machinery a trimmed periodic face needs and
//! that we spent a long time approximating by hand.
//!
//! Face identity does not survive the trip structurally (STEP transfer can
//! reorder and split faces), so it is recovered geometrically: each OCCT face
//! is matched to the Parasolid FACE whose surface it actually lies on. That
//! keeps `diff` colouring working unchanged.

use std::io::Write;

use opencascade_sys::ffi;

use crate::geom::{dist, surfaces::make_surface, P3};
use crate::graph::Graph;
use crate::mesh::Mesh;
use crate::value::NodeId;

/// Linear deflection as a fraction of part size. OCCT's angular default
/// (0.5 rad) rides along with it.
const DEFLECTION_FRAC: f64 = 5e-4;

/// OCCT's STEP reader keeps global state (`Interface_Static`, the message
/// system), and concurrent readers abort with "terminate called recursively".
/// One process, one transfer at a time. The CLI parallelizes across
/// *processes*, so this costs nothing there; it exists for multithreaded
/// callers like the test harness.
static OCCT_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Tessellate via OCCT. Returns `None` when the round trip produces nothing
/// renderable for this body.
pub fn tessellate(graph: &Graph, tol: Option<f64>) -> Option<Mesh> {
    let _serial = OCCT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let scale = graph.model_scale();
    let export = crate::step::export(graph, tol);
    if export.faces.is_empty() {
        return None;
    }

    // The reader wants a path. Everything else is in-process.
    let dir = std::env::temp_dir();
    let path = dir.join(format!(
        "solid-diff-{}-{:x}.step",
        std::process::id(),
        export.text.len()
    ));
    {
        let mut f = std::fs::File::create(&path).ok()?;
        f.write_all(export.text.as_bytes()).ok()?;
    }
    let result = mesh_step_file(&path, graph, scale, tol);
    let _ = std::fs::remove_file(&path);
    result
}

fn mesh_step_file(
    path: &std::path::Path,
    graph: &Graph,
    scale: f64,
    tol: Option<f64>,
) -> Option<Mesh> {
    let debug = std::env::var_os("SD_OCCT_DEBUG").is_some();
    let mut reader = ffi::STEPControl_Reader_ctor();
    let status = ffi::read_step(reader.pin_mut(), path.to_string_lossy().to_string());
    if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
        return None;
    }
    let progress = ffi::Message_ProgressRange_ctor();
    let n = reader.pin_mut().TransferRoots(&progress);
    if n <= 0 {
        return None;
    }
    let shape = ffi::one_shape(&reader);

    // The reader has converted to millimetres; our deflection is stated in
    // model units (metres), so scale it.
    let defl_mm = (tol.unwrap_or(DEFLECTION_FRAC * scale) * 1000.0).max(1e-4);
    let mesher = ffi::BRepMesh_IncrementalMesh_ctor(&shape, defl_mm);
    if !mesher.IsDone() {
        return None;
    }

    // Everything the matcher needs about our faces, once.
    let matcher = FaceMatcher::new(graph);

    let mut mesh = Mesh::default();
    // OCCT triangulations are stored per face, each with its own node array;
    // the nodes along a shared edge are computed once per EDGE and injected
    // into both faces bit-identically. Welding by exact bits therefore closes
    // the shell without any tolerance guesswork.
    let mut weld: std::collections::HashMap<[u64; 3], u32> = std::collections::HashMap::new();
    let mut ex = ffi::TopExp_Explorer_ctor(mesher.Shape(), ffi::TopAbs_ShapeEnum::TopAbs_FACE);
    while ex.More() {
        let face_shape = ffi::ExplorerCurrentShape(&ex);
        let face = ffi::TopoDS_cast_to_face(&face_shape);
        let mut location = ffi::TopLoc_Location_ctor();
        let handle = ffi::BRep_Tool_Triangulation(face, location.pin_mut());
        let reversed = face_shape.Orientation() == ffi::TopAbs_Orientation::TopAbs_REVERSED;

        if let Ok(tri) = ffi::Handle_Poly_Triangulation_Get(&handle) {
            let trsf = ffi::TopLoc_Location_Transformation(&location);
            let n_nodes = tri.NbNodes();
            let mut local: Vec<u32> = Vec::with_capacity(n_nodes as usize);
            let mut probe_acc = [0.0f64; 3];
            for i in 1..=n_nodes {
                let mut p = ffi::Poly_Triangulation_Node(tri, i);
                p.pin_mut().Transform(&trsf);
                // Back to metres.
                let v = [p.X() / 1000.0, p.Y() / 1000.0, p.Z() / 1000.0];
                if i <= 16 {
                    for d in 0..3 {
                        probe_acc[d] += v[d];
                    }
                }
                let key = [v[0].to_bits(), v[1].to_bits(), v[2].to_bits()];
                let idx = *weld.entry(key).or_insert_with(|| {
                    mesh.vertices.push(v);
                    (mesh.vertices.len() - 1) as u32
                });
                local.push(idx);
            }
            // One sample point identifies which of our faces this is.
            let take = n_nodes.clamp(1, 16) as f64;
            let face_id = matcher.identify([
                probe_acc[0] / take,
                probe_acc[1] / take,
                probe_acc[2] / take,
            ]);

            let n_tris = tri.NbTriangles();
            for i in 1..=n_tris {
                let t = tri.Triangle(i);
                let (a, b, c) = (t.Value(1), t.Value(2), t.Value(3));
                if a == b || b == c || a == c {
                    continue;
                }
                let (a, b, c) = if reversed { (a, c, b) } else { (a, b, c) };
                let t = [
                    local[a as usize - 1],
                    local[b as usize - 1],
                    local[c as usize - 1],
                ];
                if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                    continue; // collapsed by welding: degenerate sliver
                }
                mesh.triangles.push(t);
                mesh.face_ids.push(face_id);
            }
        }
        ex.pin_mut().Next();
    }

    if debug {
        let mut n_faces = 0;
        let mut ex2 = ffi::TopExp_Explorer_ctor(mesher.Shape(), ffi::TopAbs_ShapeEnum::TopAbs_FACE);
        while ex2.More() {
            n_faces += 1;
            ex2.pin_mut().Next();
        }
        eprintln!(
            "occt: {} faces in shape, {} triangles, {} verts after weld",
            n_faces,
            mesh.triangles.len(),
            mesh.vertices.len()
        );
    }
    if mesh.triangles.is_empty() {
        return None;
    }
    // OCCT's orientation plus our reversed-face flip is usually consistent
    // already; make it exactly so, the same pass the native path runs.
    mesh.orient();
    for face in graph.by_type("FACE") {
        if let Some(c) = graph.face_color(face) {
            mesh.colors.insert(face.id, c);
        }
    }
    Some(mesh)
}

/// Matches a point in space to the Parasolid FACE it lies on.
///
/// Candidates are prefiltered by the bounding box of each face's boundary
/// samples (inflated), then ranked by true distance to the face's surface.
/// The surfaces are exact, so a triangulation vertex sits within meshing
/// deflection of the right one and usually far from every other.
struct FaceMatcher {
    faces: Vec<FaceEntry>,
    fallback: NodeId,
}

struct FaceEntry {
    id: NodeId,
    lo: P3,
    hi: P3,
    surf: Box<dyn crate::geom::Surface>,
}

impl FaceMatcher {
    fn new(graph: &Graph) -> Self {
        let scale = graph.model_scale();
        let slack = scale * 0.02;
        let mut sampler = crate::sample::EdgeSampler::new(scale * 2e-3);
        let mut faces = Vec::new();
        let mut fallback = 0;
        for face in graph.by_type("FACE") {
            fallback = face.id;
            let Some(snode) = graph.deref(face, "surface") else {
                continue;
            };
            let Some(surf) = make_surface(graph, snode) else {
                continue;
            };
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let mut any = false;
            for lp in graph.face_loops(face) {
                for he in graph.loop_halfedges(lp) {
                    let Some(edge) = graph.deref(he, "edge") else {
                        continue;
                    };
                    let Some(pts) = sampler.get(graph, edge) else {
                        continue;
                    };
                    for p in pts {
                        any = true;
                        for d in 0..3 {
                            lo[d] = lo[d].min(p[d]);
                            hi[d] = hi[d].max(p[d]);
                        }
                    }
                }
            }
            if !any {
                // A face with no sampled boundary (whole sphere/torus): accept
                // any point, ranked purely by surface distance.
                lo = [f64::NEG_INFINITY; 3];
                hi = [f64::INFINITY; 3];
            } else {
                for d in 0..3 {
                    lo[d] -= slack;
                    hi[d] += slack;
                }
            }
            faces.push(FaceEntry {
                id: face.id,
                lo,
                hi,
                surf,
            });
        }
        FaceMatcher { faces, fallback }
    }

    fn identify(&self, p: P3) -> NodeId {
        let mut best = (f64::INFINITY, self.fallback);
        for f in &self.faces {
            if (0..3).any(|d| p[d] < f.lo[d] || p[d] > f.hi[d]) {
                continue;
            }
            let d = dist(f.surf.eval(f.surf.inv(p)), p);
            if d < best.0 {
                best = (d, f.id);
            }
        }
        best.1
    }
}
