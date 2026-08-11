//! Read SolidWorks part files (2015+), extract the embedded Parasolid B-rep,
//! tessellate it, and render it.
//!
//! Pipeline (see `docs/FORMAT.md`):
//! ```text
//! .SLDPRT --container--> streams --sections--> XT transmit --xt--> nodes
//!         --graph--> topology --step--> STEP --occt--> Mesh --render--> SVG
//! ```

pub mod anim;
pub mod container;
pub mod diff;
pub mod font;
pub mod geom;
pub mod graph;
pub mod iso;
pub mod mesh;
pub mod occt;
pub mod pdm;
pub mod render;
pub mod sample;
pub mod sections;
pub mod step;
pub mod value;
pub mod xt;

use std::path::Path;

pub use graph::Graph;
pub use mesh::Mesh;

/// Error type for the whole pipeline.
#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    /// Not a SolidWorks 2015+ chunk-container file (e.g. legacy OLE2).
    NotModernSldprt,
    /// No parseable Parasolid transmit containing geometry.
    NoGeometry,
    Parse(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "io error: {e}"),
            Error::NotModernSldprt => write!(f, "not a modern (2015+) SolidWorks file"),
            Error::NoGeometry => write!(f, "no embedded Parasolid geometry found"),
            Error::Parse(m) => write!(f, "parse error: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

/// A Parasolid transmit found inside a part file.
pub struct BodyGraph {
    /// Container stream the transmit was carved from.
    pub stream: String,
    pub graph: Graph,
}

/// Every parseable transmit in a part file that carries geometry, best first.
///
/// Where the solid lives varies by SolidWorks era (`Contents/Config-N-Partition`
/// in older files, `Config-N-FeatureBodies/LocalBodies` in ~2024 ones), so this
/// carves every stream and ranks by face count rather than trusting names.
/// Ghost partitions (reference wire bodies) are skipped.
pub fn body_graphs(path: &Path) -> Result<Vec<BodyGraph>> {
    let data = std::fs::read(path)?;
    let file = container::parse(&data)?;
    let mut out = Vec::new();
    for stream in file.streams() {
        if stream.name.ends_with("-GhostPartition") {
            continue;
        }
        for blob in sections::carve_zlib(&stream.data) {
            let Some(kind) = sections::transmit_kind(&blob) else {
                continue;
            };
            if kind == sections::TransmitKind::Deltas {
                continue;
            }
            if let Ok(nodes) = xt::parse_transmit(&blob) {
                out.push(BodyGraph {
                    stream: stream.name.clone(),
                    graph: Graph::new(nodes),
                });
            }
        }
    }
    out.sort_by_key(|b| std::cmp::Reverse(b.graph.by_type("FACE").len()));
    if out.is_empty() {
        return Err(Error::NoGeometry);
    }
    Ok(out)
}

/// Tessellate a body graph: export the B-rep to STEP and mesh it with
/// OpenCASCADE, whose reader-side shape healing (pcurves, seams, wire
/// orientation) closes trimmed periodic faces properly.
///
/// An empty mesh means the round trip produced nothing renderable for this
/// body; callers already treat an empty mesh as a failed part.
pub fn tessellate(graph: &Graph, tol: Option<f64>) -> Mesh {
    occt::tessellate(graph, tol).unwrap_or_default()
}

pub fn mesh_file(path: &Path, tol: Option<f64>) -> Result<Mesh> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if ext == "x_b" || ext == "xb" {
        let data = std::fs::read(path)?;
        let nodes = xt::parse_transmit(&data).map_err(|e| Error::Parse(e.to_string()))?;
        return Ok(tessellate(&Graph::new(nodes), tol));
    }
    let graphs = body_graphs(path)?;
    // A part is often several bodies (a connector: shell, coupling nut,
    // contacts), and rendering only the largest loses the rest. But the file
    // also carries OTHER streams -- sibling configurations (a sheet-metal
    // part's flat pattern) and ResolvedFeatures copies -- and merging those
    // superimposes the bent part with its unbent blank. Sibling bodies of one
    // configuration live in one stream, so merge exactly the solid bodies
    // that share the best body's stream and nothing else.
    let mut out = Mesh::default();
    for bg in &graphs {
        if bg.stream != graphs[0].stream {
            continue;
        }
        let solid = bg
            .graph
            .by_type("BODY")
            .first()
            .and_then(|b| b.i64("body_type"))
            .map(|t| t == 1)
            .unwrap_or(true);
        if !solid {
            continue;
        }
        let m = tessellate(&bg.graph, tol);
        if !m.is_empty() {
            out.merge(m);
        }
    }
    if out.is_empty() {
        out = tessellate(&graphs[0].graph, tol);
    }
    Ok(out)
}
