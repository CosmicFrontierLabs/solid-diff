//! solid-diff CLI: inspect, extract, mesh and render SolidWorks part files.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand, ValueEnum};

use solid_diff::render::{render_mesh_svg, svg_document, Order, RenderOptions};
use solid_diff::{body_graphs, container, mesh_file, sections, tess, xt, Graph};

#[derive(Parser)]
#[command(
    name = "solid-diff",
    about = "Read, mesh and render SolidWorks part files"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Report container streams and embedded Parasolid data.
    Scan {
        files: Vec<PathBuf>,
        /// Also list every stream, with sizes.
        #[arg(long)]
        streams: bool,
    },
    /// Extract embedded Parasolid transmits as .x_b files.
    Extract {
        files: Vec<PathBuf>,
        #[arg(short, long, default_value = ".")]
        outdir: PathBuf,
    },
    /// Tessellate a part to OBJ (and optionally STL).
    Mesh {
        file: PathBuf,
        #[arg(short, long)]
        obj: Option<PathBuf>,
        #[arg(long)]
        stl: Option<PathBuf>,
        /// Chordal tolerance in model units (default: 0.2% of part size).
        #[arg(long)]
        tol: Option<f64>,
        /// Print mesh quality statistics.
        #[arg(long)]
        stats: bool,
        #[arg(short, long)]
        quiet: bool,
    },
    /// Render parts to an SVG contact sheet.
    Render {
        files: Vec<PathBuf>,
        #[arg(short, long, default_value = "render.svg")]
        out: PathBuf,
        #[arg(long, default_value_t = 0.55)]
        alpha: f64,
        #[arg(long, default_value_t = 28.0)]
        elev: f64,
        #[arg(long, default_value_t = -55.0)]
        azim: f64,
        /// Perspective field of view in degrees (default: orthographic).
        #[arg(long)]
        fov: Option<f64>,
        #[arg(long, default_value_t = 520.0)]
        size: f64,
        #[arg(long)]
        cols: Option<usize>,
        #[arg(long, value_enum, default_value_t = OrderArg::Auto)]
        order: OrderArg,
        #[arg(long)]
        no_edges: bool,
        #[arg(long)]
        tol: Option<f64>,
    },
}

#[derive(Copy, Clone, PartialEq, Eq, ValueEnum)]
enum OrderArg {
    Auto,
    Bsp,
    Depth,
}

impl From<OrderArg> for Order {
    fn from(o: OrderArg) -> Order {
        match o {
            OrderArg::Auto => Order::Auto,
            OrderArg::Bsp => Order::Bsp,
            OrderArg::Depth => Order::Depth,
        }
    }
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Scan { files, streams } => cmd_scan(&files, streams),
        Cmd::Extract { files, outdir } => cmd_extract(&files, &outdir),
        Cmd::Mesh {
            file,
            obj,
            stl,
            tol,
            stats,
            quiet,
        } => cmd_mesh(&file, obj.as_deref(), stl.as_deref(), tol, stats, quiet),
        Cmd::Render {
            files,
            out,
            alpha,
            elev,
            azim,
            fov,
            size,
            cols,
            order,
            no_edges,
            tol,
        } => {
            let opts = RenderOptions {
                alpha,
                elev,
                azim,
                fov,
                size,
                title: None,
                order: order.into(),
                edges: !no_edges,
                color_map: HashMap::new(),
            };
            cmd_render(&files, &out, &opts, cols, tol)
        }
    }
}

fn cmd_scan(files: &[PathBuf], list_streams: bool) -> ExitCode {
    let mut any = false;
    for path in files {
        println!("\n=== {}", path.display());
        let data = match std::fs::read(path) {
            Ok(d) => d,
            Err(e) => {
                println!("  read error: {e}");
                continue;
            }
        };
        let file = match container::parse(&data) {
            Ok(f) => f,
            Err(e) => {
                println!("  {e}");
                continue;
            }
        };
        let streams = file.streams();
        println!(
            "ROL key: 0x{:02x}; {} chunks, {} named streams",
            file.rol_key,
            file.chunks.len(),
            streams.len()
        );
        if let Some(v) = streams.iter().find(|s| s.name.starts_with("_MO_VERSION_")) {
            println!("version stream: {}", v.name);
        }
        if list_streams {
            let mut sorted: Vec<_> = streams.iter().collect();
            sorted.sort_by_key(|s| std::cmp::Reverse(s.data.len()));
            for s in sorted {
                println!("  {:52} {:>9}", s.name, s.data.len());
            }
        }
        for s in &streams {
            for blob in sections::carve_zlib(&s.data) {
                if let Some(kind) = sections::transmit_kind(&blob) {
                    any = true;
                    let nodes = xt::parse_transmit(&blob).map(|n| n.len()).unwrap_or(0);
                    let faces = xt::parse_transmit(&blob)
                        .map(|n| Graph::new(n).by_type("FACE").len())
                        .unwrap_or(0);
                    println!(
                        "  parasolid {:?} in {}: {} bytes, {} nodes, {} faces",
                        kind,
                        s.name,
                        blob.len(),
                        nodes,
                        faces
                    );
                }
            }
        }
    }
    if any {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn cmd_extract(files: &[PathBuf], outdir: &Path) -> ExitCode {
    if let Err(e) = std::fs::create_dir_all(outdir) {
        eprintln!("cannot create {}: {e}", outdir.display());
        return ExitCode::FAILURE;
    }
    let mut count = 0;
    for path in files {
        let Ok(data) = std::fs::read(path) else {
            continue;
        };
        let Ok(file) = container::parse(&data) else {
            println!("{}: not a modern SolidWorks file", path.display());
            continue;
        };
        let stem = path.file_stem().unwrap_or_default().to_string_lossy();
        for s in file.streams() {
            for blob in sections::carve_zlib(&s.data) {
                let Some(kind) = sections::transmit_kind(&blob) else {
                    continue;
                };
                let safe = s.name.replace('/', ".");
                let dest = outdir.join(format!("{stem}.{safe}.{kind:?}.x_b").to_lowercase());
                if std::fs::write(&dest, &blob).is_ok() {
                    println!("{} ({} bytes, {:?})", dest.display(), blob.len(), kind);
                    count += 1;
                }
            }
        }
    }
    if count > 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn cmd_mesh(
    file: &Path,
    obj: Option<&Path>,
    stl: Option<&Path>,
    tol: Option<f64>,
    stats: bool,
    quiet: bool,
) -> ExitCode {
    let mesh = match mesh_file(file, tol) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("{}: {e}", file.display());
            return ExitCode::FAILURE;
        }
    };
    let obj_path = obj
        .map(PathBuf::from)
        .unwrap_or_else(|| file.with_extension("obj"));
    if let Err(e) = mesh.write_obj(&obj_path) {
        eprintln!("write {}: {e}", obj_path.display());
        return ExitCode::FAILURE;
    }
    if let Some(p) = stl {
        if let Err(e) = mesh.write_stl(p) {
            eprintln!("write {}: {e}", p.display());
            return ExitCode::FAILURE;
        }
    }
    let faces: std::collections::HashSet<_> = mesh.face_ids.iter().collect();
    println!(
        "{}: {} vertices, {} triangles from {} faces -> {}",
        file.display(),
        mesh.vertices.len(),
        mesh.triangles.len(),
        faces.len(),
        obj_path.display()
    );
    if stats {
        println!(
            "  boundary edges: {}  signed volume: {:+.6e}  area: {:.6e}  diag: {:.6e}",
            mesh.boundary_edge_count(),
            mesh.signed_volume(),
            mesh.surface_area(),
            mesh.bbox_diagonal()
        );
    }
    if !quiet {
        for w in &mesh.warnings {
            eprintln!("  warn: {w}");
        }
    }
    if mesh.triangles.is_empty() {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

fn cmd_render(
    files: &[PathBuf],
    out: &Path,
    opts: &RenderOptions,
    cols: Option<usize>,
    tol: Option<f64>,
) -> ExitCode {
    let mut frags = Vec::new();
    for path in files {
        let title = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        match mesh_file(path, tol) {
            Ok(mesh) => {
                let mut o = opts.clone();
                o.title = Some(title);
                frags.push(render_mesh_svg(&mesh, &o));
                println!(
                    "{}: {} triangles rendered",
                    path.display(),
                    mesh.triangles.len()
                );
            }
            Err(e) => {
                println!("{}: FAILED: {e}", path.display());
                frags.push(format!(
                    r##"<g><text x="{:.0}" y="{:.0}" text-anchor="middle" fill="#f7768e" font-size="12" font-family="sans-serif">{}: failed</text></g>"##,
                    opts.size / 2.0,
                    opts.size / 2.0,
                    title
                ));
            }
        }
    }
    let cols = cols.unwrap_or_else(|| (frags.len() as f64).sqrt().ceil().max(1.0) as usize);
    let doc = svg_document(&frags, cols, opts.size);
    if let Err(e) = std::fs::write(out, doc) {
        eprintln!("write {}: {e}", out.display());
        return ExitCode::FAILURE;
    }
    println!("wrote {}", out.display());
    ExitCode::SUCCESS
}

/// Keep `tess` and `body_graphs` referenced for the binary's link graph even
/// when only some subcommands are used.
#[allow(dead_code)]
fn _unused(g: &Graph) {
    let _ = tess::tessellate(g, None);
    let _ = body_graphs(Path::new("/dev/null"));
}
