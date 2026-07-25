//! Corpus test: parse every sample and vault part and compare the container /
//! section / XT decoding against ground truth produced by the Python pipeline
//! (`solid_diff` + `vendor/ps-parser`).
//!
//! `tests/data/golden.txt` was generated from Python; each line is either
//!   `<relpath> streams=N`             file-level summary
//!   `<relpath> NOT_MODERN`            legacy OLE2 file
//!   `  <stream>@<off> <kind> size=N nodes=M vals=<hash> NAME:count,...`
//!   `  <stream>@<off> <kind> size=N ERR`   (Python refused to parse it)
//!
//! `vals` is an FNV-1a 64 hash of a canonical dump of *every field value of
//! every node* (floats as raw big-endian bits), so the check covers field
//! decoding, not just node counts.
//!
//! Regenerate with `.venv/bin/python rust/tests/data/gen_golden.py`.
//! Parts missing from the checkout (samples/*.SLDPRT are fetched, not
//! committed) are skipped rather than failing.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use solid_diff::value::{Node, Value};
use solid_diff::{container, sections, xt};

const GOLDEN: &str = include_str!("data/golden.txt");

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf()
}

fn corpus() -> Vec<PathBuf> {
    let root = repo_root();
    let mut out = Vec::new();
    for dir in ["samples", "vault"] {
        let Ok(entries) = std::fs::read_dir(root.join(dir)) else {
            continue; // corpus directory not present in this checkout
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

fn kind_str(k: sections::TransmitKind) -> &'static str {
    match k {
        sections::TransmitKind::Partition => "partition",
        sections::TransmitKind::Deltas => "deltas",
        sections::TransmitKind::Part => "part",
    }
}

fn fmt_value(v: &Value, out: &mut String) {
    use std::fmt::Write;
    match v {
        Value::Null | Value::Ptr(None) => out.push('N'),
        Value::Bool(b) => out.push(if *b { 'T' } else { 'F' }),
        Value::U8(x) => write!(out, "{x}").unwrap(),
        Value::I16(x) => write!(out, "{x}").unwrap(),
        Value::I32(x) => write!(out, "{x}").unwrap(),
        Value::Ptr(Some(x)) => write!(out, "{x}").unwrap(),
        Value::F64(x) => write!(out, "f{:016x}", x.to_bits()).unwrap(),
        Value::Char(s) | Value::Utf16(s) => write!(out, "s{s}").unwrap(),
        Value::Vec3(v) => {
            out.push('(');
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                fmt_value(&Value::F64(*x), out);
            }
            out.push(')');
        }
        Value::Interval(v) => {
            out.push('(');
            fmt_value(&Value::F64(v[0]), out);
            out.push(',');
            fmt_value(&Value::F64(v[1]), out);
            out.push(')');
        }
        Value::Box3(b) => {
            out.push('(');
            for (i, iv) in b.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                fmt_value(&Value::Interval(*iv), out);
            }
            out.push(')');
        }
        Value::Array(items) => {
            out.push('[');
            for (i, it) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                fmt_value(it, out);
            }
            out.push(']');
        }
    }
}

fn fnv1a(data: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in data {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// Canonical dump of every node's every field; hashed for comparison with the
/// same dump produced by the Python reference.
fn value_hash(nodes: &[Node]) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    for (i, n) in nodes.iter().enumerate() {
        if i > 0 {
            s.push('\n');
        }
        match n.count {
            Some(c) => write!(s, "{}|{}|{}|", n.id, n.node_type, c).unwrap(),
            None => write!(s, "{}|{}||", n.id, n.node_type).unwrap(),
        }
        for (j, (name, value)) in n.fields.iter().enumerate() {
            if j > 0 {
                s.push(';');
            }
            write!(s, "{name}=").unwrap();
            fmt_value(value, &mut s);
        }
    }
    format!("{:016x}", fnv1a(s.as_bytes()))
}

/// Re-derive the golden summary for one part file: a header line plus one line
/// per Parasolid transmit found in it.
fn summarize(path: &Path) -> Vec<String> {
    let root = repo_root();
    let mut lines = Vec::new();
    let rel = path
        .strip_prefix(&root)
        .unwrap()
        .to_string_lossy()
        .to_string();
    let data = std::fs::read(path).unwrap();

    if !container::is_modern_swx(&data) {
        assert!(
            matches!(
                container::parse(&data),
                Err(solid_diff::Error::NotModernSldprt)
            ),
            "{rel}: expected NotModernSldprt"
        );
        return vec![format!("{rel} NOT_MODERN")];
    }

    let file = container::parse(&data).unwrap_or_else(|e| panic!("{rel}: {e}"));
    let streams = file.streams();
    lines.push(format!("{rel} streams={}", streams.len()));
    for stream in &streams {
        for (off, blob) in sections::carve_zlib_offsets(&stream.data) {
            let Some(kind) = sections::transmit_kind(&blob) else {
                continue;
            };
            let head = format!(
                "  {}@{} {} size={}",
                stream.name,
                off,
                kind_str(kind),
                blob.len()
            );
            match xt::parse_transmit(&blob) {
                Ok(nodes) => {
                    let mut hist: BTreeMap<&str, usize> = BTreeMap::new();
                    for n in &nodes {
                        *hist.entry(n.name.as_str()).or_default() += 1;
                    }
                    let h: Vec<String> = hist.iter().map(|(k, v)| format!("{k}:{v}")).collect();
                    lines.push(format!(
                        "{head} nodes={} vals={} {}",
                        nodes.len(),
                        value_hash(&nodes),
                        h.join(",")
                    ));
                }
                Err(_) => lines.push(format!("{head} ERR")),
            }
        }
    }
    lines
}

/// Split the golden file into per-part blocks keyed by relative path.
fn golden_blocks() -> Vec<(String, Vec<&'static str>)> {
    let mut blocks: Vec<(String, Vec<&str>)> = Vec::new();
    for line in GOLDEN.lines() {
        if line.starts_with("  ") {
            blocks
                .last_mut()
                .expect("golden: transmit line before any file line")
                .1
                .push(line);
        } else {
            let file = line.split(' ').next().unwrap().to_string();
            blocks.push((file, vec![line]));
        }
    }
    blocks
}

/// Read a corpus file, or `None` when it isn't on disk (samples/*.SLDPRT are
/// fetched by samples/fetch.sh, not committed).
fn read_part(file: &str) -> Option<Vec<u8>> {
    match std::fs::read(repo_root().join(file)) {
        Ok(d) => Some(d),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            eprintln!("skipping: {file} is not on disk (run samples/fetch.sh)");
            None
        }
        Err(e) => panic!("{file}: {e}"),
    }
}

/// Parse one part file and return the nodes of the first transmit whose
/// container stream name ends with `stream_suffix`.
fn nodes_of(file: &str, stream_suffix: &str) -> Option<Vec<Node>> {
    let data = read_part(file)?;
    let container = container::parse(&data).unwrap();
    for stream in container.streams() {
        if !stream.name.ends_with(stream_suffix) {
            continue;
        }
        for blob in sections::carve_zlib(&stream.data) {
            if sections::transmit_kind(&blob) == Some(sections::TransmitKind::Deltas) {
                continue;
            }
            if let Ok(nodes) = xt::parse_transmit(&blob) {
                return Some(nodes);
            }
        }
    }
    panic!("{file}: no parseable transmit in a stream ending {stream_suffix:?}");
}

fn count_of(nodes: &[Node], name: &str) -> usize {
    nodes.iter().filter(|n| n.name == name).count()
}

fn by_id(nodes: &[Node], id: i16) -> &Node {
    nodes.iter().find(|n| n.id == id).unwrap()
}

#[test]
fn legacy_ole2_files_are_rejected() {
    for f in [
        "samples/Arm_link1_tube.SLDPRT",
        "samples/BlockA.SLDPRT",
        "vault/04_M83513_01-AN_PART3__000005d4.SLDPRT",
    ] {
        let Some(data) = read_part(f) else { continue };
        assert!(!container::is_modern_swx(&data), "{f}");
        assert!(
            matches!(container::parse(&data), Err(solid_diff::Error::NotModernSldprt)),
            "{f}"
        );
    }
}

#[test]
fn arm_base_partition_topology() {
    let Some(nodes) = nodes_of("samples/3_DOF_ARM_BASE.SLDPRT", "Contents/Config-0-Partition")
    else {
        return;
    };
    assert_eq!(nodes.len(), 134);
    for (name, want) in [
        ("BODY", 1),
        ("FACE", 6),
        ("CIRCLE", 6),
        ("CYLINDER", 3),
        ("PLANE", 3),
        ("LOOP", 12),
        ("HALFEDGE", 12),
        ("EDGE", 6),
    ] {
        assert_eq!(count_of(&nodes, name), want, "{name}");
    }

    // Field values, checked against the Python decoder.
    let cyl = by_id(&nodes, 8);
    assert_eq!(cyl.name, "CYLINDER");
    assert_eq!(cyl.node_type, 51);
    assert_eq!(cyl.i64("node_id"), Some(28));
    assert_eq!(cyl.str("sense"), Some("-"));
    assert_eq!(cyl.f64("radius"), Some(0.05));
    assert_eq!(cyl.vec3("pvec"), Some([0.0, 0.005, 0.0]));
    assert_eq!(cyl.vec3("axis"), Some([-0.0, -1.0, -0.0]));
    assert_eq!(cyl.vec3("x_axis"), Some([0.0, 0.0, -1.0]));
    assert_eq!(cyl.ptr("owner"), Some(35));
    assert_eq!(cyl.ptr("next"), Some(36));
    assert_eq!(cyl.ptr("previous"), None); // XT null pointer
    // Note: `Node::field` only filters `Value::Null`, so a null *pointer*
    // still comes back as `Some(Ptr(None))`.
    assert_eq!(cyl.field("attributes_features"), Some(&Value::Ptr(None)));
    assert_eq!(cyl.ptr("attributes_features"), None);

    let circle = by_id(&nodes, 9);
    assert_eq!(circle.name, "CIRCLE");
    assert_eq!(circle.f64("radius"), Some(0.05));
    assert_eq!(circle.vec3("normal"), Some([0.0, 1.0, 0.0]));
    assert_eq!(circle.str("sense"), Some("+"));

    let body = by_id(&nodes, 3);
    assert_eq!(body.name, "BODY");
    assert_eq!(body.i64("body_type"), Some(1)); // solid
    assert_eq!(body.i64("highest_node_id"), Some(191));
    assert_eq!(body.f64("res_linear"), Some(1e-08));
    assert_eq!(body.ptr("shell"), Some(7));
}

#[test]
fn vault_local_bodies_surfaces() {
    let nodes = nodes_of(
        "vault/01_Detector__STAR_tube_window__00000108.SLDPRT",
        "Config-0-FeatureBodies/LocalBodies",
    )
    .expect("vault parts are committed");
    assert_eq!(nodes.len(), 344);
    for (name, want) in [
        ("FACE", 12),
        ("TORUS", 2),
        ("CONE", 4),
        ("PLANE", 3),
        ("CYLINDER", 3),
        // UTF-16 field decoding is exercised by these.
        ("UNICODE_VALUES", 70),
    ] {
        assert_eq!(count_of(&nodes, name), want, "{name}");
    }
    // The 1-node Partition stub newer files carry alongside LocalBodies.
    let stub = nodes_of(
        "vault/01_Detector__STAR_tube_window__00000108.SLDPRT",
        "Contents/Config-0-Partition",
    )
    .unwrap();
    assert_eq!(stub.len(), 1);
    assert_eq!(stub[0].name, "WORLD");
}

#[test]
fn variable_length_nodes_and_arrays() {
    let Some(nodes) = nodes_of("samples/bbox-precision.SLDPRT", "Contents/Config-0-Partition")
    else {
        return;
    };
    assert_eq!(nodes.len(), 540);

    // KNOT_SET: variable-length node, last field repeats `count` times.
    let knots = by_id(&nodes, 131);
    assert_eq!(knots.name, "KNOT_SET");
    assert_eq!(knots.count, Some(15));
    let k = knots.f64_vec("knots").unwrap();
    assert_eq!(k.len(), 15);
    assert_eq!(k[0], -0.3081671385999001);
    assert_eq!(k[3], 0.0);
    assert_eq!(k[11], 1.0);
    assert_eq!(k[14], 1.2982817840084953);

    let mult = by_id(&nodes, 130);
    assert_eq!(mult.name, "KNOT_MULT");
    assert_eq!(mult.count, Some(15));
    assert_eq!(mult.i64_vec("mult").unwrap(), vec![1i64; 15]);

    let nurbs = by_id(&nodes, 124);
    assert_eq!(nurbs.name, "NURBS_CURVE");
    assert_eq!(nurbs.i64("degree"), Some(3));
    assert_eq!(nurbs.i64("n_vertices"), Some(11));
    assert_eq!(nurbs.i64("vertex_dim"), Some(3));
    assert!(nurbs.bool("periodic") && nurbs.bool("closed") && !nurbs.bool("rational"));
    assert_eq!(nurbs.ptr("bspline_vertices"), Some(129));
    assert_eq!(nurbs.ptr("knots"), Some(131));

    let verts = by_id(&nodes, 129);
    assert_eq!(verts.name, "BSPLINE_VERTICES");
    assert_eq!(verts.count, Some(33));
    let v = verts.f64_vec("vertices").unwrap();
    assert_eq!(v.len(), 33);
    assert_eq!(&v[..3], &[-0.0574889345335749, 0.05875585229752688, 0.0]);

    // Fixed arrays: BLENDED_EDGE.surface is 2 pointers, .range 2 floats.
    let blend = by_id(&nodes, 208);
    assert_eq!(blend.name, "BLENDED_EDGE");
    assert_eq!(blend.str("blend_type"), Some("R"));
    assert_eq!(blend.ptrs("surface"), vec![39, 149]);
    assert_eq!(blend.f64_vec("range").unwrap(), vec![0.01, 0.01]);

    // CHART: variable node whose repeating field is a vector.
    let chart = by_id(&nodes, 135);
    assert_eq!(chart.name, "CHART");
    assert_eq!(chart.count, Some(25));
    assert_eq!(chart.i64("chart_count"), Some(25));
    let hvec = chart.f64_vec("hvec").unwrap();
    assert_eq!(hvec.len(), 75);
    assert_eq!(&hvec[..3], &[-0.01838903930593006, 0.054, 0.06728998362774519]);
    // `parameter_error` is a 2-element f64 array of XT nulls.
    match chart.fields.iter().find(|(n, _)| n == "parameter_error") {
        Some((_, Value::Array(items))) => {
            assert_eq!(items.len(), 2);
            assert!(items.iter().all(|v| v.is_null()));
        }
        other => panic!("unexpected parameter_error: {other:?}"),
    }

    let swept = by_id(&nodes, 39);
    assert_eq!(swept.name, "SWEPT_SURF");
    assert_eq!(swept.vec3("sweep"), Some([-0.0, -1.0, -0.0]));
    assert_eq!(swept.f64("scale"), Some(0.06653143851578683));
    let offset = by_id(&nodes, 9);
    assert_eq!(offset.name, "OFFSET_SURF");
    assert_eq!(offset.f64("offset"), Some(0.01));
    assert_eq!(offset.str("check"), Some("U"));
    assert!(!offset.bool("true_offset"));
}

/// Standalone `.x_b` exports carry an ASCII `**PARASOLID` header before the
/// binary `PS` payload; the decoder must skip it.
#[test]
fn ascii_parasolid_banner_is_skipped() {
    let Some(nodes) = nodes_of("samples/3_DOF_ARM_BASE.SLDPRT", "Contents/Config-0-Partition")
    else {
        return;
    };
    let data = read_part("samples/3_DOF_ARM_BASE.SLDPRT").unwrap();
    let stream = container::parse(&data)
        .unwrap()
        .streams()
        .into_iter()
        .find(|s| s.name.ends_with("Contents/Config-0-Partition"))
        .unwrap();
    let blob = sections::carve_zlib(&stream.data).remove(0);

    let mut xb: Vec<u8> = Vec::new();
    xb.extend_from_slice(b"**PARASOLID !\"#$%&'()*+,-./:;<=>?@\n**PART1;FORMAT=binary;\n");
    xb.extend_from_slice(b"**END_OF_HEADER**\n");
    xb.extend_from_slice(&blob);
    let from_xb = xt::parse_transmit(&xb).unwrap();
    assert_eq!(from_xb.len(), nodes.len());
    assert_eq!(from_xb[0].name, nodes[0].name);
    assert_eq!(value_hash(&from_xb), value_hash(&nodes));
}

/// Truncated / corrupted input must produce errors, never panics.
#[test]
fn malformed_input_is_an_error_not_a_panic() {
    let Some(data) = read_part("samples/3_DOF_ARM_BASE.SLDPRT") else {
        return;
    };
    for cut in [0usize, 1, 7, 21, 22, 64, 1000, 5000] {
        let head = &data[..cut.min(data.len())];
        let _ = container::is_modern_swx(head);
        let _ = container::parse(head);
    }

    let stream = container::parse(&data)
        .unwrap()
        .streams()
        .into_iter()
        .find(|s| s.name.ends_with("Contents/Config-0-Partition"))
        .unwrap();
    let blob = sections::carve_zlib(&stream.data).remove(0);
    for cut in (0..blob.len()).step_by(37) {
        let _ = xt::parse_transmit(&blob[..cut]);
    }
    // Flip bytes throughout the payload; every outcome must be Ok or Err.
    for i in (0..blob.len()).step_by(53) {
        let mut bad = blob.clone();
        bad[i] ^= 0xFF;
        let _ = xt::parse_transmit(&bad);
    }
    // Carving must terminate on garbage that merely looks like zlib. (This
    // particular junk decodes as one truncated stream in Python too, so the
    // expected output length is Python's.)
    let mut junk = vec![0x78u8, 0x9c];
    junk.extend(std::iter::repeat_n(0xABu8, 5000));
    let carved = sections::carve_zlib(&junk);
    assert_eq!(carved.len(), 1);
    assert_eq!(carved[0].len(), 4999);
}

#[test]
fn matches_python_pipeline() {
    let root = repo_root();
    let on_disk: std::collections::HashSet<String> = corpus()
        .iter()
        .map(|p| p.strip_prefix(&root).unwrap().to_string_lossy().to_string())
        .collect();

    let mut diffs = Vec::new();
    let mut checked = 0;
    let mut skipped = 0;
    let mut in_golden = std::collections::HashSet::new();

    for (file, want) in golden_blocks() {
        in_golden.insert(file.clone());
        if !on_disk.contains(&file) {
            // samples/*.SLDPRT are fetched, not committed (samples/fetch.sh).
            skipped += 1;
            continue;
        }
        checked += 1;
        let got = summarize(&root.join(&file));
        for i in 0..got.len().max(want.len()) {
            let g = got.get(i).map(|s| s.as_str()).unwrap_or("<missing>");
            let w = want.get(i).copied().unwrap_or("<missing>");
            if g != w {
                diffs.push(format!("{file} line {i}:\n  python: {w}\n  rust:   {g}"));
            }
        }
    }

    let extra: Vec<&String> = on_disk.iter().filter(|f| !in_golden.contains(*f)).collect();
    assert!(
        extra.is_empty(),
        "corpus files with no golden entry (regenerate tests/data/golden.txt): {extra:?}"
    );
    assert!(checked > 0, "no corpus files found on disk");
    assert!(
        diffs.is_empty(),
        "{} lines differ from the Python pipeline ({checked} files checked, {skipped} \
         missing from disk):\n{}",
        diffs.len(),
        diffs.iter().take(20).cloned().collect::<Vec<_>>().join("\n")
    );
}
