//! Corpus test: parse every sample and vault part and assert structural
//! facts about the decode -- node types, field values, error handling.
//!
//! Whole-corpus invariants that the files assert about themselves live in
//! `tests/invariants.rs`; this file holds hand-written spot checks.
//!
//! Parts missing from the checkout (samples/*.SLDPRT are fetched, not
//! committed) are skipped rather than failing.

use std::path::{Path, PathBuf};

use solid_diff::value::{Node, Value};
use solid_diff::{container, sections, xt};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
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
            matches!(
                container::parse(&data),
                Err(solid_diff::Error::NotModernSldprt)
            ),
            "{f}"
        );
    }
}

#[test]
fn arm_base_partition_topology() {
    let Some(nodes) = nodes_of(
        "samples/3_DOF_ARM_BASE.SLDPRT",
        "Contents/Config-0-Partition",
    ) else {
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
    let Some(nodes) = nodes_of(
        "samples/bbox-precision.SLDPRT",
        "Contents/Config-0-Partition",
    ) else {
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
    assert_eq!(
        &hvec[..3],
        &[-0.01838903930593006, 0.054, 0.06728998362774519]
    );
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
    let Some(nodes) = nodes_of(
        "samples/3_DOF_ARM_BASE.SLDPRT",
        "Contents/Config-0-Partition",
    ) else {
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
    // Same bytes with and without the banner must decode to identical nodes,
    // field for field -- the banner is skipped, not partially consumed.
    for (a, b) in from_xb.iter().zip(nodes.iter()) {
        assert_eq!(format!("{a:?}"), format!("{b:?}"), "node {} differs", a.id);
    }
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
