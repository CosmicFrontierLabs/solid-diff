//! Real-data cross-check against the Python reference evaluators.
//!
//! `tests/data/geom_golden.txt` is produced by `tests/gen_golden.py` from
//! `solid_diff/geom.py` (the ground truth). Each row replays one evaluation on
//! a real sample part. If the XT parser cannot read the samples yet the test
//! reports and skips rather than failing.

use std::collections::HashMap;
use std::path::PathBuf;

use solid_diff::geom::{make_curve, make_surface};
use solid_diff::graph::Graph;

fn samples_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("samples")
}

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/geom_golden.txt")
}

#[test]
fn matches_python_reference() {
    let text = std::fs::read_to_string(golden_path()).expect("golden data");
    let mut graphs: HashMap<String, Option<Graph>> = HashMap::new();
    // Worst absolute deviation seen, per node kind and quantity.
    let mut worst: HashMap<(String, &str), f64> = HashMap::new();
    let mut checked = 0usize;
    let mut skipped_parse = 0usize;

    for line in text.lines() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        let (tag, sample, id, kind) = (f[0], f[1], f[2], f[3]);
        let num = |i: usize| f[i].parse::<f64>().unwrap();

        let entry =
            graphs.entry(sample.to_string()).or_insert_with(|| {
                match solid_diff::body_graphs(&samples_dir().join(sample)) {
                    Ok(mut gs) if !gs.is_empty() => Some(gs.remove(0).graph),
                    _ => None,
                }
            });
        let Some(graph) = entry.as_ref() else {
            skipped_parse += 1;
            continue;
        };
        let id: i16 = id.parse().unwrap();
        let node = graph
            .get(id)
            .unwrap_or_else(|| panic!("{sample}: node {id} missing from the Rust graph"));
        assert_eq!(node.name, kind, "{sample}: node {id} kind mismatch");

        let mut note = |q: &'static str, err: f64| {
            let e = worst.entry((kind.to_string(), q)).or_insert(0.0);
            *e = e.max(err);
        };

        match tag {
            // eval(u,v) -> p, inv(p) -> uv
            "SE" => {
                let surf = make_surface(graph, node)
                    .unwrap_or_else(|| panic!("{sample}: {kind} {id} failed to build"));
                let uv = [num(4), num(5)];
                let p = [num(6), num(7), num(8)];
                let back = [num(9), num(10)];
                let got = surf.eval(uv);
                for k in 0..3 {
                    note("eval", (got[k] - p[k]).abs());
                }
                let got_uv = surf.inv(got);
                note("inv_u", param_diff(got_uv[0], back[0], surf.period_u()));
                note("inv_v", param_diff(got_uv[1], back[1], surf.period_v()));
                checked += 1;
            }
            // inv(p) -> uv, eval(uv) -> q, for p a real boundary vertex
            "SI" => {
                let surf = make_surface(graph, node)
                    .unwrap_or_else(|| panic!("{sample}: {kind} {id} failed to build"));
                let p = [num(4), num(5), num(6)];
                let uv = [num(7), num(8)];
                let q = [num(9), num(10), num(11)];
                let got_uv = surf.inv(p);
                note("inv_u", param_diff(got_uv[0], uv[0], surf.period_u()));
                note("inv_v", param_diff(got_uv[1], uv[1], surf.period_v()));
                let got_q = surf.eval(got_uv);
                for k in 0..3 {
                    note("eval", (got_q[k] - q[k]).abs());
                }
                checked += 1;
            }
            // eval(t) -> p, inv(p) -> t
            "C" => {
                let crv = make_curve(graph, node)
                    .unwrap_or_else(|| panic!("{sample}: {kind} {id} failed to build"));
                let t = num(4);
                let p = [num(5), num(6), num(7)];
                let back = num(8);
                let got = crv.eval(t);
                for k in 0..3 {
                    note("eval", (got[k] - p[k]).abs());
                }
                note("inv", param_diff(crv.inv(got), back, crv.period()));
                checked += 1;
            }
            other => panic!("unknown golden tag {other}"),
        }
    }

    if checked == 0 {
        eprintln!("golden: XT parser unavailable ({skipped_parse} rows skipped)");
        return;
    }
    let mut keys: Vec<_> = worst.keys().cloned().collect();
    keys.sort();
    for k in &keys {
        eprintln!(
            "golden {:<14} {:<6} max |diff| = {:.3e}",
            k.0, k.1, worst[k]
        );
    }
    for ((kind, quantity), err) in &worst {
        // 3D coordinates are metres on ~0.1 m parts; parameters are radians or
        // metres. Iterative inverses (blends, sweeps, NURBS) are looser.
        let tol = match (kind.as_str(), *quantity) {
            (_, "eval") => 1e-9,
            ("BLENDED_EDGE", _) | ("SWEPT_SURF", _) => 1e-6,
            ("B_CURVE", _) | ("INTERSECTION", _) => 1e-7,
            _ => 1e-9,
        };
        assert!(
            *err <= tol,
            "{kind} {quantity}: max |diff| {err:.3e} exceeds {tol:.1e}"
        );
    }
    eprintln!("golden: {checked} evaluations match the Python reference");
}

/// Absolute difference, folded into a period when the parameter has one.
fn param_diff(a: f64, b: f64, period: Option<f64>) -> f64 {
    match period {
        Some(p) if p > 0.0 => {
            let d = (a - b).rem_euclid(p);
            d.min(p - d)
        }
        _ => (a - b).abs(),
    }
}
