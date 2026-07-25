"""Compare the Rust and Python pipelines over a corpus of part files.

Runs both meshers on every file and reports per-file agreement on triangle
count, boundary-edge count and signed volume, plus wall-clock speedup.

Usage: python tools/parity.py DIR_OR_FILES... [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

RUST_BIN = "/tmp/target-main/release/solid-diff"


def python_mesh(path: str, timeout: float):
    from solid_diff.brep2mesh import graphs_from_sldprt
    from solid_diff.tess import tessellate

    t0 = time.time()
    graphs = graphs_from_sldprt(path)
    if not graphs:
        return {"status": "no-geometry"}
    m = tessellate(graphs[0][1])
    secs = time.time() - t0
    import numpy as np

    v, t = m.vertices, m.triangles
    if not len(t):
        return {"status": "empty", "secs": secs}
    edges = {}
    for tri in t:
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            k = (min(a, b), max(a, b))
            edges[k] = edges.get(k, 0) + 1
    vol = float(
        np.einsum("ij,ij->i", v[t[:, 0]], np.cross(v[t[:, 1]], v[t[:, 2]])).sum() / 6
    )
    return {
        "status": "ok",
        "tris": int(len(t)),
        "boundary": int(sum(1 for c in edges.values() if c == 1)),
        "volume": vol,
        "secs": secs,
    }


def rust_mesh(path: str, timeout: float):
    t0 = time.time()
    try:
        r = subprocess.run(
            [RUST_BIN, "mesh", path, "-o", "/tmp/parity_rust.obj", "--stats", "-q"],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"status": f"timeout>{timeout}s"}
    secs = time.time() - t0
    out = r.stdout
    if "triangles from" not in out:
        return {"status": (r.stderr.strip() or out.strip() or "failed")[:80], "secs": secs}
    try:
        tris = int(out.split(" triangles from")[0].split(",")[-1].strip())
        bnd = int(out.split("boundary edges:")[1].split()[0])
        vol = float(out.split("signed volume:")[1].split()[0])
    except (IndexError, ValueError) as e:
        return {"status": f"unparseable output: {e}", "secs": secs}
    return {"status": "ok", "tris": tris, "boundary": bnd, "volume": vol, "secs": secs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--json", default=None)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()

    files = []
    for p in args.paths:
        p = Path(p)
        files.extend(sorted(p.glob("*.SLDPRT")) if p.is_dir() else [p])
    if args.limit:
        files = files[: args.limit]

    rows = []
    agree = same_tris = 0
    for f in files:
        py = python_mesh(str(f), args.timeout)
        rs = rust_mesh(str(f), args.timeout)
        row = {"file": f.name, "py": py, "rust": rs}
        if py.get("status") == "ok" and rs.get("status") == "ok":
            # volumes should match closely; triangle counts within a few %
            dv = abs(py["volume"] - rs["volume"])
            scale = max(abs(py["volume"]), abs(rs["volume"]), 1e-12)
            row["vol_rel"] = dv / scale
            row["tri_ratio"] = rs["tris"] / max(py["tris"], 1)
            row["speedup"] = py["secs"] / max(rs["secs"], 1e-6)
            if row["vol_rel"] < 0.02:
                agree += 1
            if 0.8 <= row["tri_ratio"] <= 1.25:
                same_tris += 1
        rows.append(row)
        print(
            f"{f.name[:40]:42} py={py.get('tris', py.get('status'))} "
            f"rust={rs.get('tris', rs.get('status'))} "
            f"vol_rel={row.get('vol_rel', float('nan')):.4f} "
            f"speedup={row.get('speedup', float('nan')):.1f}x",
            flush=True,
        )

    both_ok = [r for r in rows if r["py"].get("status") == "ok" and r["rust"].get("status") == "ok"]
    print(
        f"\n{len(files)} files: both ok {len(both_ok)}, "
        f"volume agrees {agree}, triangle count comparable {same_tris}"
    )
    if both_ok:
        sp = sorted(r["speedup"] for r in both_ok)
        print(f"median speedup: {sp[len(sp)//2]:.1f}x")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
