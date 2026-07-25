"""Mesh the Parasolid B-rep inside a SolidWorks part (or a raw .x_b).

Usage:
  python -m solid_diff.brep2mesh part.SLDPRT [-o out.obj] [--stl out.stl] [--tol M]

For a .SLDPRT the embedded Config partition transmits are extracted in-memory
(see extract.py); for a .x_b the file is parsed directly.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .container import parse_file
from .extract import carve_zlib, describe_transmit
from .tess import Mesh, tessellate, write_obj, write_stl
from .xt import Graph


def graphs_from_sldprt(path: str) -> list[tuple[str, Graph]]:
    """(stream name, node graph) for embedded transmits that carry faces.

    Where the solid lives varies by SolidWorks era: older 2015+ files keep it
    in `Contents/Config-N-Partition`; newer (~2024) files store an empty
    partition and put the bodies in `Config-N-FeatureBodies/LocalBodies`.
    Rather than hardcode stream names, carve every stream and keep every
    parseable partition/part transmit, sorted by face count (best first).
    Ghost partitions are excluded (reference wire bodies, not the solid).
    """
    swx = parse_file(path)
    out = []
    for name, data in swx.streams.items():
        if name.endswith("-GhostPartition"):
            continue
        for _, blob in carve_zlib(data):
            info = describe_transmit(blob)
            if not info or info[0] == "deltas":
                continue
            try:
                g = Graph.from_bytes(blob)
            except Exception:
                continue
            out.append((name, g))
    out.sort(key=lambda ng: len(ng[1].by_type("FACE")), reverse=True)
    return out


def mesh_file(path: str, tol: float | None = None) -> Mesh:
    """Mesh the first partition of a .SLDPRT, or a bare .x_b."""
    if path.lower().endswith((".x_b", ".xb", ".bin")):
        return tessellate(Graph.from_file(path), tol)
    graphs = graphs_from_sldprt(path)
    if not graphs:
        raise ValueError(f"{path}: no embedded Parasolid partition found")
    if len(graphs) > 1:
        print(f"note: {len(graphs)} configs embedded; meshing {graphs[0][0]}",
              file=sys.stderr)
    return tessellate(graphs[0][1], tol)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("input", help=".SLDPRT or .x_b file")
    ap.add_argument("-o", "--obj", help="output OBJ path (default: input stem .obj)")
    ap.add_argument("--stl", help="also write binary STL here")
    ap.add_argument("--tol", type=float, default=None,
                    help="chordal tolerance in model units (default: 0.2%% of size)")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args()

    mesh = mesh_file(args.input, args.tol)
    obj_path = args.obj or str(Path(args.input).with_suffix(".obj"))
    write_obj(mesh, obj_path)
    if args.stl:
        write_stl(mesh, args.stl)

    nfaces = len(set(mesh.face_ids.tolist()))
    print(f"{args.input}: {len(mesh.vertices)} vertices, "
          f"{len(mesh.triangles)} triangles from {nfaces} faces -> {obj_path}")
    if mesh.warnings and not args.quiet:
        for w in mesh.warnings:
            print(f"  warn: {w}", file=sys.stderr)
    if not len(mesh.triangles):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
