"""Painter's-algorithm SVG renderer for tessellated parts.

Orthographic projection, triangles depth-sorted back-to-front and drawn as
semi-transparent shaded polygons, so interior structure shows through.

Usage:
  python -m solid_diff.render part.SLDPRT [more.SLDPRT ...] [-o out.svg]
                              [--alpha A] [--elev DEG] [--azim DEG] [--size PX]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .brep2mesh import mesh_file
from .tess import Mesh

BASE_RGB = (0.48, 0.64, 0.97)  # portal-friendly blue
BACK_TINT = (0.72, 0.55, 0.90)  # backfaces lean purple so cavities read
LIGHT = np.array([0.45, 0.5, 0.75])


def _view_matrix(elev_deg: float, azim_deg: float) -> np.ndarray:
    """Rows are the camera's right/up/forward axes in world space."""
    e, a = np.radians(elev_deg), np.radians(azim_deg)
    fwd = -np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    if np.linalg.norm(right) < 1e-9:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return np.stack([right, up, fwd])


def render_mesh_svg(
    mesh: Mesh,
    alpha: float = 0.55,
    elev: float = 28.0,
    azim: float = -55.0,
    size: int = 520,
    title: str | None = None,
) -> str:
    """Render one mesh to an SVG fragment (a <g> sized to `size` px)."""
    v, t = mesh.vertices, mesh.triangles
    if not len(t):
        return f'<g><text fill="#f7768e">empty mesh: {title}</text></g>'

    M = _view_matrix(elev, azim)
    pv = v @ M.T  # x=right, y=up, z=depth (camera looks along +z)

    tri = pv[t]  # (n,3,3)
    depth = tri[:, :, 2].mean(axis=1)
    order = np.argsort(depth)  # farthest (most negative fwd dist) first
    tri = tri[order]

    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    facing = n[:, 2] < 0  # normal towards camera
    lightdir = M @ (LIGHT / np.linalg.norm(LIGHT))
    shade = np.abs(n @ lightdir) * 0.65 + 0.35

    lo = tri[:, :, :2].reshape(-1, 2).min(axis=0)
    hi = tri[:, :, :2].reshape(-1, 2).max(axis=0)
    span = (hi - lo).max() or 1.0
    pad = 0.06 * span
    scale = size / (span + 2 * pad)

    def px(p):
        x = (p[:, 0] - lo[0] + pad + (span - (hi[0] - lo[0])) / 2) * scale
        y = size - (p[:, 1] - lo[1] + pad + (span - (hi[1] - lo[1])) / 2) * scale
        return np.column_stack([x, y])

    parts = ["<g>"]
    if title:
        parts.append(
            f'<text x="{size/2:.0f}" y="16" text-anchor="middle" '
            f'fill="#c0caf5" font-size="13" font-family="sans-serif">{title}</text>'
        )
    for i in range(len(tri)):
        base = BASE_RGB if facing[i] else BACK_TINT
        r, g, b = (int(255 * min(1.0, c * shade[i])) for c in base)
        p = px(tri[i, :, :2])
        pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in p)
        parts.append(
            f'<polygon points="{pts}" fill="rgb({r},{g},{b})" '
            f'fill-opacity="{alpha}" stroke="rgb({r},{g},{b})" '
            f'stroke-opacity="{min(1.0, alpha + 0.15)}" stroke-width="0.4"/>'
        )
    parts.append("</g>")
    return "\n".join(parts)


def svg_document(fragments: list[str], cols: int, cell: int) -> str:
    rows = (len(fragments) + cols - 1) // cols
    w, h = cols * cell, rows * cell
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">'
    ]
    for i, frag in enumerate(fragments):
        x, y = (i % cols) * cell, (i // cols) * cell
        out.append(f'<g transform="translate({x},{y})">{frag}</g>')
    out.append("</svg>")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("inputs", nargs="+", help=".SLDPRT or .x_b files")
    ap.add_argument("-o", "--out", default="render.svg")
    ap.add_argument("--alpha", type=float, default=0.55)
    ap.add_argument("--elev", type=float, default=28.0)
    ap.add_argument("--azim", type=float, default=-55.0)
    ap.add_argument("--size", type=int, default=520, help="px per part cell")
    ap.add_argument("--tol", type=float, default=None)
    args = ap.parse_args()

    frags = []
    for path in args.inputs:
        mesh = mesh_file(path, args.tol)
        frags.append(
            render_mesh_svg(mesh, alpha=args.alpha, elev=args.elev,
                            azim=args.azim, size=args.size,
                            title=Path(path).stem)
        )
        print(f"{path}: {len(mesh.triangles)} triangles rendered")
    cols = 2 if len(frags) > 1 else 1
    Path(args.out).write_text(svg_document(frags, cols, args.size))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
