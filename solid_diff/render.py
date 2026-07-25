"""Painter's-algorithm SVG renderer for tessellated parts.

Triangles are drawn back-to-front as semi-transparent shaded polygons so
interior structure shows through. Ordering is exact via a BSP tree (crossing
polygons are split) up to a size budget, falling back to centroid depth sort
for very large meshes. Real per-face colors (SDL/TYSA_COLOUR) are used when
present, with a per-face override hook for diff rendering. Feature edges
(sharp dihedrals, open boundaries) are stroked in paint order.

Usage:
  python -m solid_diff.render part.SLDPRT [more ...] [-o out.svg]
     [--alpha A] [--elev DEG] [--azim DEG] [--fov DEG] [--size PX]
     [--cols N] [--order auto|bsp|depth] [--tol M]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .brep2mesh import mesh_file
from .tess import Mesh

BASE_RGB = np.array([0.48, 0.64, 0.97])  # portal-friendly blue
KEY_LIGHT = np.array([0.45, 0.5, 0.75])
FILL_LIGHT = np.array([-0.6, -0.2, 0.35])
BSP_BUDGET = 12000  # triangles; beyond this exact ordering costs too much
FEATURE_ANGLE_DEG = 28.0


# ── Geometry helpers ─────────────────────────────────────────────────────────


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


class _Poly:
    """A convex polygon fragment carried through the BSP."""

    __slots__ = ("pts", "normal", "color", "edges")

    def __init__(self, pts, normal, color, edges):
        self.pts = pts          # (k,3) camera-space points
        self.normal = normal    # unit normal (camera space)
        self.color = color      # base rgb
        self.edges = edges      # list of (i, j) vertex-index pairs to stroke


def _split_poly(poly: _Poly, pn, pd, eps):
    """Split a convex polygon by plane (pn·x = pd) into (front, back)."""
    d = poly.pts @ pn - pd
    front_pts, back_pts = [], []
    fmap, bmap = {}, {}  # original vertex index -> new index (for edges)
    n = len(poly.pts)
    for i in range(n):
        j = (i + 1) % n
        di, dj = d[i], d[j]
        if di >= -eps:
            fmap[i] = len(front_pts)
            front_pts.append(poly.pts[i])
        if di <= eps:
            bmap[i] = len(back_pts)
            back_pts.append(poly.pts[i])
        if (di > eps and dj < -eps) or (di < -eps and dj > eps):
            t = di / (di - dj)
            x = poly.pts[i] + t * (poly.pts[j] - poly.pts[i])
            front_pts.append(x)
            back_pts.append(x)
    out = []
    for pts, vmap in ((front_pts, fmap), (back_pts, bmap)):
        if len(pts) >= 3:
            edges = [(vmap[a], vmap[b]) for a, b in poly.edges
                     if a in vmap and b in vmap]
            out.append(_Poly(np.asarray(pts), poly.normal, poly.color, edges))
        else:
            out.append(None)
    return out[0], out[1]


class _BspNode:
    __slots__ = ("pn", "pd", "coplanar", "front", "back")


def _build_bsp(polys, eps, depth=0):
    if not polys:
        return None
    node = _BspNode()
    splitter = polys[len(polys) // 2]
    node.pn = splitter.normal
    node.pd = float(node.pn @ splitter.pts[0])
    node.coplanar = [splitter]
    front, back = [], []
    for p in polys[: len(polys) // 2] + polys[len(polys) // 2 + 1:]:
        d = p.pts @ node.pn - node.pd
        if np.all(np.abs(d) <= eps):
            node.coplanar.append(p)
        elif np.all(d >= -eps):
            front.append(p)
        elif np.all(d <= eps):
            back.append(p)
        else:
            f, b = _split_poly(p, node.pn, node.pd, eps)
            if f is not None:
                front.append(f)
            if b is not None:
                back.append(b)
    node.front = _build_bsp(front, eps, depth + 1)
    node.back = _build_bsp(back, eps, depth + 1)
    return node


def _traverse_bsp(node, eye_dir, out):
    """Back-to-front traversal for an orthographic view direction."""
    if node is None:
        return
    if node.pn @ eye_dir < 0:
        _traverse_bsp(node.front, eye_dir, out)
        out.extend(node.coplanar)
        _traverse_bsp(node.back, eye_dir, out)
    else:
        _traverse_bsp(node.back, eye_dir, out)
        out.extend(node.coplanar)
        _traverse_bsp(node.front, eye_dir, out)


# ── Rendering ────────────────────────────────────────────────────────────────


def _feature_edges(mesh: Mesh):
    """Per-triangle vertex-index pairs lying on sharp or open edges."""
    v, t = mesh.vertices, mesh.triangles
    n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    owner: dict = {}
    for ti, tri in enumerate(t):
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            owner.setdefault((min(a, b), max(a, b)), []).append(ti)
    cos_lim = np.cos(np.radians(FEATURE_ANGLE_DEG))
    marks = [[] for _ in range(len(t))]
    for (a, b), tris in owner.items():
        sharp = (len(tris) == 1
                 or (len(tris) == 2 and float(n[tris[0]] @ n[tris[1]]) < cos_lim)
                 or (len(tris) == 2
                     and mesh.face_ids[tris[0]] != mesh.face_ids[tris[1]]
                     and float(n[tris[0]] @ n[tris[1]]) < 0.9995))
        if sharp:
            for ti in tris:
                loc = [int(np.where(t[ti] == a)[0][0]), int(np.where(t[ti] == b)[0][0])]
                marks[ti].append((loc[0], loc[1]))
    return marks


def render_mesh_svg(
    mesh: Mesh,
    alpha: float = 0.55,
    elev: float = 28.0,
    azim: float = -55.0,
    fov: float | None = None,
    size: int = 520,
    title: str | None = None,
    color_map: dict | None = None,
    order: str = "auto",
    edges: bool = True,
) -> str:
    """Render one mesh to an SVG <g> fragment of `size`x`size` px.

    color_map overrides base colors per FACE node id ({face_id: (r,g,b)}),
    the hook diff rendering uses to paint added/removed/unchanged faces.
    """
    v, t = mesh.vertices, mesh.triangles
    if not len(t):
        return f'<g><text y="20" fill="#f7768e">empty mesh: {title}</text></g>'

    M = _view_matrix(elev, azim)
    pv = v @ M.T  # camera space: x right, y up, z towards scene

    marks = _feature_edges(mesh) if edges else [[] for _ in range(len(t))]
    polys = []
    for ti, tri in enumerate(t):
        pts = pv[tri]
        nrm = np.cross(pts[1] - pts[0], pts[2] - pts[0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-30:
            continue
        fid = int(mesh.face_ids[ti])
        base = None
        if color_map is not None:
            base = color_map.get(fid)
        if base is None:
            base = mesh.colors.get(fid, BASE_RGB)
        polys.append(_Poly(pts, nrm / ln, np.asarray(base, dtype=float),
                           marks[ti]))

    scene_scale = float(np.abs(pv - pv.mean(axis=0)).max()) or 1.0
    use_bsp = order == "bsp" or (order == "auto" and len(polys) <= BSP_BUDGET)
    if use_bsp:
        root = _build_bsp(polys, eps=1e-9 * scene_scale)
        ordered: list = []
        _traverse_bsp(root, np.array([0.0, 0.0, 1.0]), ordered)
    else:
        ordered = sorted(polys, key=lambda p: p.pts[:, 2].mean())

    # projection
    if fov:
        span = pv[:, :2].ptp(axis=0).max()
        eye_z = pv[:, 2].min() - span / (2 * np.tan(np.radians(fov) / 2))

        def project(pts):
            f = 1.0 / (pts[:, 2] - eye_z)
            ref = 1.0 / (pv[:, 2].mean() - eye_z)
            return pts[:, :2] * (f / ref)[:, None]
    else:
        def project(pts):
            return pts[:, :2]

    all2d = project(np.vstack([p.pts for p in ordered]))
    lo, hi = all2d.min(axis=0), all2d.max(axis=0)
    span = (hi - lo).max() or 1.0
    pad = 0.06 * span
    scale = size / (span + 2 * pad)

    def px(p2):
        x = (p2[:, 0] - lo[0] + pad + (span - (hi[0] - lo[0])) / 2) * scale
        y = size - (p2[:, 1] - lo[1] + pad + (span - (hi[1] - lo[1])) / 2) * scale
        return np.column_stack([x, y])

    key = _view_matrix(elev, azim) @ (KEY_LIGHT / np.linalg.norm(KEY_LIGHT))
    fill = _view_matrix(elev, azim) @ (FILL_LIGHT / np.linalg.norm(FILL_LIGHT))

    out = ["<g>"]
    if title:
        out.append(
            f'<text x="{size/2:.0f}" y="16" text-anchor="middle" '
            f'fill="#c0caf5" font-size="13" font-family="sans-serif">{title}</text>'
        )
    stroke_alpha = min(1.0, alpha + 0.25)
    for p in ordered:
        facing = p.normal[2] < 0
        nl = p.normal if facing else -p.normal
        shade = 0.30 + 0.55 * max(0.0, float(-nl @ key)) + 0.15 * max(0.0, float(-nl @ fill))
        rgb = p.color * shade
        if not facing:
            rgb = rgb * 0.55 + np.array([0.25, 0.10, 0.30]) * 0.45  # interior tint
        r, g, b = (int(255 * min(1.0, c)) for c in rgb)
        p2 = px(project(p.pts))
        pts_s = " ".join(f"{x:.1f},{y:.1f}" for x, y in p2)
        out.append(
            f'<polygon points="{pts_s}" fill="rgb({r},{g},{b})" '
            f'fill-opacity="{alpha}" stroke="rgb({r},{g},{b})" '
            f'stroke-opacity="{alpha*0.25:.2f}" stroke-width="0.3"/>'
        )
        for a, bb in p.edges:
            if a < len(p2) and bb < len(p2):
                out.append(
                    f'<line x1="{p2[a,0]:.1f}" y1="{p2[a,1]:.1f}" '
                    f'x2="{p2[bb,0]:.1f}" y2="{p2[bb,1]:.1f}" '
                    f'stroke="#e8ecff" stroke-opacity="{stroke_alpha*0.55:.2f}" '
                    f'stroke-width="0.8"/>'
                )
    out.append("</g>")
    return "\n".join(out)


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
    ap.add_argument("--fov", type=float, default=None,
                    help="perspective FOV in degrees (default: orthographic)")
    ap.add_argument("--size", type=int, default=520, help="px per part cell")
    ap.add_argument("--cols", type=int, default=None,
                    help="grid columns (default: ~square)")
    ap.add_argument("--order", choices=("auto", "bsp", "depth"), default="auto")
    ap.add_argument("--no-edges", action="store_true")
    ap.add_argument("--tol", type=float, default=None)
    args = ap.parse_args()

    frags = []
    for path in args.inputs:
        title = Path(path).stem
        try:
            mesh = mesh_file(path, args.tol)
            frags.append(
                render_mesh_svg(mesh, alpha=args.alpha, elev=args.elev,
                                azim=args.azim, fov=args.fov, size=args.size,
                                title=title, order=args.order,
                                edges=not args.no_edges)
            )
            print(f"{path}: {len(mesh.triangles)} triangles rendered")
        except Exception as e:
            print(f"{path}: FAILED: {e}")
            frags.append(
                f'<g><text x="{args.size/2:.0f}" y="{args.size/2:.0f}" '
                f'text-anchor="middle" fill="#f7768e" font-size="12" '
                f'font-family="sans-serif">{title}: failed</text></g>'
            )
    cols = args.cols or max(1, int(np.ceil(np.sqrt(len(frags)))))
    Path(args.out).write_text(svg_document(frags, cols, args.size))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
