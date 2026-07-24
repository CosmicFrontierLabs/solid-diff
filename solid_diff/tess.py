"""Tessellate Parasolid XT faces into triangle meshes.

Per face: sample the boundary loops as 3D polylines (edge curves), map them
into the surface's UV space, cut periodic surfaces at a seam, triangulate in
UV (scipy Delaunay + point-in-polygon filtering), and map back to 3D. Faces
with unsupported surfaces (e.g. BLENDED_EDGE fillets) fall back to a best-fit
plane projection of their boundary — coarse but present.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import Delaunay, QhullError

from .geom import make_curve, make_surface

MAX_EDGE_SAMPLES = 512
MAX_GRID = 96


# ── Edge / loop sampling ─────────────────────────────────────────────────────


def _vertex_point(graph, vertex_ref):
    v = graph.deref(vertex_ref)
    if v is None:
        return None
    p = graph.deref(v.get("point"))
    return np.asarray(p["pvec"], dtype=float) if p else None


def _adaptive_sample(curve, t0, t1, tol, n0=8):
    n = max(2, n0)
    while True:
        ts = np.linspace(t0, t1, n + 1)
        pts = curve.eval(ts)
        mids = curve.eval((ts[:-1] + ts[1:]) / 2)
        dev = np.linalg.norm(mids - (pts[:-1] + pts[1:]) / 2, axis=1).max()
        if dev <= tol or n >= MAX_EDGE_SAMPLES:
            return pts
        n *= 2


def sample_edge(graph, edge, tol):
    """Ordered 3D samples along an edge, from its '+' halfedge's vertex."""
    curve_node = graph.deref(edge.get("curve"))
    he = graph.deref(edge.get("halfedge"))
    he_pos = he if he and he.get("sense") == "+" else graph.deref(he.get("other")) if he else None
    p_start = _vertex_point(graph, he_pos.get("vertex")) if he_pos else None
    p_end = None
    if he_pos is not None:
        other = graph.deref(he_pos.get("other"))
        p_end = _vertex_point(graph, other.get("vertex")) if other else None

    curve = make_curve(graph, curve_node) if curve_node else None
    if curve is None:
        if p_start is not None and p_end is not None:
            return np.array([p_start, p_end])
        raise ValueError(f"edge #{edge['id']}: no usable curve or vertices")

    if p_start is None or p_end is None:
        t0, t1 = curve.full_range()  # closed edge (full circle etc.)
    else:
        t0, t1 = curve.inv(p_start), curve.inv(p_end)
        if curve.periodic and t1 <= t0 + 1e-12:
            t1 += curve.periodic
        if abs(t1 - t0) < 1e-14:  # closed edge with a single vertex on it
            t1 = t0 + (curve.periodic or 0.0)
    pts = _adaptive_sample(curve, t0, t1, tol)
    # trust exact vertex coordinates at the ends (shared across faces)
    if p_start is not None:
        pts[0], pts[-1] = p_start, p_end
    return pts


def loop_polyline(graph, loop, tol, warn):
    """Closed 3D polyline for a loop, assembled by endpoint continuity."""
    pts_out = None
    eps = max(tol * 50, 1e-9)
    for he in graph.loop_halfedges(loop):
        edge = graph.deref(he.get("edge"))
        if edge is None:
            continue
        seg = sample_edge(graph, edge, tol)
        if he.get("sense") == "-":
            seg = seg[::-1]
        if pts_out is None:
            pts_out = seg
            continue
        tail = pts_out[-1]
        if np.linalg.norm(seg[0] - tail) <= eps:
            pass
        elif np.linalg.norm(seg[-1] - tail) <= eps:
            seg = seg[::-1]
        else:
            warn(f"loop #{loop['id']}: gap {np.linalg.norm(seg[0]-tail):.2e} while chaining")
        pts_out = np.vstack([pts_out[:-1], seg])
    if pts_out is None:
        return None
    if np.linalg.norm(pts_out[0] - pts_out[-1]) <= eps:
        pts_out = pts_out[:-1]
    return pts_out if len(pts_out) >= 3 else None


# ── 2D helpers ───────────────────────────────────────────────────────────────


def _signed_area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _points_in_poly(pts, poly):
    """Vectorized ray-cast: which of pts (n,2) are inside poly (m,2)."""
    x, y = pts[:, 0], pts[:, 1]
    x0, y0 = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
    inside = np.zeros(len(pts), dtype=bool)
    for xa, ya, xb, yb in zip(x0, y0, x1, y1):
        crosses = (ya > y) != (yb > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = xa + (y - ya) * (xb - xa) / (yb - ya)
        inside ^= crosses & (x < xint)
    return inside


def _subdivide_closed(poly, max_len):
    """Insert points so no closed-polygon segment exceeds max_len."""
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        out.append(a)
        seg = np.linalg.norm(b - a)
        k = int(seg // max_len)
        for j in range(1, k + 1):
            out.append(a + (b - a) * j / (k + 1))
    return np.asarray(out)


def _net_winding(u, period):
    """Net winding count of an unwrapped closed polyline coordinate.

    After unwrapping, the closing step (last -> first) is by construction the
    short way around, so the winding is just the unwrapped end-to-end span.
    """
    return int(np.round((u[-1] - u[0]) / period))


# ── Face tessellation ────────────────────────────────────────────────────────


@dataclass
class Mesh:
    vertices: np.ndarray = None
    triangles: np.ndarray = None
    face_ids: np.ndarray = None  # per-triangle source FACE node id
    colors: dict = field(default_factory=dict)  # face id -> rgb
    warnings: list = field(default_factory=list)


def _metric_scales(surf, uv_center):
    h = 1e-5
    du = np.linalg.norm(surf.eval(uv_center + [h, 0]) - surf.eval(uv_center - [h, 0])) / (2 * h)
    dv = np.linalg.norm(surf.eval(uv_center + [0, h]) - surf.eval(uv_center - [0, h])) / (2 * h)
    return max(du, 1e-12), max(dv, 1e-12)


def _grid_step(surf, bbox, tol):
    """Curvature-probed UV grid steps meeting a chordal tolerance."""
    (u0, v0), (u1, v1) = bbox
    steps = []
    for dim, lo, hi in ((0, u0, u1), (1, v0, v1)):
        span = hi - lo
        if span <= 0:
            steps.append(1.0)
            continue
        n = 4
        while n < MAX_GRID:
            ts = np.linspace(lo, hi, n + 1)
            mid = np.full(n, (v0 + v1) / 2 if dim == 0 else (u0 + u1) / 2)
            uv = np.column_stack([ts[:-1], mid[: n]]) if dim == 0 else np.column_stack([mid[:n], ts[:-1]])
            uv2 = uv.copy()
            uv2[:, dim] = ts[1:]
            uvm = uv.copy()
            uvm[:, dim] = (ts[:-1] + ts[1:]) / 2
            dev = np.linalg.norm(surf.eval(uvm) - (surf.eval(uv) + surf.eval(uv2)) / 2, axis=1).max()
            if dev <= tol:
                break
            n *= 2
        steps.append(span / n)
    return steps


def _triangulate_uv(outer, holes, surf, boundary_3d, tol, sense, warn):
    """Triangulate polygon-with-holes in UV; return (verts3d, tris)."""
    if abs(_signed_area(outer)) < 1e-30:
        warn("degenerate UV outer boundary; skipped")
        return None
    bbox = (outer.min(axis=0), outer.max(axis=0))
    su, sv = _grid_step(surf, bbox, tol)
    max_seg = np.array([su, sv]) * 1.5

    def densify(poly, pts3d):
        out_uv, out_3d = [], []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            out_uv.append(a)
            out_3d.append(pts3d[i])
            k = int(np.max(np.abs(b - a) / max_seg))
            for j in range(1, k + 1):
                out_uv.append(a + (b - a) * j / (k + 1))
                out_3d.append(None)  # evaluate later
        return np.asarray(out_uv), out_3d

    all_uv, all_3d = densify(outer, boundary_3d[0])
    hole_polys = []
    for h, h3d in zip(holes, boundary_3d[1:]):
        huv, h3 = densify(h, h3d)
        hole_polys.append(huv)
        all_uv = np.vstack([all_uv, huv])
        all_3d += h3

    # interior grid, clear of the boundary
    (u0, v0), (u1, v1) = bbox
    gu = np.arange(u0 + su / 2, u1, su)
    gv = np.arange(v0 + sv / 2, v1, sv)
    if len(gu) and len(gv):
        gpts = np.stack(np.meshgrid(gu, gv), axis=-1).reshape(-1, 2)
        keep = _points_in_poly(gpts, outer)
        for hp in hole_polys:
            keep &= ~_points_in_poly(gpts, hp)
        gpts = gpts[keep]
        if len(gpts):
            bs = all_uv / [su, sv]
            keep2 = np.empty(len(gpts), dtype=bool)
            for i0 in range(0, len(gpts), 512):  # chunked to bound memory
                gs = gpts[i0 : i0 + 512] / [su, sv]
                d2 = ((gs[:, None, :] - bs[None, :, :]) ** 2).sum(-1)
                keep2[i0 : i0 + 512] = np.sqrt(d2.min(axis=1)) > 0.45
            gpts = gpts[keep2]
        if len(gpts):
            all_uv = np.vstack([all_uv, gpts])
            all_3d += [None] * len(gpts)

    scaled = all_uv / [su, sv]
    try:
        tri = Delaunay(scaled)
    except QhullError:
        warn("Delaunay failed; skipped")
        return None
    cent = all_uv[tri.simplices].mean(axis=1)
    keep = _points_in_poly(cent, outer)
    for hp in hole_polys:
        keep &= ~_points_in_poly(cent, hp)
    tris = tri.simplices[keep]
    if not len(tris):
        warn("no triangles survived polygon filtering")
        return None

    verts3d = np.empty((len(all_uv), 3))
    need = [i for i, p in enumerate(all_3d) if p is None]
    have = [i for i, p in enumerate(all_3d) if p is not None]
    if have:
        verts3d[have] = np.asarray([all_3d[i] for i in have])
    if need:
        verts3d[need] = surf.eval(all_uv[need])

    # Orient triangles outward: the XT face normal is the parametric surface
    # normal flipped by the surface node's sense, then by the face's sense
    # (validated against analytic volumes of the sample parts).
    areas2d = np.cross(
        all_uv[tris[:, 1]] - all_uv[tris[:, 0]], all_uv[tris[:, 2]] - all_uv[tris[:, 0]]
    )
    big = int(np.argmax(np.abs(areas2d)))
    a, b, c = verts3d[tris[big]]
    n_geo = np.cross(b - a, c - a)
    n_out = (surf.normal(cent[big : big + 1])[0] * surf.sense_sign
             * (1 if sense == "+" else -1))
    if np.dot(n_geo, n_out) < 0:
        tris = tris[:, ::-1]
    return verts3d, tris


def _seam_cut(loops_uv, winding_loops, period, warn):
    """Combine two opposite winding loops into one seam-cut outer polygon.

    Think of a cylinder wall: loop A (winding +1) becomes the top edge of a
    rectangle-ish strip spanning one period; loop B (winding -1) the bottom
    edge, traversed in the opposite direction; two vertical seam bridges
    (added implicitly by the polygon closure and the A-end/B-start adjacency)
    close it into a simple polygon.
    """
    ia, ib = winding_loops
    A = loops_uv[ia].copy()
    B = loops_uv[ib].copy()
    if _net_winding(A[:, 0], period) < 0:
        A, B = B, A
    # close A's span: it ascends from u0 to ~u0+period; add the wrapped start
    A = np.vstack([A, A[0] + [period, 0]])
    seam_u = A[-1, 0]
    # rotate B so its first point sits nearest the right seam (mod period)
    rot = int(np.argmin(np.abs(((B[:, 0] - seam_u) + period / 2) % period - period / 2)))
    B = np.roll(B, -rot, axis=0)
    B[0, 0] -= period * np.round((B[0, 0] - seam_u) / period)
    for i in range(1, len(B)):
        B[i, 0] -= period * np.round((B[i, 0] - B[i - 1, 0]) / period)
    if _net_winding(B[:, 0], period) > 0:  # must descend right seam -> left
        B = B[::-1]
        B[:, 0] -= period * np.round((B[0, 0] - seam_u) / period)
    # close B's span at the left seam
    B = np.vstack([B, B[0] - [period, 0]])
    return np.vstack([A, B])


def tessellate_face(graph, face, tol, warn):
    loops3d = []
    for loop in graph.face_loops(face):
        pl = loop_polyline(graph, loop, tol, warn)
        if pl is not None:
            loops3d.append(pl)
    if not loops3d:
        warn(f"face #{face['id']}: no usable loops; skipped")
        return None

    surf_node = graph.deref(face["surface"])
    surf = make_surface(graph, surf_node) if surf_node else None
    if surf is None:
        return _fallback_planar(face, loops3d, surf_node, tol, warn)

    loops_uv, windings = [], []
    for pl in loops3d:
        uv = surf.inv(pl)
        for dim, period in ((0, surf.period_u), (1, surf.period_v)):
            if period:
                c = uv[:, dim]
                for i in range(1, len(c)):
                    c[i] -= period * np.round((c[i] - c[i - 1]) / period)
        w = _net_winding(uv[:, 0], surf.period_u) if surf.period_u else 0
        loops_uv.append(uv)
        windings.append(w)

    winding_idx = [i for i, w in enumerate(windings) if w != 0]
    if winding_idx:
        if len(winding_idx) != 2 or not surf.period_u:
            warn(f"face #{face['id']}: unsupported winding config {windings}; planar fallback")
            return _fallback_planar(face, loops3d, surf_node, tol, warn)
        outer = _seam_cut(loops_uv, winding_idx, surf.period_u, warn)
        hole_idx = [i for i in range(len(loops_uv)) if i not in winding_idx]
        outer_3d = [None] * len(outer)  # seam-cut boundary: re-eval from UV
    else:
        areas = [abs(_signed_area(uv)) for uv in loops_uv]
        oi = int(np.argmax(areas))
        outer = loops_uv[oi]
        outer_3d = list(loops3d[oi])
        hole_idx = [i for i in range(len(loops_uv)) if i != oi]
        # shift holes into the outer's period window
        for i in hole_idx:
            for dim, period in ((0, surf.period_u), (1, surf.period_v)):
                if period:
                    delta = loops_uv[i][:, dim].mean() - outer[:, dim].mean()
                    loops_uv[i][:, dim] -= period * np.round(delta / period)

    holes = [loops_uv[i] for i in hole_idx]
    holes_3d = [list(loops3d[i]) for i in hole_idx]
    result = _triangulate_uv(outer, holes, surf, [outer_3d] + holes_3d, tol,
                             face.get("sense", "+"), warn)
    if result is None:
        return _fallback_planar(face, loops3d, surf_node, tol, warn)
    return result


class _PlaneShim:
    """Duck-typed Surface over a fitted plane, for the fallback path."""

    period_u = period_v = None
    sense_sign = 1

    def __init__(self, origin, x, y, n):
        self.o, self.x, self.y, self.n = origin, x, y, n

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        return self.o + np.outer(uv[:, 0], self.x) + np.outer(uv[:, 1], self.y)

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.o
        return np.column_stack([q @ self.x, q @ self.y])

    def normal(self, uv, h=0.0):
        return np.tile(self.n, (len(np.atleast_2d(uv)), 1))


def _fallback_planar(face, loops3d, surf_node, tol, warn):
    kind = surf_node["node_name"] if surf_node else "?"
    warn(f"face #{face['id']} ({kind}): best-fit-plane fallback")
    allp = np.vstack(loops3d)
    origin = allp.mean(axis=0)
    _, _, vt = np.linalg.svd(allp - origin, full_matrices=False)
    x, y, n = vt[0], vt[1], vt[2]
    shim = _PlaneShim(origin, x, y, n)
    loops_uv = [shim.inv(pl) for pl in loops3d]
    areas = [abs(_signed_area(uv)) for uv in loops_uv]
    oi = int(np.argmax(areas))
    holes = [loops_uv[i] for i in range(len(loops_uv)) if i != oi]
    holes_3d = [list(loops3d[i]) for i in range(len(loops3d)) if i != oi]
    return _triangulate_uv(loops_uv[oi], holes, shim, [list(loops3d[oi])] + holes_3d,
                           max(tol, 1e-9), face.get("sense", "+"), warn)


# ── Whole-body driver ────────────────────────────────────────────────────────


def _model_scale(graph):
    pts = []
    for n in graph.nodes.values():
        for key in ("pvec", "centre"):
            v = n.get(key)
            if isinstance(v, list) and len(v) == 3:
                pts.append(v)
    if len(pts) < 2:
        return 1.0
    pts = np.asarray(pts)
    return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))) or 1.0


def tessellate(graph, tol: float | None = None) -> Mesh:
    """Tessellate every FACE in the graph into one welded triangle mesh."""
    mesh = Mesh()
    scale = _model_scale(graph)
    if tol is None:
        tol = 2e-3 * scale

    def warn(msg):
        mesh.warnings.append(msg)

    vert_index: dict = {}
    verts: list = []
    tris: list = []
    face_ids: list = []
    weld = max(tol * 1e-3, 1e-12)

    for face in graph.by_type("FACE"):
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = tessellate_face(graph, face, tol, warn)
        except Exception as e:  # never let one face kill the body
            warn(f"face #{face['id']}: error: {e}")
            result = None
        if result is None:
            continue
        v3, t = result
        color = graph.face_color(face)
        if color:
            mesh.colors[face["id"]] = color
        remap = np.empty(len(v3), dtype=int)
        for i, p in enumerate(v3):
            key = tuple(np.round(p / weld).astype(np.int64))
            if key not in vert_index:
                vert_index[key] = len(verts)
                verts.append(p)
            remap[i] = vert_index[key]
        for tri_ in remap[t]:
            if len(set(tri_)) == 3:
                tris.append(tri_)
                face_ids.append(face["id"])

    mesh.vertices = np.asarray(verts) if verts else np.zeros((0, 3))
    mesh.triangles = np.asarray(tris, dtype=int) if tris else np.zeros((0, 3), dtype=int)
    mesh.face_ids = np.asarray(face_ids, dtype=int) if face_ids else np.zeros(0, dtype=int)
    return mesh


# ── Writers ──────────────────────────────────────────────────────────────────


def write_obj(mesh: Mesh, path: str):
    with open(path, "w") as f:
        f.write("# solid-diff brep2mesh\n")
        for v in mesh.vertices:
            f.write(f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n")
        last_fid = None
        for tri, fid in zip(mesh.triangles, mesh.face_ids):
            if fid != last_fid:
                f.write(f"g face_{fid}\n")
                last_fid = fid
            f.write(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}\n")


def write_stl(mesh: Mesh, path: str):
    import struct

    v, t = mesh.vertices, mesh.triangles
    n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.where(norm > 0, norm, 1)
    with open(path, "wb") as f:
        f.write(b"solid-diff brep2mesh".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(t)))
        for i, tri in enumerate(t):
            f.write(struct.pack("<3f", *n[i]))
            for vi in tri:
                f.write(struct.pack("<3f", *v[vi]))
            f.write(struct.pack("<H", 0))
