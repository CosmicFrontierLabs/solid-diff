"""Tessellate Parasolid XT faces into triangle meshes.

Per face: sample the boundary loops as 3D polylines (edge curves), map them
into the surface's UV space, and triangulate with a topology-universal
inside/outside test — crossing parity against all boundary loops (replicated
across parameter periods), anchored by Parasolid's material-on-the-left loop
convention. One code path covers plain polygons, holes, periodic faces of any
winding configuration, and fully closed surfaces.

Cracks between faces are avoided at the root: edges are sampled once, in 3D,
at the finest density any adjacent face requires (two-pass), so neighboring
faces share identical boundary points and vertex welding closes the mesh.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import Delaunay, QhullError

from .geom import make_curve, make_surface

MAX_EDGE_SAMPLES = 1024
MAX_GRID = 96


# ── Edge sampling (two-pass, shared across faces) ────────────────────────────


def _vertex_point(graph, vertex_ref):
    v = graph.deref(vertex_ref)
    if v is None:
        return None
    p = graph.deref(v.get("point"))
    return np.asarray(p["pvec"], dtype=float) if p else None


class EdgeSampler:
    """Samples edge curves in 3D once, shared by both adjacent faces.

    Pass 1 (tessellate_face dry runs) records the finest spacing each face
    wants via `request()`; pass 2 serves the final samples via `get()`.
    Boundary points are therefore bit-identical on both sides of every edge.
    """

    def __init__(self, graph, tol):
        self.graph = graph
        self.tol = tol
        self._range: dict = {}    # edge id -> (curve, t0, t1) or ("line", a, b)
        self._spacing: dict = {}  # edge id -> requested max 3D spacing
        self._cache: dict = {}

    def _curve_range(self, edge):
        eid = edge["id"]
        if eid in self._range:
            return self._range[eid]
        graph = self.graph
        curve_node = graph.deref(edge.get("curve"))
        he = graph.deref(edge.get("halfedge"))
        he_pos = he if he and he.get("sense") == "+" else (
            graph.deref(he.get("other")) if he else None)
        p_start = _vertex_point(graph, he_pos.get("vertex")) if he_pos else None
        p_end = None
        if he_pos is not None:
            other = graph.deref(he_pos.get("other"))
            p_end = _vertex_point(graph, other.get("vertex")) if other else None

        curve = make_curve(graph, curve_node) if curve_node else None
        if curve is None:
            if p_start is None or p_end is None:
                raise ValueError(f"edge #{eid}: no usable curve or vertices")
            entry = ("line", p_start, p_end)
        else:
            if p_start is None or p_end is None:
                t0, t1 = curve.full_range()
            else:
                t0, t1 = curve.inv(p_start), curve.inv(p_end)
                if curve.periodic and t1 <= t0 + 1e-12:
                    t1 += curve.periodic
                if abs(t1 - t0) < 1e-14:
                    t1 = t0 + (curve.periodic or 0.0)
            entry = (curve, t0, t1, p_start, p_end)
        self._range[eid] = entry
        return entry

    def request(self, edge, spacing):
        eid = edge["id"]
        self._spacing[eid] = min(self._spacing.get(eid, np.inf), spacing)

    def get(self, edge):
        eid = edge["id"]
        spacing = self._spacing.get(eid, np.inf)
        key = (eid, spacing)
        if key in self._cache:
            return self._cache[key]
        entry = self._curve_range(edge)
        if entry[0] == "line":
            _, a, b = entry
            n = max(1, min(64, int(np.ceil(np.linalg.norm(b - a) / spacing))
                           if np.isfinite(spacing) else 1))
            ts = np.linspace(0, 1, n + 1)
            pts = a + np.outer(ts, b - a)
        else:
            curve, t0, t1, p_start, p_end = entry
            n = 8
            while True:
                ts = np.linspace(t0, t1, n + 1)
                pts = curve.eval(ts)
                mids = curve.eval((ts[:-1] + ts[1:]) / 2)
                dev = np.linalg.norm(mids - (pts[:-1] + pts[1:]) / 2, axis=1).max()
                seg = np.linalg.norm(np.diff(pts, axis=0), axis=1).max()
                if (dev <= self.tol and seg <= spacing) or n >= MAX_EDGE_SAMPLES:
                    break
                n *= 2
            if p_start is not None:
                pts[0], pts[-1] = p_start, p_end
        self._cache[key] = pts
        return pts


def loop_polyline(graph, sampler, loop, warn):
    """Closed 3D polyline for a loop, assembled by endpoint continuity."""
    pts_out = None
    eps = max(sampler.tol * 50, 1e-9)
    for he in graph.loop_halfedges(loop):
        edge = graph.deref(he.get("edge"))
        if edge is None:
            continue
        try:
            seg = sampler.get(edge)
        except ValueError as e:
            warn(str(e))
            continue
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


def loop_edges(graph, loop):
    out = []
    for he in graph.loop_halfedges(loop):
        edge = graph.deref(he.get("edge"))
        if edge is not None:
            out.append(edge)
    return out


# ── Universal UV inside/outside classifier ───────────────────────────────────


class LoopClassifier:
    """Inside/outside test by crossing parity against all boundary segments.

    Segments are replicated across parameter periods; a query point q is
    classified by counting crossings of the straight segment anchor→q against
    a reference anchor of known insideness. Whenever some parameter direction
    is open, the anchor is placed beyond the loops' extent there — provably
    outside — making the test exact and independent of loop orientation
    (which survives neither Parasolid's conventions nor loop assembly
    reliably). Only doubly-periodic surfaces fall back to the material-left
    heuristic.
    """

    def __init__(self, loops_uv, period_u, period_v, left_is_inside, scale,
                 outside_anchor=None):
        segs = []
        for uv in loops_uv:
            a = uv
            b = np.roll(uv, -1, axis=0).copy()
            # Close winding loops the short way round: the unwrapped last
            # point sits ~one period from the first, so the closing segment
            # targets the first point shifted by the net winding offset.
            close = uv[0].copy()
            for dim, period in ((0, period_u), (1, period_v)):
                if period:
                    close[dim] += period * np.round((uv[-1, dim] - uv[0, dim]) / period)
            b[-1] = close
            segs.append(np.stack([a, b], axis=1))
        segs = np.concatenate(segs, axis=0) if segs else np.zeros((0, 2, 2))
        reps = [segs]
        for du in ((-period_u, period_u) if period_u else ()):
            reps.append(segs + [du, 0])
        base = np.concatenate(reps, axis=0)
        reps = [base]
        for dv in ((-period_v, period_v) if period_v else ()):
            reps.append(base + [0, dv])
        self.segs = np.concatenate(reps, axis=0)
        self.scale = np.asarray(scale, dtype=float)

        if outside_anchor is not None:
            # several offset anchors majority-vote away ray-through-vertex
            # parity errors (the classic point-in-polygon corner case)
            base = np.asarray(outside_anchor, dtype=float)
            self.anchors = [base,
                            base + [0.917 * scale[0], -1.313 * scale[1]],
                            base + [-1.531 * scale[0], -0.717 * scale[1]]]
            self.anchor = base
            self.anchor_inside = False
            return
        self.anchors = None

        # doubly-periodic fallback: anchor just left of a long boundary
        # segment, stepped in by an eps small enough not to overshoot
        self.anchor = None
        self.anchor_inside = True
        if len(segs):
            d = (segs[:, 1] - segs[:, 0]) / self.scale
            lens = np.linalg.norm(d, axis=1)
            for eps_frac in (0.1, 0.03, 0.01, 0.003, 0.001):
                for k in np.argsort(-lens)[:8]:
                    a, b = segs[k]
                    t = (b - a) / self.scale
                    tn = t / max(np.linalg.norm(t), 1e-30)
                    left = np.array([-tn[1], tn[0]]) * self.scale
                    eps = eps_frac * lens[k]
                    cand = (a + b) / 2 + (left * eps if left_is_inside else -left * eps)
                    # the candidate's own segment sits exactly eps away; any
                    # closer segment means we may have overshot a thin face
                    if self._min_dist(cand) > eps * 0.99:
                        self.anchor = cand
                        break
                if self.anchor is not None:
                    break
            if self.anchor is None:
                self.anchor = (segs[0, 0] + segs[0, 1]) / 2  # last resort

    def _min_dist(self, q):
        return float(self.dist(q[None])[0])

    def dist(self, pts):
        """Metric-scaled distance from each point to the nearest boundary segment."""
        pts = np.atleast_2d(pts)
        if not len(self.segs):
            return np.full(len(pts), np.inf)
        a = self.segs[:, 0] / self.scale
        ab = (self.segs[:, 1] - self.segs[:, 0]) / self.scale
        ab2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-30)
        out = np.empty(len(pts))
        chunk = max(1, int(2_000_000 / len(a)))
        for i0 in range(0, len(pts), chunk):
            q = pts[i0:i0 + chunk] / self.scale  # (k,2)
            aq = q[:, None, :] - a[None, :, :]   # (k,m,2)
            t = np.clip(np.einsum("kmj,mj->km", aq, ab) / ab2[None, :], 0, 1)
            d = aq - t[..., None] * ab[None, :, :]
            out[i0:i0 + chunk] = np.sqrt(np.einsum("kmj,kmj->km", d, d).min(axis=1))
        return out

    def inside(self, pts):
        pts = np.atleast_2d(pts)
        if self.anchor is None or not len(self.segs):
            return np.ones(len(pts), dtype=bool)  # no boundary: everything in
        if self.anchors is not None:
            votes = sum(self._inside_from(a, pts).astype(int) for a in self.anchors)
            return votes >= 2
        return self._inside_from(self.anchor, pts)

    def _inside_from(self, q0, pts):
        """Chunked-vectorized parity classification of pts (n,2)."""
        a, b = self.segs[:, 0], self.segs[:, 1]
        s = b - a                      # (m,2)
        qp = a - q0                    # (m,2)
        cross_qp_s = qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]  # (m,)
        out = np.empty(len(pts), dtype=bool)
        chunk = max(1, int(2_000_000 / max(len(a), 1)))
        for i0 in range(0, len(pts), chunk):
            r = pts[i0:i0 + chunk] - q0  # (k,2)
            denom = r[:, 0:1] * s[None, :, 1] - r[:, 1:2] * s[None, :, 0]  # (k,m)
            with np.errstate(divide="ignore", invalid="ignore"):
                t = cross_qp_s[None, :] / denom
                u = (qp[None, :, 0] * r[:, 1:2] - qp[None, :, 1] * r[:, 0:1]) / denom
            hits = (np.abs(denom) > 1e-30) & (t > 0) & (t < 1) & (u >= 0) & (u < 1)
            out[i0:i0 + chunk] = self.anchor_inside ^ (hits.sum(axis=1) & 1).astype(bool)
        return out


# ── Face tessellation ────────────────────────────────────────────────────────


@dataclass
class Mesh:
    vertices: np.ndarray = None
    triangles: np.ndarray = None
    face_ids: np.ndarray = None  # per-triangle source FACE node id
    colors: dict = field(default_factory=dict)  # face id -> rgb
    warnings: list = field(default_factory=list)


class _PlaneShim:
    """Duck-typed Surface over a fitted plane, for unsupported surfaces."""

    period_u = period_v = None
    sense_sign = 1
    v_bounds = None

    def __init__(self, loops3d):
        allp = np.vstack(loops3d)
        self.o = allp.mean(axis=0)
        _, _, vt = np.linalg.svd(allp - self.o, full_matrices=False)
        self.x, self.y, self.n = vt[0], vt[1], np.cross(vt[0], vt[1])

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        return self.o + np.outer(uv[:, 0], self.x) + np.outer(uv[:, 1], self.y)

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.o
        return np.column_stack([q @ self.x, q @ self.y])

    def normal(self, uv, h=0.0):
        return np.tile(self.n, (len(np.atleast_2d(uv)), 1))


def _metric_scale(surf, uv_center):
    h = 1e-5
    c = np.atleast_2d(uv_center)
    du = np.linalg.norm(surf.eval(c + [h, 0]) - surf.eval(c - [h, 0])) / (2 * h)
    dv = np.linalg.norm(surf.eval(c + [0, h]) - surf.eval(c - [0, h])) / (2 * h)
    return np.array([max(du, 1e-12), max(dv, 1e-12)])


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
            uv = np.column_stack([ts[:-1], mid]) if dim == 0 else np.column_stack([mid, ts[:-1]])
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


def _signed_area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _unwrap(uv, period_u, period_v):
    uv = uv.copy()
    for dim, period in ((0, period_u), (1, period_v)):
        if not period:
            continue
        c = uv[:, dim]
        for i in range(1, len(c)):
            c[i] -= period * np.round((c[i] - c[i - 1]) / period)
    return uv


def _face_uv_domain(surf, loops_uv):
    """UV window covering the face: loop extents, or a full period."""
    if loops_uv:
        allv = np.vstack(loops_uv)
        lo, hi = allv.min(axis=0), allv.max(axis=0)
    else:
        lo = np.zeros(2)
        hi = np.zeros(2)
    for dim, period in ((0, surf.period_u), (1, surf.period_v)):
        if period and (not loops_uv or hi[dim] - lo[dim] < period * 0.999):
            # windings (or no loops at all) mean the face spans a full period
            span = hi[dim] - lo[dim]
            if not loops_uv or span < 1e-12 or span >= period * 0.5:
                hi[dim] = lo[dim] + period
    # a near-zero v-span with natural bounds (sphere pole, cone apex) means
    # the face extends towards them; the parity classifier trims the excess
    if surf.v_bounds is not None:
        blo, bhi = surf.v_bounds
        natural = (bhi - blo) if (blo is not None and bhi is not None) else None
        if hi[1] - lo[1] < (0.01 * natural if natural else 1e-9):
            if blo is not None:
                lo[1] = min(lo[1], blo)
            if bhi is not None:
                hi[1] = max(hi[1], bhi)
    return lo, hi


def tessellate_face(graph, face, sampler, tol, warn, dry_run=False):
    loops3d = []
    loops = graph.face_loops(face)
    for loop in loops:
        pl = loop_polyline(graph, sampler, loop, warn)
        if pl is not None:
            loops3d.append(pl)

    surf_node = graph.deref(face["surface"]) if face.get("surface") else None
    surf = make_surface(graph, surf_node) if surf_node else None
    used_shim = False
    if surf is None:
        if not loops3d:
            if not dry_run:
                warn(f"face #{face['id']}: no surface and no loops; skipped")
            return None
        if not dry_run:
            kind = surf_node["node_name"] if surf_node else "?"
            warn(f"face #{face['id']} ({kind}): best-fit-plane fallback")
        surf = _PlaneShim(loops3d)
        used_shim = True

    # map loops to unwrapped UV, then align them all to the first loop's
    # period window (each loop's unwrap base is otherwise arbitrary)
    loops_uv = [_unwrap(surf.inv(pl), surf.period_u, surf.period_v) for pl in loops3d]
    for i in range(1, len(loops_uv)):
        for dim, period in ((0, surf.period_u), (1, surf.period_v)):
            if period:
                delta = loops_uv[i][:, dim].mean() - loops_uv[0][:, dim].mean()
                loops_uv[i][:, dim] -= period * np.round(delta / period)
    if not loops_uv and not (surf.period_u and surf.period_v) and not used_shim:
        if not dry_run:
            warn(f"face #{face['id']}: open surface with no loops; skipped")
        return None

    lo, hi = _face_uv_domain(surf, loops_uv)
    scale = _metric_scale(surf, (lo + hi) / 2)
    su, sv = _grid_step(surf, (lo, hi), tol)

    if dry_run:
        # request boundary sampling at the interior grid density
        spacing = min(su * scale[0], sv * scale[1])
        for loop in loops:
            for edge in loop_edges(graph, loop):
                sampler.request(edge, spacing)
        return None

    face_sign = 1 if face.get("sense", "+") == "+" else -1
    # An anchor placed beyond the loops' extent in any OPEN parameter
    # direction is provably outside the face, making parity classification
    # exact with no dependence on loop orientation. Only doubly-periodic
    # surfaces (torus) lack such a point and use the material-left heuristic.
    outside = None
    if loops_uv:
        allv = np.vstack(loops_uv)
        # If the domain was extended to a natural bound (sphere pole, cone
        # apex), "beyond the loops in v" is outside the parameter domain, not
        # outside the face — the heuristic anchor must be used instead.
        pole_extended = (surf.v_bounds is not None
                         and (allv[:, 1].max() - allv[:, 1].min())
                         < 0.5 * (hi[1] - lo[1]))
        # irrational-ish offsets keep the anchor ray off grid/seam lines,
        # where exact crossings make parity fragile
        if not surf.period_v and not pole_extended:
            outside = np.array([allv[:, 0].mean() + 0.3717 * su,
                                allv[:, 1].min() - 2.637 * sv])
        elif not surf.period_u:
            outside = np.array([allv[:, 0].min() - 2.637 * su,
                                allv[:, 1].mean() + 0.3717 * sv])
    left_inside = (surf.sense_sign * face_sign) > 0
    clf = LoopClassifier(loops_uv, surf.period_u, surf.period_v, left_inside,
                         np.array([su, sv]), outside_anchor=outside)

    pts_3d = []
    for pl in loops3d:
        pts_3d.extend(pl)
    all_uv = np.vstack(loops_uv) if loops_uv else np.zeros((0, 2))
    all_3d = list(pts_3d)

    # interior grid + explicit seam lines for periodic dims
    gu = np.arange(lo[0], hi[0] + su * 0.5, su)
    gv = np.arange(lo[1], hi[1] + sv * 0.5, sv)
    gpts = np.stack(np.meshgrid(gu, gv, indexing="ij"), axis=-1).reshape(-1, 2)
    keep = clf.inside(gpts)
    gpts = gpts[keep]
    if len(gpts) and len(all_uv):
        # drop grid points hugging the boundary: T-junctions arise when a
        # grid point lands ON a boundary segment between loop samples
        gpts = gpts[clf.dist(gpts) > 0.45]
    if len(gpts):
        all_uv = np.vstack([all_uv, gpts]) if len(all_uv) else gpts
        all_3d += [None] * len(gpts)

    if len(all_uv) < 3:
        warn(f"face #{face['id']}: too few points; skipped")
        return None

    try:
        tri = Delaunay(all_uv / [su, sv])
    except QhullError:
        try:
            tri = Delaunay(all_uv / [su, sv], qhull_options="QJ")
        except QhullError:
            warn(f"face #{face['id']}: Delaunay failed; skipped")
            return None
    cent = all_uv[tri.simplices].mean(axis=1)
    keep = clf.inside(cent)
    tris = tri.simplices[keep]
    if not len(tris):
        # an empty face is never right: the anchor most likely landed on the
        # wrong side (thin face); the complement is the best available answer
        tris = tri.simplices[~keep]
        if len(tris):
            warn(f"face #{face['id']}: parity anchor flipped (thin face?)")
        else:
            warn(f"face #{face['id']}: no triangles survived classification")
            return None

    verts3d = np.empty((len(all_uv), 3))
    need = [i for i, p in enumerate(all_3d) if p is None]
    have = [i for i, p in enumerate(all_3d) if p is not None]
    if have:
        verts3d[have] = np.asarray([all_3d[i] for i in have])
    if need:
        verts3d[need] = surf.eval(all_uv[need])

    # orient outward: param normal * surface sense * face sense
    areas2d = np.cross(all_uv[tris[:, 1]] - all_uv[tris[:, 0]],
                       all_uv[tris[:, 2]] - all_uv[tris[:, 0]])
    big = int(np.argmax(np.abs(areas2d)))
    a, b, c = verts3d[tris[big]]
    n_geo = np.cross(b - a, c - a)
    n_out = surf.normal(cent[big:big + 1])[0] * surf.sense_sign * face_sign
    if np.dot(n_geo, n_out) < 0:
        tris = tris[:, ::-1]
    return verts3d, tris


# ── Whole-body driver ────────────────────────────────────────────────────────


def _model_scale(graph):
    pts = []
    for n in graph.nodes.values():
        for key in ("pvec", "centre"):
            v = n.get(key)
            if isinstance(v, (list, tuple)) and len(v) == 3:
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

    sampler = EdgeSampler(graph, tol)
    faces = graph.by_type("FACE")
    for face in faces:  # pass 1: agree on shared edge sampling densities
        try:
            tessellate_face(graph, face, sampler, tol, warn, dry_run=True)
        except Exception:
            pass

    vert_index: dict = {}
    verts: list = []
    tris: list = []
    face_ids: list = []
    weld = max(tol * 1e-3, 1e-12)

    for face in faces:  # pass 2: tessellate for real
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = tessellate_face(graph, face, sampler, tol, warn)
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
