//! Triangle mesh produced by tessellation, plus OBJ/STL writers.

use std::collections::HashMap;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::geom::{cross, norm, scale, sub, P3};
use crate::value::NodeId;

/// Does the +x ray from `o` pass through triangle `abc`?
///
/// Half-open on the projected edges (the same convention the UV classifier
/// uses) so a ray grazing a shared edge is counted by exactly one of the two
/// triangles that share it.
fn ray_x_hits(o: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> bool {
    // Project to the yz plane and do a 2D point-in-triangle with orientation
    // tests; then require the crossing to be ahead of the origin in x.
    let sign = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| -> f64 {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    let p = [o[1], o[2]];
    let (pa, pb, pc) = ([a[1], a[2]], [b[1], b[2]], [c[1], c[2]]);
    let (d1, d2, d3) = (sign(p, pa, pb), sign(p, pb, pc), sign(p, pc, pa));
    let strictly_in = (d1 > 0.0 && d2 > 0.0 && d3 > 0.0) || (d1 < 0.0 && d2 < 0.0 && d3 < 0.0);
    if !strictly_in {
        return false;
    }
    // Barycentric interpolation of the crossing's x.
    let area = sign(pa, pb, pc);
    if area == 0.0 {
        return false;
    }
    let (wa, wb, wc) = (d2 / area, d3 / area, d1 / area);
    let x = wa * a[0] + wb * b[0] + wc * c[0];
    x > o[0]
}

#[derive(Default)]
pub struct Mesh {
    pub vertices: Vec<P3>,
    pub triangles: Vec<[u32; 3]>,
    /// Source FACE node id per triangle.
    pub face_ids: Vec<NodeId>,
    /// Face colour by FACE node id, when the file carries one.
    pub colors: HashMap<NodeId, [f64; 3]>,
    pub warnings: Vec<String>,
}

impl Mesh {
    /// Make neighbouring triangles agree on winding, then point each closed
    /// piece outwards.
    ///
    /// Winding is decided per face during tessellation, from the parametric
    /// normal times the surface and face senses. That rule is right for the
    /// analytic surfaces but depends on the handedness of the parameterisation,
    /// and for a NURBS surface the handedness is whatever the control net
    /// happens to give -- so neighbouring faces disagreed and roughly half of
    /// all shared edges on NURBS-heavy parts came out wound opposite (#21).
    ///
    /// Consistency does not need any of that. Two triangles sharing an edge
    /// agree exactly when they traverse it in opposite directions, which is a
    /// purely combinatorial test. Flood filling that relation orients each
    /// connected piece up to one global sign per piece.
    ///
    /// The remaining sign per piece follows a convention, not a vote: the
    /// embedding decides. A component at even nesting depth is an outer
    /// boundary and must enclose positive volume; one at odd depth is the
    /// wall of a cavity and must enclose negative volume. Depth is measured
    /// by ray parity against the *other* components, so no part of this
    /// trusts the per-face winding rule at all -- an earlier version voted on
    /// that rule and was majority-wrong on a part whose faces mostly wound
    /// inward, shipping it inside out.
    pub fn orient(&mut self) {
        if self.triangles.is_empty() {
            return;
        }
        // Undirected edge -> the triangles using it, with the direction each
        // one traverses it in.
        let mut users: HashMap<(u32, u32), Vec<(usize, bool)>> = HashMap::new();
        for (ti, t) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                users.entry(key).or_default().push((ti, a < b));
            }
        }

        let n = self.triangles.len();
        let mut component = vec![usize::MAX; n];
        let mut flip = vec![false; n];
        let mut components = 0usize;

        for seed in 0..n {
            if component[seed] != usize::MAX {
                continue;
            }
            component[seed] = components;
            let mut stack = vec![seed];
            while let Some(ti) = stack.pop() {
                let t = self.triangles[ti];
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    let key = if a < b { (a, b) } else { (b, a) };
                    // Direction this triangle traverses the edge, accounting
                    // for a flip already scheduled for it.
                    let dir = (a < b) != flip[ti];
                    let Some(list) = users.get(&key) else {
                        continue;
                    };
                    // Only propagate across manifold edges. Where three or
                    // more triangles meet there is no single correct partner,
                    // and guessing would spread one arbitrary choice.
                    if list.len() != 2 {
                        continue;
                    }
                    for &(tj, dj) in list {
                        if tj == ti {
                            continue;
                        }
                        let want = !dir; // neighbours must run it the other way
                        if component[tj] == usize::MAX {
                            component[tj] = components;
                            flip[tj] = dj != want;
                            stack.push(tj);
                        }
                    }
                }
            }
            components += 1;
        }

        // Apply the consistency flips first, so each component is a coherent
        // 2-manifold whose signed volume means something.
        for (t, f) in self.triangles.iter_mut().zip(&flip) {
            if *f {
                t.swap(1, 2);
            }
        }

        // Signed volume and boundary count per component.
        let mut vol = vec![0.0f64; components];
        let mut open_edges = vec![0usize; components];
        let mut tris_per = vec![0usize; components];
        for (ti, t) in self.triangles.iter().enumerate() {
            let (a, b, c) = (
                self.vertices[t[0] as usize],
                self.vertices[t[1] as usize],
                self.vertices[t[2] as usize],
            );
            vol[component[ti]] += crate::geom::dot(a, cross(b, c)) / 6.0;
            tris_per[component[ti]] += 1;
        }
        for list in users.values() {
            if list.len() == 1 {
                open_edges[component[list[0].0]] += 1;
            }
        }

        // Nesting depth of each component: parity of ray crossings against
        // every OTHER component. The ray leaves from just outside the
        // component's own +x extreme, pointing further +x, so it cannot hit
        // its own surface and leaves the scene cleanly.
        let mut seed_pt = vec![[f64::NEG_INFINITY; 3]; components];
        for (ti, t) in self.triangles.iter().enumerate() {
            let ci = component[ti];
            for vi in t {
                let v = self.vertices[*vi as usize];
                if v[0] > seed_pt[ci][0] {
                    seed_pt[ci] = v;
                }
            }
        }
        let mut depth = vec![0usize; components];
        for ci in 0..components {
            // Nudge past the surface; scale-free epsilon from the seed itself.
            let eps = 1e-9 * (1.0 + seed_pt[ci][0].abs());
            let origin = [seed_pt[ci][0] + eps, seed_pt[ci][1], seed_pt[ci][2]];
            let mut crossings = 0usize;
            for (ti, t) in self.triangles.iter().enumerate() {
                if component[ti] == ci {
                    continue;
                }
                let (a, b, c) = (
                    self.vertices[t[0] as usize],
                    self.vertices[t[1] as usize],
                    self.vertices[t[2] as usize],
                );
                if ray_x_hits(origin, a, b, c) {
                    crossings += 1;
                }
            }
            depth[ci] = crossings % 2;
        }

        // The convention: outward at even depth, inward at odd. Components
        // that are substantially open, or enclose no volume to speak of, make
        // no claim and keep the orientation consistency gave them.
        let (blo, bhi) = self.bounds();
        let scale3 = (0..3)
            .map(|i| bhi[i] - blo[i])
            .fold(0.0f64, f64::max)
            .powi(3)
            .max(1e-300);
        for (t, ci) in self.triangles.iter_mut().zip(&component) {
            if open_edges[*ci] * 10 >= tris_per[*ci].max(1) {
                continue; // open sheet: volume is meaningless
            }
            if vol[*ci].abs() < scale3 * 1e-9 {
                continue; // degenerate enclosure
            }
            let want_positive = depth[*ci] == 0;
            if (vol[*ci] > 0.0) != want_positive {
                t.swap(1, 2);
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// Overlay another body's mesh onto this one. FACE node ids are per-body
    /// and can collide across bodies; that only perturbs diff colouring, and
    /// each body arrives already oriented.
    pub fn merge(&mut self, other: Mesh) {
        let off = self.vertices.len() as u32;
        self.vertices.extend(other.vertices);
        self.triangles
            .extend(other.triangles.into_iter().map(|t| t.map(|i| i + off)));
        self.face_ids.extend(other.face_ids);
        self.colors.extend(other.colors);
        self.warnings.extend(other.warnings);
    }

    /// Number of edges used by exactly one triangle (0 == closed surface).
    pub fn boundary_edge_count(&self) -> usize {
        let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
        for t in &self.triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *counts.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        counts.values().filter(|c| **c == 1).count()
    }

    /// Signed volume via the divergence theorem; meaningful when closed and
    /// consistently oriented.
    pub fn signed_volume(&self) -> f64 {
        let mut v = 0.0;
        for t in &self.triangles {
            let (a, b, c) = (
                self.vertices[t[0] as usize],
                self.vertices[t[1] as usize],
                self.vertices[t[2] as usize],
            );
            v += crate::geom::dot(a, cross(b, c));
        }
        v / 6.0
    }

    pub fn bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for v in &self.vertices {
            for i in 0..3 {
                lo[i] = lo[i].min(v[i]);
                hi[i] = hi[i].max(v[i]);
            }
        }
        (lo, hi)
    }

    pub fn write_obj(&self, path: &Path) -> std::io::Result<()> {
        let mut f = BufWriter::new(std::fs::File::create(path)?);
        writeln!(f, "# solid-diff brep2mesh")?;
        for v in &self.vertices {
            writeln!(f, "v {:.9} {:.9} {:.9}", v[0], v[1], v[2])?;
        }
        let mut last: Option<NodeId> = None;
        for (t, fid) in self.triangles.iter().zip(&self.face_ids) {
            if last != Some(*fid) {
                writeln!(f, "g face_{}", fid)?;
                last = Some(*fid);
            }
            writeln!(f, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1)?;
        }
        Ok(())
    }

    pub fn write_stl(&self, path: &Path) -> std::io::Result<()> {
        let mut f = BufWriter::new(std::fs::File::create(path)?);
        let mut header = [0u8; 80];
        let tag = b"solid-diff brep2mesh";
        header[..tag.len()].copy_from_slice(tag);
        f.write_all(&header)?;
        f.write_all(&(self.triangles.len() as u32).to_le_bytes())?;
        for t in &self.triangles {
            let (a, b, c) = (
                self.vertices[t[0] as usize],
                self.vertices[t[1] as usize],
                self.vertices[t[2] as usize],
            );
            let n = cross(sub(b, a), sub(c, a));
            let ln = norm(n);
            let n = if ln > 0.0 { scale(n, 1.0 / ln) } else { n };
            for v in [n, a, b, c] {
                for comp in v {
                    f.write_all(&(comp as f32).to_le_bytes())?;
                }
            }
            f.write_all(&0u16.to_le_bytes())?;
        }
        Ok(())
    }
}

impl Mesh {
    /// Total triangle area. Orientation-independent, so it compares meshes
    /// fairly even when one of them is open.
    pub fn surface_area(&self) -> f64 {
        let mut a = 0.0;
        for t in &self.triangles {
            let (p, q, r) = (
                self.vertices[t[0] as usize],
                self.vertices[t[1] as usize],
                self.vertices[t[2] as usize],
            );
            a += 0.5 * norm(cross(sub(q, p), sub(r, p)));
        }
        a
    }

    /// Bounding-box diagonal length.
    pub fn bbox_diagonal(&self) -> f64 {
        let (lo, hi) = self.bounds();
        if !lo[0].is_finite() {
            return 0.0;
        }
        norm(sub(hi, lo))
    }
}

/// Edge accounting for a rendered mesh, by the directed-edge rule.
///
/// This is a rendering tool, so the report exists to answer one question:
/// where will the picture be wrong? An open edge is a place you can see
/// through the part. The other counts are kept because they explain *why*
/// gaps appear and make regressions attributable, not as goals in
/// themselves.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct EdgeReport {
    pub triangles: usize,
    /// Directed edges with no opposite: a visible gap starts here.
    pub open: usize,
    /// Directed edges traversed the same way by two triangles: the
    /// neighbours disagree on winding. Invisible under two-sided shading.
    pub reversed: usize,
    /// Undirected edges shared by more than two triangles: overlapping
    /// surface, which renders the same as one copy.
    pub overshared: usize,
    /// Triangles with a repeated vertex.
    pub degenerate: usize,
}

impl std::fmt::Display for EdgeReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "edges: {} open, {} reversed, {} shared>2, {} degenerate (of {} triangles)",
            self.open, self.reversed, self.overshared, self.degenerate, self.triangles
        )
    }
}

impl Mesh {
    /// Classify every edge once; see [`EdgeReport`].
    ///
    /// Each undirected edge is classified once, so the counts do not overlap:
    /// used by one triangle is open, by two triangles traversing it the same
    /// way is reversed, by more than two is overshared.
    pub fn edge_report(&self) -> EdgeReport {
        // (total uses, uses in the low->high direction)
        let mut edges: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
        let mut r = EdgeReport {
            triangles: self.triangles.len(),
            ..Default::default()
        };
        for t in &self.triangles {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                r.degenerate += 1;
                continue;
            }
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let e = edges.entry((a.min(b), a.max(b))).or_insert((0, 0));
                e.0 += 1;
                if a < b {
                    e.1 += 1;
                }
            }
        }
        for &(total, forward) in edges.values() {
            match total {
                1 => r.open += 1,
                2 => {
                    // Both uses in the same direction: the two triangles
                    // sharing this edge are wound opposite to each other.
                    if forward != 1 {
                        r.reversed += 1;
                    }
                }
                _ => r.overshared += 1,
            }
        }
        r
    }
}

#[cfg(test)]
mod edge_report_tests {
    #[test]
    fn merge_overlays_bodies_with_offset_indices() {
        let mut a = Mesh {
            vertices: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            triangles: vec![[0, 1, 2]],
            face_ids: vec![7],
            ..Default::default()
        };
        let b = Mesh {
            vertices: vec![[5.0; 3], [6.0, 5.0, 5.0], [5.0, 6.0, 5.0]],
            triangles: vec![[0, 1, 2]],
            face_ids: vec![9],
            ..Default::default()
        };
        a.merge(b);
        assert_eq!(a.vertices.len(), 6);
        assert_eq!(a.triangles, vec![[0, 1, 2], [3, 4, 5]]);
        assert_eq!(a.face_ids, vec![7, 9]);
    }

    use super::*;

    /// Closed, consistently-wound tetrahedron.
    fn tetra() -> Mesh {
        Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            face_ids: vec![1, 2, 3, 4],
            ..Default::default()
        }
    }

    #[test]
    fn closed_solid_has_no_defective_edges() {
        let r = tetra().edge_report();
        assert_eq!(
            r,
            EdgeReport {
                triangles: 4,
                ..Default::default()
            },
            "{r}"
        );
    }

    #[test]
    fn a_missing_triangle_shows_up_as_open_edges() {
        let mut m = tetra();
        m.triangles.pop();
        m.face_ids.pop();
        let r = m.edge_report();
        assert_eq!(r.open, 3, "the three edges of the missing face: {r}");
        assert_eq!(r.reversed, 0);
    }

    #[test]
    fn a_reversed_neighbour_is_caught_though_no_edge_is_missing() {
        // Undirected counting cannot see this: every edge still has exactly
        // two users. Only the direction reveals the flip.
        let mut m = tetra();
        m.triangles[3] = [1, 3, 2];
        let r = m.edge_report();
        assert_eq!(
            r.reversed, 3,
            "3 edges now traversed the same way twice: {r}"
        );
        assert_eq!(
            m.boundary_edge_count(),
            0,
            "the naive check calls this closed"
        );
    }

    #[test]
    fn overshared_and_degenerate_are_reported() {
        let mut m = tetra();
        m.vertices.push([1.0, 1.0, 1.0]);
        m.triangles.push([0, 1, 4]); // a fin on an existing edge
        m.face_ids.push(5);
        m.triangles.push([2, 2, 3]); // degenerate
        m.face_ids.push(6);
        let r = m.edge_report();
        assert_eq!(r.degenerate, 1, "{r}");
        assert!(r.overshared >= 1, "edge 0-1 now has three users: {r}");
    }
}

#[cfg(test)]
mod orient_tests {
    use super::*;

    fn tetra() -> Mesh {
        Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            face_ids: vec![1, 2, 3, 4],
            ..Default::default()
        }
    }

    #[test]
    fn one_reversed_face_is_repaired() {
        let mut m = tetra();
        m.triangles[2].swap(1, 2);
        assert_eq!(m.edge_report().reversed, 3, "setup: three bad edges");
        m.orient();
        let r = m.edge_report();
        assert_eq!(r.reversed, 0, "orient must make neighbours agree: {r}");
        assert_eq!(r.open, 0, "{r}");
    }

    #[test]
    fn an_already_correct_mesh_is_left_alone() {
        let before = tetra();
        let mut after = tetra();
        after.orient();
        assert_eq!(before.triangles, after.triangles);
        assert!(after.signed_volume() > 0.0);
    }

    #[test]
    fn a_lone_inverted_shell_is_turned_right_side_out() {
        // A mesh that is one closed component with negative volume can only
        // be inside out: a genuine void needs an enclosing shell, and there
        // is none here. This is the exact failure seen on a vault part where
        // the per-face rule was wrong for MOST faces and the majority vote
        // dutifully inverted the whole body.
        let mut m = tetra();
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
        assert!(m.signed_volume() < 0.0);
        m.orient();
        assert_eq!(m.edge_report().reversed, 0);
        assert!(
            m.signed_volume() > 0.0,
            "a lone closed shell must enclose positive volume"
        );
    }

    #[test]
    fn consistency_then_convention_fixes_the_sign() {
        // Three of four faces reversed: flood fill makes the shell
        // consistent, then the nesting convention turns it outward. No vote
        // anywhere -- the embedding decides.
        let mut m = tetra();
        for i in [0, 1, 2] {
            m.triangles[i].swap(1, 2);
        }
        m.orient();
        assert_eq!(m.edge_report().reversed, 0);
        assert!(m.signed_volume() > 0.0);
    }

    #[test]
    fn a_void_inside_a_shell_stays_a_void() {
        // Outer tetra plus a smaller inverted tetra inside it: the global
        // flip must act on the TOTAL volume (positive here), leaving the
        // void's orientation alone. This is why the correction is global
        // rather than per component.
        let mut m = tetra();
        let base = m.vertices.len() as u32;
        for v in tetra().vertices {
            m.vertices
                .push([0.25 + v[0] * 0.2, 0.25 + v[1] * 0.2, 0.25 + v[2] * 0.2]);
        }
        for t in tetra().triangles {
            // inverted: a void's normals point into the enclosed material
            m.triangles.push([base + t[0], base + t[2], base + t[1]]);
            m.face_ids.push(9);
        }
        let before = m.signed_volume();
        assert!(before > 0.0, "outer minus void is still positive");
        m.orient();
        assert!(
            (m.signed_volume() - before).abs() < 1e-12,
            "orientation of a consistent outer+void pair must not change"
        );
    }
}
