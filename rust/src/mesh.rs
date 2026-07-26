//! Triangle mesh produced by tessellation, plus OBJ/STL writers.

use std::collections::HashMap;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::geom::{cross, norm, scale, sub, P3};
use crate::value::NodeId;

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
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
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

/// Result of the directed-edge manifold check.
///
/// In a closed, consistently-oriented triangle mesh every directed edge
/// `a -> b` appears exactly once, and its opposite `b -> a` appears exactly
/// once on the neighbouring triangle. Checking direction rather than just
/// counting undirected edges catches three distinct defects at once: holes,
/// non-manifold junctions, and neighbours whose winding disagrees.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct ManifoldReport {
    pub triangles: usize,
    /// Directed edges with no opposite: the mesh has a hole here.
    pub boundary: usize,
    /// Directed edges traversed the same way by two triangles, i.e. the two
    /// share an edge but wind oppositely — one of them is flipped.
    pub flipped: usize,
    /// Undirected edges shared by more than two triangles.
    pub non_manifold: usize,
    /// Triangles with a repeated vertex.
    pub degenerate: usize,
}

impl ManifoldReport {
    /// A closed, consistently-oriented, manifold surface.
    pub fn is_watertight(&self) -> bool {
        self.boundary == 0 && self.flipped == 0 && self.non_manifold == 0 && self.degenerate == 0
    }
}

impl std::fmt::Display for ManifoldReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_watertight() {
            return write!(f, "watertight ({} triangles)", self.triangles);
        }
        write!(
            f,
            "not watertight: {} boundary, {} flipped, {} non-manifold, {} degenerate \
             (of {} triangles)",
            self.boundary, self.flipped, self.non_manifold, self.degenerate, self.triangles
        )
    }
}

impl Mesh {
    /// Check the mesh with the directed-edge rule; see [`ManifoldReport`].
    ///
    /// Each undirected edge is classified once, so the counts do not overlap:
    /// used by one triangle is a hole, by two triangles traversing it the same
    /// way is a winding mismatch, by more than two is non-manifold.
    pub fn manifold_report(&self) -> ManifoldReport {
        // (total uses, uses in the low->high direction)
        let mut edges: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
        let mut r = ManifoldReport {
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
                1 => r.boundary += 1,
                2 => {
                    // Both uses in the same direction: the two triangles
                    // sharing this edge are wound opposite to each other.
                    if forward != 1 {
                        r.flipped += 1;
                    }
                }
                _ => r.non_manifold += 1,
            }
        }
        r
    }
}

#[cfg(test)]
mod manifold_tests {
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
    fn closed_solid_is_watertight() {
        let r = tetra().manifold_report();
        assert!(r.is_watertight(), "{r}");
        assert_eq!(r.triangles, 4);
        assert_eq!(r.boundary, 0);
    }

    #[test]
    fn a_hole_shows_up_as_boundary_edges() {
        let mut m = tetra();
        m.triangles.pop();
        m.face_ids.pop();
        let r = m.manifold_report();
        assert!(!r.is_watertight());
        assert_eq!(r.boundary, 3, "the three edges of the missing face: {r}");
        assert_eq!(r.flipped, 0);
    }

    #[test]
    fn a_reversed_neighbour_is_caught_though_no_edge_is_missing() {
        // Undirected counting cannot see this: every edge still has exactly
        // two users. Only the direction reveals the flip.
        let mut m = tetra();
        m.triangles[3] = [1, 3, 2];
        let r = m.manifold_report();
        assert_eq!(
            r.flipped, 3,
            "3 edges now traversed the same way twice: {r}"
        );
        assert!(!r.is_watertight());
        assert_eq!(
            m.boundary_edge_count(),
            0,
            "the naive check calls this closed"
        );
    }

    #[test]
    fn non_manifold_and_degenerate_are_reported() {
        let mut m = tetra();
        m.vertices.push([1.0, 1.0, 1.0]);
        m.triangles.push([0, 1, 4]); // a fin on an existing edge
        m.face_ids.push(5);
        m.triangles.push([2, 2, 3]); // degenerate
        m.face_ids.push(6);
        let r = m.manifold_report();
        assert_eq!(r.degenerate, 1, "{r}");
        assert!(r.non_manifold >= 1, "edge 0-1 now has three users: {r}");
    }
}
