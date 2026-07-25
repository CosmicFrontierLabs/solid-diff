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
