//! Components + orientation for suspect parts.
use std::collections::HashMap;
fn main() {
    for path in std::env::args().skip(1) {
        let g = solid_diff::body_graphs(std::path::Path::new(&path))
            .unwrap()
            .remove(0)
            .graph;
        let mesh = solid_diff::tess::tessellate(&g, None);
        let n = mesh.vertices.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut Vec<usize>, x: usize) -> usize {
            if p[x] != x {
                let r = find(p, p[x]);
                p[x] = r;
            }
            p[x]
        }
        for t in &mesh.triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2])] {
                let (ra, rb) = (find(&mut parent, a as usize), find(&mut parent, b as usize));
                if ra != rb {
                    parent[ra] = rb;
                }
            }
        }
        let mut sizes: HashMap<usize, usize> = HashMap::new();
        for t in &mesh.triangles {
            *sizes.entry(find(&mut parent, t[0] as usize)).or_default() += 1;
        }
        let mut v: Vec<usize> = sizes.values().copied().collect();
        v.sort_by(|a, b| b.cmp(a));
        println!(
            "{}: vol={:+.3e} components={} sizes={:?}",
            std::path::Path::new(&path)
                .file_name()
                .unwrap()
                .to_string_lossy()
                .chars()
                .take(30)
                .collect::<String>(),
            mesh.signed_volume(),
            v.len(),
            &v[..v.len().min(8)]
        );
        // which faces produced nothing?
        let meshed: std::collections::HashSet<i16> = mesh.face_ids.iter().copied().collect();
        for f in g.by_type("FACE") {
            if !meshed.contains(&f.id) {
                let sn = g
                    .deref(f, "surface")
                    .map(|s| s.name.clone())
                    .unwrap_or_default();
                println!("   face {} ({sn}) produced NO triangles", f.id);
            }
        }
    }
}
