//! Connected-component separation.

use glam::Vec3;
use openslicer_core::Mesh;

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// Splits a mesh into its connected components (triangles sharing a vertex are connected).
/// Each component is returned as an independent mesh with compacted vertices.
/// Components are ordered by their first triangle, which keeps results deterministic.
pub fn split_components(mesh: &Mesh) -> Vec<Mesh> {
    if mesh.triangles.is_empty() {
        return Vec::new();
    }
    let n = mesh.vertices.len();
    let mut parent: Vec<u32> = (0..n as u32).collect();
    for t in &mesh.triangles {
        let (a, b, c) = (find(&mut parent, t[0]), find(&mut parent, t[1]), find(&mut parent, t[2]));
        parent[b as usize] = a;
        let c = find(&mut parent, c);
        let a = find(&mut parent, a);
        parent[c as usize] = a;
    }
    let mut comp_of_root = vec![u32::MAX; n];
    let mut comps: Vec<Vec<u32>> = Vec::new(); // triangle indices per component
    for (i, t) in mesh.triangles.iter().enumerate() {
        let root = find(&mut parent, t[0]) as usize;
        if comp_of_root[root] == u32::MAX {
            comp_of_root[root] = comps.len() as u32;
            comps.push(Vec::new());
        }
        comps[comp_of_root[root] as usize].push(i as u32);
    }
    comps.iter().map(|tris| extract(mesh, tris)).collect()
}

/// Builds a mesh from a subset of triangles, dropping unused vertices.
pub fn extract(mesh: &Mesh, tri_ids: &[u32]) -> Mesh {
    let mut map = vec![u32::MAX; mesh.vertices.len()];
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris = Vec::with_capacity(tri_ids.len());
    for &ti in tri_ids {
        let t = mesh.triangles[ti as usize];
        let mut out = [0u32; 3];
        for k in 0..3 {
            let v = t[k] as usize;
            if map[v] == u32::MAX {
                map[v] = verts.len() as u32;
                verts.push(mesh.vertices[v]);
            }
            out[k] = map[v];
        }
        tris.push(out);
    }
    Mesh::new(verts, tris)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_separate_cubes_become_two_components() {
        let a = Mesh::cuboid(Vec3::splat(10.0));
        let mut b = Mesh::cuboid(Vec3::splat(4.0));
        b.translate(Vec3::new(50.0, 0.0, 0.0));
        let mut all = a.clone();
        let base = all.vertices.len() as u32;
        all.vertices.extend(b.vertices.iter().copied());
        all.triangles.extend(b.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        let parts = split_components(&all);
        assert_eq!(parts.len(), 2);
        let mut vols: Vec<f64> = parts.iter().map(|p| p.stats().signed_volume).collect();
        vols.sort_by(|x, y| x.total_cmp(y));
        assert!((vols[0] - 64.0).abs() < 1e-4 && (vols[1] - 1000.0).abs() < 1e-3);
        assert!(parts.iter().all(|p| p.stats().is_watertight()));
    }

    #[test]
    fn single_connected_mesh_stays_whole() {
        let parts = split_components(&Mesh::cuboid(Vec3::ONE));
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].triangles.len(), 12);
    }
}
