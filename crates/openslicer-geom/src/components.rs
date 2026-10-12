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

/// Whether `p` lies inside a closed mesh, by ray-crossing parity along a slightly skewed direction
/// (skewed so rays rarely graze edges or vertices of axis-aligned geometry). Odd crossings mean inside.
pub fn point_inside(mesh: &Mesh, p: Vec3) -> bool {
    let dirs = [Vec3::new(1.0, 0.0137, 0.0071), Vec3::new(0.0113, 1.0, 0.0191), Vec3::new(0.0231, 0.0089, 1.0)];
    let mut votes = 0;
    for d in dirs {
        let mut hits = 0usize;
        for t in &mesh.triangles {
            let a = mesh.vertices[t[0] as usize];
            let (e1, e2) = (mesh.vertices[t[1] as usize] - a, mesh.vertices[t[2] as usize] - a);
            let pv = d.cross(e2);
            let det = e1.dot(pv);
            if det.abs() < 1e-14 {
                continue;
            }
            let inv = 1.0 / det;
            let s = p - a;
            let u = s.dot(pv) * inv;
            if !(0.0..1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
            let v = d.dot(q) * inv;
            if v < 0.0 || u + v >= 1.0 {
                continue;
            }
            if e2.dot(q) * inv > 0.0 {
                hits += 1;
            }
        }
        votes += hits % 2;
    }
    votes >= 2
}

/// Splits a mesh into independent solids.
///
/// Connected components are separated, but a component that encloses a cavity
/// (an inward-facing component lying inside an outward-facing one, as in a hollow
/// part) stays attached to the solid that contains it. Inward-facing components that
/// lie inside no solid are returned as-is, so the caller can report them.
pub fn separate_solids(mesh: &Mesh) -> Vec<Mesh> {
    let comps = split_components(mesh);
    if comps.len() <= 1 {
        return comps;
    }
    let vols: Vec<f64> = comps.iter().map(|c| c.stats().signed_volume).collect();
    let mut solids: Vec<Option<Mesh>> = Vec::new();
    let mut solid_src: Vec<usize> = Vec::new();
    let mut cavities: Vec<usize> = Vec::new();
    for (i, c) in comps.iter().enumerate() {
        if vols[i] < 0.0 {
            cavities.push(i);
        } else {
            solids.push(Some(c.clone()));
            solid_src.push(i);
        }
    }
    let mut orphans = Vec::new();
    for ci in cavities {
        let probe = comps[ci].vertices[0];
        // Smallest-volume solid that contains the cavity.
        let mut owner: Option<usize> = None;
        for (k, &src) in solid_src.iter().enumerate() {
            let outer = &comps[src];
            let (ob, cb) = (outer.bounds().unwrap(), comps[ci].bounds().unwrap());
            let contains = ob.min.cmple(cb.min + Vec3::splat(1e-4)).all() && ob.max.cmpge(cb.max - Vec3::splat(1e-4)).all();
            if contains && point_inside(outer, probe) && owner.is_none_or(|o| vols[src] < vols[solid_src[o]]) {
                owner = Some(k);
            }
        }
        match owner {
            Some(k) => {
                let s = solids[k].as_mut().unwrap();
                let base = s.vertices.len() as u32;
                s.vertices.extend(comps[ci].vertices.iter().copied());
                s.triangles.extend(comps[ci].triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
            }
            None => orphans.push(comps[ci].clone()),
        }
    }
    let mut out: Vec<Mesh> = solids.into_iter().flatten().collect();
    out.extend(orphans);
    out
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

    #[test]
    fn point_inside_cube_and_outside() {
        let c = Mesh::cuboid(Vec3::splat(10.0));
        assert!(point_inside(&c, Vec3::ZERO));
        assert!(point_inside(&c, Vec3::new(4.9, -4.9, 4.9)));
        assert!(!point_inside(&c, Vec3::new(5.5, 0.0, 0.0)));
        assert!(!point_inside(&c, Vec3::new(0.0, 0.0, 50.0)));
    }

    #[test]
    fn hollow_shell_stays_one_solid_but_separate_solids_are_split() {
        let shell = crate::tests_support::sphere_shell(25.0, 23.0, 32, 16);
        assert_eq!(split_components(&shell).len(), 2, "raw components: outer and cavity");
        let solids = separate_solids(&shell);
        assert_eq!(solids.len(), 1, "the cavity belongs to the outer solid");
        assert!((solids[0].stats().signed_volume - shell.stats().signed_volume).abs() < 1e-3);

        // Two shells far apart remain two solids, each with its cavity.
        let mut two = shell.clone();
        let mut other = shell.clone();
        other.translate(Vec3::new(100.0, 0.0, 0.0));
        let base = two.vertices.len() as u32;
        two.vertices.extend(other.vertices.iter().copied());
        two.triangles.extend(other.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        let solids = separate_solids(&two);
        assert_eq!(solids.len(), 2);
        assert!(solids.iter().all(|s| s.stats().signed_volume > 0.0 && s.stats().is_watertight()));
    }

    #[test]
    fn nested_solid_inside_a_cavity_is_not_swallowed() {
        // A shell with a solid ball floating inside its cavity: three components, ball must be its own solid.
        let shell = crate::tests_support::sphere_shell(25.0, 23.0, 32, 16);
        let ball = crate::tests_support::sphere(5.0, 24, 12, false);
        let mut all = shell.clone();
        let base = all.vertices.len() as u32;
        all.vertices.extend(ball.vertices.iter().copied());
        all.triangles.extend(ball.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        assert_eq!(separate_solids(&all).len(), 2);
    }
}
