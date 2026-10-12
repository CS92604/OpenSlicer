//! Oriented planes and in-plane 2D frames.

use glam::{DVec2, DVec3};

/// The plane `n · x = d` with unit normal `n`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub n: DVec3,
    pub d: f64,
}

/// An orthonormal frame in a plane: `u × v = n`, with an origin on the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: DVec3,
    pub u: DVec3,
    pub v: DVec3,
    pub n: DVec3,
}

impl Plane {
    /// Plane through `point` with the given normal. Returns `None` for a zero or non-finite normal.
    pub fn from_point_normal(point: DVec3, normal: DVec3) -> Option<Self> {
        let len = normal.length();
        if !len.is_finite() || len < 1e-12 || !point.is_finite() {
            return None;
        }
        let n = normal / len;
        Some(Self { n, d: n.dot(point) })
    }

    /// Axis-aligned plane `x[axis] = offset`, normal along +axis.
    pub fn axis(axis: usize, offset: f64) -> Self {
        let mut n = DVec3::ZERO;
        n[axis] = 1.0;
        Self { n, d: offset }
    }

    pub fn signed_distance(&self, p: DVec3) -> f64 {
        self.n.dot(p) - self.d
    }

    pub fn flipped(&self) -> Self {
        Self { n: -self.n, d: -self.d }
    }

    pub fn project(&self, p: DVec3) -> DVec3 {
        p - self.n * self.signed_distance(p)
    }

    /// Deterministic frame on this plane. The origin is the point of the plane closest to the world origin.
    pub fn frame(&self) -> Frame {
        let helper = if self.n.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
        let u = helper.cross(self.n).normalize();
        let v = self.n.cross(u);
        Frame { origin: self.n * self.d, u, v, n: self.n }
    }
}

impl Frame {
    pub fn to_2d(&self, p: DVec3) -> DVec2 {
        let r = p - self.origin;
        DVec2::new(r.dot(self.u), r.dot(self.v))
    }

    pub fn to_3d(&self, p: DVec2) -> DVec3 {
        self.origin + self.u * p.x + self.v * p.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_is_orthonormal_and_right_handed() {
        for n in [DVec3::Z, DVec3::X, DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.0, 0.0, -1.0)] {
            let p = Plane::from_point_normal(DVec3::new(5.0, -3.0, 2.0), n).unwrap();
            let f = p.frame();
            assert!((f.u.length() - 1.0).abs() < 1e-12 && (f.v.length() - 1.0).abs() < 1e-12);
            assert!(f.u.dot(f.v).abs() < 1e-12 && f.u.dot(f.n).abs() < 1e-12);
            assert!((f.u.cross(f.v) - f.n).length() < 1e-12);
            let q = DVec3::new(1.0, 2.0, 3.0);
            let on_plane = p.project(q);
            assert!((f.to_3d(f.to_2d(on_plane)) - on_plane).length() < 1e-9);
        }
    }

    #[test]
    fn degenerate_normals_are_rejected() {
        assert!(Plane::from_point_normal(DVec3::ZERO, DVec3::ZERO).is_none());
        assert!(Plane::from_point_normal(DVec3::ZERO, DVec3::new(f64::NAN, 0.0, 1.0)).is_none());
    }
}
