//! Cut plans: an ordered list of planes, each optionally restricted to part of the model.

use openslicer_geom::Plane;

/// One planned cut.
#[derive(Clone, Debug)]
pub struct PlannedCut {
    pub plane: Plane,
    /// Half-space constraints: the cut applies only to pieces whose bounding-box centre has
    /// `signed_distance >= 0` for every plane listed here. Empty means the cut applies to every piece it crosses.
    /// Scoped cuts are how staggered (brick) layouts are expressed: earlier full cuts separate the
    /// rows, then each row gets its own offset cuts.
    pub scope: Vec<Plane>,
    pub label: String,
}

impl PlannedCut {
    pub fn new(plane: Plane, label: impl Into<String>) -> Self {
        Self { plane, scope: Vec::new(), label: label.into() }
    }

    pub fn scoped(mut self, scope: Vec<Plane>) -> Self {
        self.scope = scope;
        self
    }
}

#[derive(Clone, Debug, Default)]
pub struct CutPlan {
    pub cuts: Vec<PlannedCut>,
}

impl CutPlan {
    pub fn single(plane: Plane) -> Self {
        Self { cuts: vec![PlannedCut::new(plane, "cut 1")] }
    }

    pub fn from_planes(planes: impl IntoIterator<Item = Plane>) -> Self {
        Self { cuts: planes.into_iter().enumerate().map(|(i, p)| PlannedCut::new(p, format!("cut {}", i + 1))).collect() }
    }

    pub fn len(&self) -> usize {
        self.cuts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cuts.is_empty()
    }
}
