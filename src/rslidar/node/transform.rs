use nalgebra::{UnitQuaternion, Vector3};

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

// 3d pose
#[derive(Clone, Copy, Debug)]
pub struct Transform {
    pub translation: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
}

impl Transform {
    pub fn identity() -> Self {
        Self { translation: Vector3::zeros(), rotation: UnitQuaternion::identity() }
    }

    pub fn apply(&self, p: Vector3<f32>) -> Vector3<f32> {
        self.rotation * p + self.translation
    }

    //self.then(inner).apply(p) == self.apply(inner.apply(p))W.
    pub fn then(&self, inner: &Transform) -> Transform {
        Transform {
            translation: self.apply(inner.translation),
            rotation: self.rotation * inner.rotation,
        }
    }
}

// stamped lidar -> world poses, ascending by stamp_ns.
pub type PoseBuffer = Arc<Mutex<VecDeque<(i64, Transform)>>>;

/// Pose at `stamp_ns`: lerp/SLERP between the bracketing samples, clamped to
/// the first/last sample outside the buffered range.
pub fn interpolate_pose(poses: &VecDeque<(i64, Transform)>, stamp_ns: i64) -> Option<Transform> {
    let pos = poses.partition_point(|(t, _)| *t <= stamp_ns);
    if pos == 0 {
        return poses.front().map(|p| p.1);
    }
    if pos == poses.len() {
        return Some(poses[pos - 1].1);
    }
    let (t0, p0) = poses[pos - 1];
    let (t1, p1) = poses[pos];
    let alpha = (stamp_ns - t0) as f32 / (t1 - t0) as f32;
    Some(Transform {
        translation: p0.translation.lerp(&p1.translation, alpha),
        rotation: p0.rotation.slerp(&p1.rotation, alpha),
    })
}