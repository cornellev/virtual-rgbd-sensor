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

    /// Composes `self` (outer) after `inner`, so that
    /// `self.then(inner).apply(p) == self.apply(inner.apply(p))`.
    pub fn then(&self, inner: &Transform) -> Transform {
        Transform {
            translation: self.apply(inner.translation),
            rotation: self.rotation * inner.rotation,
        }
    }
}

/// Time-ordered samples of `base_link`'s pose in the world frame, as fed by
/// an `orch/location`-style subscriber. Bounded so a long-running process
/// doesn't grow this forever; only the two samples bracketing the current
/// frame's timestamp are ever needed.
pub type PoseBuffer = Arc<Mutex<VecDeque<(i64, Transform)>>>;

/// Interpolates `poses` (sorted ascending by timestamp) at `stamp_ns`:
/// linear for translation, SLERP for rotation, between the two buffered
/// samples bracketing `stamp_ns`. In a live pipeline there's usually no
/// sample newer than the current frame yet (poses and frames both arrive in
/// real time), so this holds the latest available sample instead of
/// refusing to compensate at all -- only returns `None` if the buffer is
/// empty or every buffered sample is still newer than `stamp_ns` (pose
/// stream just started, nothing old enough to use yet).
pub fn interpolate_pose(poses: &VecDeque<(i64, Transform)>, stamp_ns: i64) -> Option<Transform> {
    if poses.is_empty() {
        return None;
    }
    let pos = poses.partition_point(|(t, _)| *t <= stamp_ns);
    if pos == 0 {
        return None;
    }
    if pos == poses.len() {
        // No newer sample yet -- hold the latest one rather than falling
        // back to identity (which would silently stop compensating at all).
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