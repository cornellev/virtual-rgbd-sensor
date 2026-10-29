//! One revolution in, perception out: decode -> segment -> local costmap
//! (sensor frame) -> ego-motion compensation -> tracking. No I/O or
//! threading; callers feed packets and poses from whatever transport.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use nalgebra::Vector3;

use crate::costmap_2d::{Costmap, Costmap2D, CostmapParams};
use crate::rslidar_math::{PointCloud2, POINT_STEP};
use crate::rslidar_sdk_node::{DriverConfig, RawPacket, RsHeliosDecoder};
use crate::segmentation::{RangeGraph, SegParams};
use crate::tracking::{aggregate_clusters, relabel, Tracker};
use crate::transform::{interpolate_pose, Transform};

/// ~17 s of 60 Hz poses.
const POSE_BUFFER_LEN: usize = 1024;

pub struct Frame<'a> {
    /// World frame.
    pub cloud: &'a PointCloud2,
    /// World frame; `beta` holds persistent track ids.
    pub range_graph: &'a RangeGraph,
    /// Track id per point in `cloud` (-1 = unclustered).
    pub point_cluster: &'a [i32],
    /// lidar -> world at the frame's stamp.
    pub tf: Transform,
    /// False if `tf` was held/clamped rather than interpolated between two
    /// poses (pose stream lagging, ahead, or absent).
    pub pose_bracketed: bool,
    /// Per-revolution local occupancy grid in the sensor frame; place it in
    /// the world with `tf`.
    pub costmap: &'a Costmap2D,
}

pub struct Pipeline {
    decoder: RsHeliosDecoder,
    costmap: Costmap,
    tracker: Tracker,
    poses: VecDeque<(i64, Transform)>,
}

impl Pipeline {
    pub fn new(
        cfg: &DriverConfig,
        costmap_params: &CostmapParams,
        seg_params: Arc<Mutex<SegParams>>,
        tracker: Tracker,
    ) -> Self {
        Self {
            decoder: RsHeliosDecoder::new(cfg, seg_params),
            costmap: Costmap::new(costmap_params),
            tracker,
            poses: VecDeque::new(),
        }
    }

    /// Poses must share the lidar clock. A stamp older than the newest one
    /// means the source restarted (replay loop, localization reset), so the
    /// history is dropped rather than interpolated across.
    pub fn push_pose(&mut self, stamp_ns: i64, pose: Transform) {
        if self.poses.back().is_some_and(|&(t, _)| stamp_ns < t) {
            self.poses.clear();
        }
        if self.poses.len() == POSE_BUFFER_LEN {
            self.poses.pop_front();
        }
        self.poses.push_back((stamp_ns, pose));
    }

    /// Calls `on_frame` once per completed revolution (usually 0 or 1 per packet).
    pub fn push_packet(&mut self, pkt: &RawPacket, mut on_frame: impl FnMut(Frame<'_>)) {
        let buf = match pkt {
            RawPacket::Difop(buf) => return self.decoder.decode_difop(buf),
            RawPacket::Msop(buf) => buf,
        };
        for (mut cloud, mut graph) in self.decoder.decode_msop(buf) {
            // The costmap raytraces from the sensor origin, so it needs the
            // sensor-frame graph.
            self.costmap.update(&graph);

            let stamp_ns = cloud.header.stamp_sec as i64 * 1_000_000_000 + cloud.header.stamp_nanosec as i64;
            let tf = interpolate_pose(&self.poses, stamp_ns).unwrap_or_else(Transform::identity);
            let pose_bracketed = matches!(
                (self.poses.front(), self.poses.back()),
                (Some(&(lo, _)), Some(&(hi, _))) if lo <= stamp_ns && stamp_ns <= hi
            );
            to_world(&tf, &mut cloud.data, &mut graph);

            let ids = self.tracker.update(&aggregate_clusters(&graph));
            relabel(&mut graph, &ids);

            let mut point_cluster = vec![-1i32; cloud.width as usize];
            for n in graph.nodes.iter().filter(|n| n.valid && n.point_idx >= 0) {
                point_cluster[n.point_idx as usize] = n.beta;
            }

            on_frame(Frame {
                cloud: &cloud,
                range_graph: &graph,
                point_cluster: &point_cluster,
                tf,
                pose_bracketed,
                costmap: self.costmap.master(),
            });
        }
    }
}

fn to_world(tf: &Transform, cloud_data: &mut [u8], graph: &mut RangeGraph) {
    for p in cloud_data.chunks_exact_mut(POINT_STEP as usize) {
        let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
        let (x, y, z) = (f(0), f(4), f(8));
        if x.is_nan() {
            continue;
        }
        let w = tf.apply(Vector3::new(x, y, z));
        p[0..4].copy_from_slice(&w.x.to_le_bytes());
        p[4..8].copy_from_slice(&w.y.to_le_bytes());
        p[8..12].copy_from_slice(&w.z.to_le_bytes());
    }
    for n in graph.nodes.iter_mut().filter(|n| n.valid) {
        let w = tf.apply(Vector3::new(n.x, n.y, n.z));
        (n.x, n.y, n.z) = (w.x, w.y, w.z);
    }
}
