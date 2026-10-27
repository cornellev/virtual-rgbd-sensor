use std::collections::HashMap;

use nalgebra::Vector2;

use crate::segmentation::RangeGraph;

pub struct ClusterFeatures {
    pub cluster_id: i32,
    pub point_count: u32,
    pub centroid: Vector2<f32>,
    pub z_centroid: f32,
    pub mean_intensity: f32,
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
    pub z_min: f32,
    pub z_max: f32,
}

struct Accum {
    point_count: u32,
    x_sum: f32,
    y_sum: f32,
    z_sum: f32,
    intensity_sum: f32,
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
}

impl Accum {
    fn new() -> Self {
        Self {
            point_count: 0,
            x_sum: 0.0, y_sum: 0.0, z_sum: 0.0, intensity_sum: 0.0,
            x_min: f32::INFINITY, x_max: f32::NEG_INFINITY,
            y_min: f32::INFINITY, y_max: f32::NEG_INFINITY,
            z_min: f32::INFINITY, z_max: f32::NEG_INFINITY,
        }
    }
}

/// Aggregates every valid, clustered cell in `g_r` (`beta != -1`, set by
/// `second_segmentation`) into one `ClusterFeatures` per cluster id. Single
/// pass over `g_r.nodes`; clusters below `min_cluster_points` were already
/// dropped to `beta == -1` by segmentation, so they're skipped here too.
pub fn aggregate_clusters(g_r: &RangeGraph) -> Vec<ClusterFeatures> {
    let mut accs: HashMap<i32, Accum> = HashMap::new();

    for node in &g_r.nodes {
        if !node.valid || node.beta == -1 {
            continue;
        }
        let acc = accs.entry(node.beta).or_insert_with(Accum::new);
        acc.point_count += 1;
        acc.x_sum += node.x;
        acc.y_sum += node.y;
        acc.z_sum += node.z;
        acc.intensity_sum += node.intensity;
        acc.x_min = acc.x_min.min(node.x);
        acc.x_max = acc.x_max.max(node.x);
        acc.y_min = acc.y_min.min(node.y);
        acc.y_max = acc.y_max.max(node.y);
        acc.z_min = acc.z_min.min(node.z);
        acc.z_max = acc.z_max.max(node.z);
    }

    accs.into_iter()
        .map(|(cluster_id, acc)| {
            let n = acc.point_count as f32;
            ClusterFeatures {
                cluster_id,
                point_count: acc.point_count,
                centroid: Vector2::new(acc.x_sum / n, acc.y_sum / n),
                z_centroid: acc.z_sum / n,
                mean_intensity: acc.intensity_sum / n,
                x_min: acc.x_min,
                x_max: acc.x_max,
                y_min: acc.y_min,
                y_max: acc.y_max,
                z_min: acc.z_min,
                z_max: acc.z_max,
            }
        })
        .collect()
}

pub struct Tracking {
    past_clusters: Vec<ClusterFeatures>,
    curr_clusters: Vec<ClusterFeatures>
}

impl Tracking {
    // Tracking::new() is called right after segmentation outputs. `past` is
    // `None` on the first frame, before any clusters have been seen yet.
    fn new(past: Option<Vec<ClusterFeatures>>, curr: Option<Vec<ClusterFeatures>>) -> Self {
        Self {
            past_clusters: past.unwrap_or_default(),
            curr_clusters: curr.unwrap_or_default()
        }
    }

    fn distance(p1: &ClusterFeatures, p2: &ClusterFeatures) -> f32 {
        let d_xy = p1.centroid - p2.centroid;
        let d_z = p1.z_centroid - p2.z_centroid;
        (d_xy.norm_squared() + d_z * d_z).sqrt()
    }

    fn align_static(&mut self) {
        // cluster centroids
        // possible KF

        // find the cluster that is more proabable to be belonging to a given KF
        for curr in self.curr_clusters.iter() {
            let mut dist_vec: Vec<f32> = Vec::new();
            for past in self.past_clusters.iter() {
                // distance metric, for now centroid distance?
                dist_vec.push(Self::distance(curr, past))
            }
        }
    }
}