use std::collections::HashMap;

use nalgebra::{Vector2, Vector3};

use crate::segmentation::RangeGraph;

pub struct ClusterFeatures {
    pub cluster_id: i32,
    pub point_count: u32,
    pub centroid: Vector3<f32>,
    pub mean_intensity: f32,
    pub x_min: f32, pub x_max: f32,
    pub y_min: f32, pub y_max: f32,
    pub z_min: f32, pub z_max: f32,
}

struct Accum {
    point_count: u32,
    x_sum: f32, y_sum: f32, z_sum: f32,
    intensity_sum: f32,
    x_min: f32, x_max: f32,
    y_min: f32, y_max: f32,
    z_min: f32, z_max: f32,
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

// rangegraph from 2nd seg consists of nodes: Vec<RangeNodes>, so we want to aggregate
// to actual vector of clusterfeatures
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
                centroid: Vector3::new(acc.x_sum / n, acc.y_sum / n, acc.z_sum / n),
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

// represents a past cluster that has not been pruned cuz it has been
// recently matched to some curr clsuter
#[derive(Clone, Copy)]
pub struct Track {
    pub id: u32, // persistent track id was last matched with centroid
    pub centroid: Vector3<f32>,
    pub misses: u32,
}

// persists throughout entire zenoh_wrapper call, is basically the
// struct / class containing all logic for obst tracking
pub struct Tracker {
    tracks: Vec<Track>,
    next_id: u32,
    max_match_dist: f32,
    max_misses: u32,
}

impl Tracker {
    // so this is basically accounting for occlusions, if a id is not
    // matched for less than max_match_dist, keep it
    pub fn new(max_match_dist: f32, max_misses: u32) -> Self {
        Self {
            tracks: Vec::new(),
            next_id: 0,
            max_match_dist: max_match_dist,
            max_misses: max_misses
        }
    }

    // given new clusters, do greedy assignment of cluster id with prev track ids
    // also prune track ids if stale, also spawn new cluster id if no match
    pub fn update(&mut self, clusters: &[ClusterFeatures]) -> HashMap<i32, u32> {
        // this is waht is called per new revolution lidar frame
        let mut beta_to_track_id = HashMap::new();
        let cost_matrix = self.cost_matrix(&clusters); // get cost matrix
        let matches = Tracker::greedy_assign(&cost_matrix, self.max_match_dist);
        let mut tracks = vec![false; self.tracks.len()]; // keeps track of which track is matched
        // greedy assign beta -> track over cost matrix
        for c in 0..clusters.len() {
            if let Some(&t) = matches.get(&c) {
                beta_to_track_id.insert(clusters[c].cluster_id, self.tracks[t].id);
                self.tracks[t].centroid = clusters[c].centroid;
                tracks[t] = true;
            } else {
                // spawn
                let id = self.spawn(&clusters[c]);
                beta_to_track_id.insert(clusters[c].cluster_id, id);
            }
        }
        // for unmatched prev clusters in tracks, maybe prune
        self.prune(tracks);
        beta_to_track_id
    }

    fn distance(track: &Track, cluster: &ClusterFeatures) -> f32 {
        let dx = track.centroid.x - cluster.centroid.x;
        let dy = track.centroid.y - cluster.centroid.y;
        let dz = track.centroid.z - cluster.centroid.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    // cost[t][c] = distance(tracks[t], clusters[c]).
    fn cost_matrix(&self, clusters: &[ClusterFeatures]) -> Vec<Vec<f32>> {
        let n = self.tracks.len();
        let m = clusters.len();
        let mut cost: Vec<Vec<f32>> = vec![vec![-1.0; m]; n];
        for t in 0..n {
            for c in 0..m {
                cost[t][c] = Self::distance(&self.tracks[t], &clusters[c]);
            }
        }
        cost
    }

    fn greedy_assign(cost: &[Vec<f32>], max_match_dist: f32) -> HashMap<usize, usize> {
        // given cost matrix, we will choose the global minimum cost (of whatever is remaining)
        // retire that row (t) and column (c): so can't use the corresponding track id and cluster
        // so guarantee injectivity
        // stop when either no more row cols, or min cost > max_match_dist
        // ig we can make cost a minheap, keep popping from it until max_match_dist
        let n_clusters = cost.first().map_or(0, |r| r.len());
        let mut matches = HashMap::new();
        let mut min_heap: Vec<(f32, usize, usize)> = cost.iter().enumerate()
            .flat_map(|(t, costs)| costs.iter().enumerate().map(move |(c, &cost)| (cost, t, c)))
            .filter(|&(cost, _, _)| cost <= max_match_dist) // like don't even think about costs > max
            .collect();
        min_heap.sort_unstable_by(|a, b| a.0.total_cmp(&b.0)); // sort by cost

        let (mut matched_t, mut matched_c) = (vec![false; cost.len()], vec![false; n_clusters]);

        for (cost, t, c) in min_heap.into_iter() {
            if !matched_t[t] && !matched_c[c] { // track and cluster both free
                matches.insert(c, t); // map cluster to track
                matched_t[t] = true;
                matched_c[c] = true;
            }
        }
        matches
    }

    fn spawn(&mut self, cluster: &ClusterFeatures) -> u32 {
        let id = self.next_id;
        self.tracks.push(Track {
            id,
            centroid: cluster.centroid,
            misses: 0
        });
        self.next_id += 1;
        id
    }

    // prune Track if unmatched for more than max_misses
    fn prune(&mut self, tracks: Vec<bool>) {
        let n_old = tracks.len();
        let mut new_tracks = Vec::new();
        for (t, valid) in tracks.into_iter().enumerate() {
            if valid { // matched, reset miss to 0
                self.tracks[t].misses = 0;
                new_tracks.push(self.tracks[t]);
            }
            else if self.tracks[t].misses < self.max_misses { // not matched
                self.tracks[t].misses += 1;
                new_tracks.push(self.tracks[t]);
            }
        }
        new_tracks.extend_from_slice(&self.tracks[n_old..]);
        self.tracks = new_tracks;
    }
}

// rewrite nodes' beta to newly assigned id
pub fn relabel(g_r: &mut RangeGraph, ids: &HashMap<i32, u32>) {
    for n in g_r.nodes.iter_mut().filter(|n| n.valid && n.beta != -1) {
        n.beta = ids.get(&n.beta).map_or(-1, |&id| id as i32);
    }
}
