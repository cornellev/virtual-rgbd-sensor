use nalgebra::Vector2;
use std::collections::HashMap;
use std::collections::VecDeque;

// should i eventually also return like a connected components union findish structure for 
// clusters?
// or hashmap

#[derive(Clone, Copy)]
#[allow(unused_variables)]
pub struct SegParams {
    pub seg_enabled: bool,
    pub height_filter_enabled: bool,
    pub min_height: f32,
    pub th_d: f32,
    pub th_z_deg: f32,
    pub th_v_base: f32,
    pub th_v_slope: f32,
    pub z_weight: f32,
    pub min_cluster_points: u32,
    pub min_gap_m: f32,
}

impl Default for SegParams {
    fn default() -> Self {
        Self {
            seg_enabled: true,
            height_filter_enabled: false,
            min_height: -0.6, // ceiling is a problem
            th_d: 0.01, th_z_deg: 15.0, th_v_base: 0.25, th_v_slope: 0.01, z_weight: 1.0,
            min_cluster_points: 3, min_gap_m: 1.0,
        }
    }
}

/// Zenoh key the visualizer publishes tuned `SegParams` on.
pub const SEG_PARAMS_KEY: &str = "rslidar/seg_params";

/// Wire format on `SEG_PARAMS_KEY`: seg_enabled and height_filter_enabled as
/// one byte each, then min_height, th_d, th_z_deg, th_v_base (m), th_v_slope (m/m),
/// z_weight (f32), min_cluster_points (u32), min_gap_m (f32), little-endian.
impl SegParams {
    pub const WIRE_LEN: usize = 34;

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(Self::WIRE_LEN);
        buf.push(self.seg_enabled as u8);
        buf.push(self.height_filter_enabled as u8);
        for v in [self.min_height, self.th_d, self.th_z_deg, self.th_v_base, self.th_v_slope, self.z_weight] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        buf.extend_from_slice(&self.min_cluster_points.to_le_bytes());
        buf.extend_from_slice(&self.min_gap_m.to_le_bytes());
        buf
    }

    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() != Self::WIRE_LEN {
            return None;
        }
        let f = |i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        Some(Self {
            seg_enabled: b[0] != 0,
            height_filter_enabled: b[1] != 0,
            min_height: f(2),
            th_d: f(6),
            th_z_deg: f(10),
            th_v_base: f(14),
            th_v_slope: f(18),
            z_weight: f(22),
            min_cluster_points: u32::from_le_bytes(b[26..30].try_into().unwrap()),
            min_gap_m: f(30),
        })
    }
}

#[derive(Clone, Copy)]
pub struct RangeNode {
    pub x : f32,
    pub y : f32,
    pub z : f32,
    pub intensity : f32,
    pub range : f32,
    pub azimuth_centideg : i32,
    pub point_idx : i32,
    pub valid : bool,
    pub alpha : i32,
    pub beta : i32
}

impl RangeNode {
    pub fn new() -> Self {
        Self {
            x : 0.0,
            y : 0.0,
            z : 0.0,
            intensity : 0.0,
            range : 0.0,
            azimuth_centideg : 0,
            point_idx : -1,
            valid : false,
            alpha : -1,
            beta : -1
        }
    }
}

// pub because it appears inside SetGraph's public `nodes` field.
pub struct GraphNode {
    pub range : f32,
    pub x_mean : f32,
    pub y_mean : f32,
    pub z_mean : f32,
    pub alpha : i32,
    pub cluster : i32,
    pub start_pos : i32,
    pub end_pos : i32,
    // indices into G_r that belong to this node
    pub members : Vec<i32>,
    // (bin = col / BIN_COLS, x_sum, y_sum, count), ascending by bin
    // binning horizontal segments cuz need better resolution for
    // vertical adjacency cmps >:(, walls are annoying
    pub bins : Vec<(u16, f32, f32, u32)>
}

impl GraphNode {
    pub fn new() -> Self {
        Self {
            range : 0.0,
            x_mean : 0.0,
            y_mean : 0.0,
            z_mean : 0.0,
            alpha : -1,
            cluster : -1,
            start_pos : -1,
            end_pos : -1,
            members : Vec::new(),
            bins : Vec::new()
        }
    }
}

pub const NUM_RINGS: usize = 32;
pub const AZ_RES_CENTIDEG: i32 = 10;
pub const NUM_COLS: usize = (36000 / AZ_RES_CENTIDEG) as usize;
/// How far `get_vecs` looks past empty columns for a surface neighbor; 4
/// covers the 0.4 deg firing step at 1200 rpm.
const MAX_NEIGHBOR_COLS: usize = 4;
// like around 20 points per bin, 4 deg
const BIN_COLS: usize = 40;

/// 2d array representation of LiDAR divided by the vertical angles fired at each time
/// frame and also the horizontal angles fired consecutively at different timeframes
/// until we get full 0 -> 359 rotation
pub struct RangeGraph {
    pub nodes: Vec<RangeNode>,
}

impl RangeGraph {
    // this is vector<RangeNode> G_r in two_layer_seg.cpp, 
    pub fn new() -> Self {
        Self { nodes: vec![RangeNode::new(); NUM_RINGS * NUM_COLS] }
    }

    /// Resets every cell to empty/invalid, ready for the next revolution.
    pub fn clear(&mut self) {
        for node in self.nodes.iter_mut() {
            *node = RangeNode::new();
        }
    }

    // inserts one decoded point into its `(ring, column)` cell
    // if another point already landed in this cell
    // this revolution, keeps whichever is closer to the sensor
    pub fn insert(
        &mut self,
        ring: usize,
        azimuth_centideg: i32,
        x: f32, y: f32, z: f32,
        intensity: f32,
        range: f32,
        point_idx: i32,
    ) {
        let col = (azimuth_centideg / AZ_RES_CENTIDEG) as usize % NUM_COLS;
        let idx = ring * NUM_COLS + col;
        let cell = &mut self.nodes[idx];
        if !cell.valid || range < cell.range {
            *cell = RangeNode { x, y, z, intensity, range, azimuth_centideg, point_idx, valid: true, alpha: -1, beta: -1 };
        }
    }

    fn azimuth_gap_centideg(a: i32, b: i32) -> i32 {
        let raw = (a - b).rem_euclid(36000);
        raw.min(36000 - raw)
    }

    fn point_dist(&mut self, idx_a : usize, idx_b : usize) -> f32 {
        let dx = self.nodes[idx_a].x - self.nodes[idx_b].x;
        let dy = self.nodes[idx_a].y - self.nodes[idx_b].y;
        (dx * dx + dy * dy).sqrt()
    }

    // Index of the first valid cell in `ring` stepping away from `col` by
    // `step` (1 forward, NUM_COLS - 1 backward), within MAX_NEIGHBOR_COLS
    fn nearest_valid(&self, ring: usize, col: usize, step: usize) -> Option<usize> {
        (1..=MAX_NEIGHBOR_COLS)
            .map(|k| ring * NUM_COLS + (col + k * step) % NUM_COLS)
            .find(|&idx| self.nodes[idx].valid)
    }

    fn get_vecs(&mut self, idx_cur : usize, idx_pre : usize, ring : usize) -> Option<[Vector2<f32>; 2]> {
        // in one horizontal ring, this is represented by one row in G_r: after seg, we get
        // |** * ***|        |*** * ****|, 2 different pcs
        //       idx_pre  idx_cur
        // idx_pre is the last cell of the previous segment (Pre Pos in Alg 1)
        // idx_cur is the first cell of the current segment

        let idx_pre_inner = self.nearest_valid(ring, idx_pre % NUM_COLS, NUM_COLS - 1)?;
        let idx_cur_inner = self.nearest_valid(ring, idx_cur % NUM_COLS, 1)?;

        let p_pre = Vector2::<f32>::new(self.nodes[idx_pre].x, self.nodes[idx_pre].y);
        let p_pre_in = Vector2::<f32>::new(self.nodes[idx_pre_inner].x, self.nodes[idx_pre_inner].y);
        let v2 = p_pre_in - p_pre;

        let p_cur = Vector2::<f32>::new(self.nodes[idx_cur].x, self.nodes[idx_cur].y);
        let p_cur_in = Vector2::<f32>::new(self.nodes[idx_cur_inner].x, self.nodes[idx_cur_inner].y);
        let v1 = p_cur_in - p_cur;

        Some([v1, v2])
    }

    /// this is basically whether or not two points are horizontally adjacent:
    /// like checks distance in terms of x and y,
    /// and also the angle formed by the vectors formed by prev -> curr and curr -> next points
    /// in the channel: like to figure out whether or not two points represent corners of seperate surfaces that are
    /// close together or actually part of the same surface
    fn should_split(&mut self, idx_cur: usize, idx_pre: usize, ring: usize, th_d: f32, th_z: f32, min_gap_m: f32) -> bool {
        let dist = self.point_dist(idx_cur, idx_pre);
        let avg_range = (self.nodes[idx_cur].range + self.nodes[idx_pre].range) / 2.0;
        let far_apart = dist > th_d * avg_range / 0.3;
        // these are vectors depicting orientation of surface between two suspected objects;
        // without them only the distance test applies
        let (v1, v2) = match self.get_vecs(idx_cur, idx_pre, ring) {
            Some(vecs) => (vecs[0], vecs[1]),
            None => return far_apart,
        };

        let angle = (v1.dot(&v2) / (v1.norm() * v2.norm())).acos();
        let v_anglebisector = (v1 + v2) / (v1 + v2).norm();
        let v_mo = Vector2::new(
            -(self.nodes[idx_cur].x + self.nodes[idx_pre].x) / 2.0,
            -(self.nodes[idx_cur].y + self.nodes[idx_pre].y) / 2.0,
        );

        let az_gap_rad = (Self::azimuth_gap_centideg(
            self.nodes[idx_cur].azimuth_centideg,
            self.nodes[idx_pre].azimuth_centideg,
        ) as f32 / 100.0).to_radians();
        let threshold = th_d * avg_range * az_gap_rad;

        // angle is to check whether two surfaces are facing away from each other, if yes, probably
        // found 2 separate corners (not a flat surface), split into different categories
        far_apart || (angle < th_z && v_mo.dot(&v_anglebisector) > 0.0)
    }

    // given each vertical channel: which are the horizontal rows in RangeGraph split defined by each vertical angle
    // of the LiDAR, segment horizontally to form SetGraph
    pub fn first_segmentation(&mut self, set_graph: &mut SetGraph, th_d : f32, th_z : f32, min_gap_m : f32) {
        set_graph.clear();

        for i in 0..NUM_RINGS {
            let mut l_cnt = 0;
            let mut pre_pos = usize::MAX;
            // alpha label -> the GraphNode accumulating that segment's cells,
            // built up live as cells get labeled below instead of by a
            // second scan over the finished labels afterward.
            let mut seg_map: HashMap<i32, GraphNode> = HashMap::new();

            for j in 0..NUM_COLS {
                let idx = i * NUM_COLS + j;
                if !self.nodes[idx].valid {
                    continue;
                }

                if l_cnt == 0 {
                    l_cnt += 1;
                    pre_pos = j;
                    self.nodes[idx].alpha = l_cnt;
                } else {
                    let pre_idx = i * NUM_COLS + pre_pos;
                    if self.should_split(idx, pre_idx, i, th_d, th_z, 0.5) {
                        l_cnt += 1;
                    }
                    pre_pos = j;
                    self.nodes[idx].alpha = l_cnt;
                }

                let cell = &self.nodes[idx];
                let node = seg_map.entry(l_cnt).or_insert_with(GraphNode::new);
                node.alpha = l_cnt;
                node.members.push(idx as i32);
                node.range += cell.range;
                node.x_mean += cell.x;
                node.y_mean += cell.y;
                node.z_mean += cell.z;
                let bin = (j / BIN_COLS) as u16;
                match node.bins.last_mut() {
                    Some(last) if last.0 == bin => {
                        last.1 += cell.x;
                        last.2 += cell.y;
                        last.3 += 1;
                    }
                    _ => node.bins.push((bin, cell.x, cell.y, 1)),
                }
                let j_i32 = j as i32;
                if node.start_pos == -1 || j_i32 < node.start_pos {
                    node.start_pos = j_i32;
                }
                if node.end_pos == -1 || j_i32 > node.end_pos {
                    node.end_pos = j_i32;
                }
            }

            let first_j = (0..NUM_COLS).find(|&j| self.nodes[i * NUM_COLS + j].valid);
            let last_j = (0..NUM_COLS).rev().find(|&j| self.nodes[i * NUM_COLS + j].valid);
            if let (Some(first_j), Some(last_j)) = (first_j, last_j) {
                let first_idx = i * NUM_COLS + first_j;
                let last_idx = i * NUM_COLS + last_j;
                let first_alpha = self.nodes[first_idx].alpha;
                let last_alpha = self.nodes[last_idx].alpha;

                if first_alpha != last_alpha && !self.should_split(first_idx, last_idx, i, th_d, th_z, min_gap_m) {
                    if let Some(last_node) = seg_map.remove(&last_alpha) {
                        for &idx in &last_node.members {
                            self.nodes[idx as usize].alpha = first_alpha;
                        }
                        let first_node = seg_map
                            .get_mut(&first_alpha)
                            .expect("first_alpha's cell was labeled during the scan above, so its GraphNode must already exist");
                        first_node.range += last_node.range;
                        first_node.x_mean += last_node.x_mean;
                        first_node.y_mean += last_node.y_mean;
                        first_node.z_mean += last_node.z_mean;
                        first_node.start_pos = first_node.start_pos.min(last_node.start_pos);
                        first_node.end_pos = first_node.end_pos.max(last_node.end_pos);
                        first_node.members.extend(last_node.members);
                        first_node.bins.extend(last_node.bins);
                    }
                }
            }

            for (_, mut node) in seg_map {
                let n = node.members.len() as f32;
                node.x_mean /= n;
                node.y_mean /= n;
                node.z_mean /= n;
                node.range /= n;
                set_graph.nodes[i].push(node);
            }
            set_graph.nodes[i].sort_by(|a, b| a.start_pos.cmp(&b.start_pos));
        }
    }
}

/// setgraph is the result of horizontal 1st segmentation
pub struct SetGraph {
    pub nodes: Vec<Vec<GraphNode>>
}

impl SetGraph {
    pub fn new() -> Self {
        Self { nodes: (0..NUM_RINGS).map(|_| Vec::new()).collect() }
    }

    pub fn clear(&mut self) {
        for ring in self.nodes.iter_mut() {
            ring.clear();
        }
    }

    // xy distance between two ring segments using their azimuth bins: the
    // closest pair of bin means in shared bins
    fn bin_dist(a: &GraphNode, b: &GraphNode) -> f32 {
        let (mut i, mut j) = (0, 0);
        let mut shared = f32::INFINITY;
        let (mut nearest_gap, mut nearest) = (u16::MAX, f32::INFINITY);
        while i < a.bins.len() && j < b.bins.len() {
            let (ba, xa, ya, na) = a.bins[i];
            let (bb, xb, yb, nb) = b.bins[j];
            let dx = xa / na as f32 - xb / nb as f32;
            let dy = ya / na as f32 - yb / nb as f32;
            let d = (dx * dx + dy * dy).sqrt();
            if ba == bb {
                shared = shared.min(d);
                i += 1;
                j += 1;
            } else {
                let gap = ba.abs_diff(bb);
                if gap < nearest_gap || (gap == nearest_gap && d < nearest) {
                    (nearest_gap, nearest) = (gap, d);
                }
                if ba < bb { i += 1 } else { j += 1 }
            }
        }
        if shared.is_finite() { shared } else { nearest }
    }

    // th_v_base + th_v_slope * range
    fn cross_ring_threshold(th_v_base: f32, th_v_slope: f32, min_range: f32) -> f32 {
        th_v_base + th_v_slope * min_range
    }

    fn get_neighbors(&self, ri: usize, ci: usize, th_v_base: f32, th_v_slope: f32) -> Vec<(usize, usize)> {
        let mut nbrs = Vec::new();
        let cur = &self.nodes[ri][ci];

        for dr in [-1i32, 1i32] {
            // vertical neighbors, above or below adjacent
            let rn = ri as i32 + dr;
            if rn < 0 || rn as usize >= NUM_RINGS {
                continue;
            }
            let rn = rn as usize;

            let try_push = |cn: usize, nbrs: &mut Vec<(usize, usize)>| {
                let cand = &self.nodes[rn][cn];
                let threshold = Self::cross_ring_threshold(th_v_base, th_v_slope, cur.range.min(cand.range));
                if Self::bin_dist(cur, cand) < threshold {
                    nbrs.push((rn, cn));
                }
            };

            let mut last_before: Option<usize> = None;
            let mut first_after: Option<usize> = None;

            for cn in 0..self.nodes[rn].len() {
                // iterating by column, so horizontally adjacent nodes
                let cand = &self.nodes[rn][cn];
                if cand.end_pos < cur.start_pos {
                    last_before = Some(cn);
                    continue;
                }
                if cand.start_pos > cur.end_pos {
                    first_after = Some(cn);
                    break;
                }
                try_push(cn, &mut nbrs);
            }
            if let Some(cn) = last_before {
                try_push(cn, &mut nbrs);
            }
            if let Some(cn) = first_after {
                try_push(cn, &mut nbrs);
            }
        }
        nbrs
    }

    // this takes the horizontally segmented range graph -> set graph and 
    // then runs BFS given vertically adjacent neighbors to merge vertically
    // and rings
    pub fn second_segmentation(
        &mut self,
        g_r: &mut RangeGraph,
        th_v_base: f32,
        th_v_slope: f32,
        min_cluster_points: u32,
    ) -> i32 {
        for i in 0..NUM_RINGS {
            for n in self.nodes[i].iter_mut() {
                n.cluster = -1;
            }
        }

        let mut l_cnt = 0;
        let mut q: VecDeque<(usize, usize)> = VecDeque::new();

        for i in 0..NUM_RINGS {
            for j in 0..self.nodes[i].len() {
                if self.nodes[i][j].cluster != -1 {
                    continue;
                }
                l_cnt += 1;
                q.push_back((i, j));
                self.nodes[i][j].cluster = l_cnt;

                while let Some((ri, ci)) = q.pop_front() {
                    for (rn, cn) in self.get_neighbors(ri, ci, th_v_base, th_v_slope) {
                        if self.nodes[rn][cn].cluster != -1 {
                            continue;
                        }
                        self.nodes[rn][cn].cluster = l_cnt;
                        q.push_back((rn, cn));
                    }
                }
            }
        }

        let mut cluster_sizes: HashMap<i32, usize> = HashMap::new();
        for i in 0..NUM_RINGS {
            for node in &self.nodes[i] {
                *cluster_sizes.entry(node.cluster).or_insert(0) += node.members.len();
            }
        }

        // propagate final cluster ids back to RangeGraph cells that each GraphNode was built from
        for i in 0..NUM_RINGS {
            for node in self.nodes[i].iter() {
                let size = cluster_sizes.get(&node.cluster).copied().unwrap_or(0);
                let beta = if size >= min_cluster_points as usize { node.cluster } else { -1 };
                for &idx in node.members.iter() {
                    g_r.nodes[idx as usize].beta = beta;
                }
            }
        }

        l_cnt
    }
}