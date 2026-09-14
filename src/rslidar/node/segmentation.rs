use nalgebra::Vector2;
use std::collections::HashMap;
use std::collections::VecDeque;

/// The tunable thresholds `first_segmentation`/`second_segmentation` read
/// each revolution. Plain data with no Bevy dependency -- wrapped in
/// `Arc<Mutex<..>>` at the call site so it can be shared between the decoder
/// thread and a live keyboard-driven tuner in the visualizer (see
/// `visualization::tune_seg_params`).
#[derive(Clone, Copy)]
pub struct SegParams {
    /// Dimensionless scale on the *expected* same-ring point spacing
    /// (average range * azimuth gap in radians) that should_split compares
    /// the actual gap against -- e.g. 1.5 means "split once the real gap
    /// exceeds 1.5x what pure angular resolution alone would produce".
    /// NOT a raw distance in meters (that was the pre-expected-distance
    /// meaning of this field; old tuned values around 0.1 don't carry over
    /// -- they'd make this almost never merge anything now).
    pub th_d: f32,
    /// first_segmentation's concavity-check angle threshold, degrees.
    pub th_z_deg: f32,
    pub th_d_second: f32,
    pub k_deg: f32,
    pub z_weight: f32,
    pub min_cluster_points: u32,
    /// Does double duty:
    /// 1. Absolute floor (meters) under both should_split's and
    ///    cross_ring_threshold's thresholds. This exists for short range:
    ///    both thresholds scale down toward zero as range shrinks (since
    ///    they're built on `expected = range * angle_gap`), which can fall
    ///    below the sensor's actual noise floor and make nearby points fail
    ///    to merge almost at random.
    /// 2. A dimensionless fraction in `node_dist`'s dz-gate (see its doc):
    ///    `min_gap_m * z_weight * expected_gap` is the dz below which two
    ///    cross-ring segments are treated as exactly coplanar (dz forced to
    ///    0) rather than merely close -- e.g. 0.5 zeroes out any dz smaller
    ///    than half the expected cross-ring spacing at that range/angle.
    pub min_gap_m: f32,
}

impl Default for SegParams {
    fn default() -> Self {
        Self {
            th_d: 5.0, th_z_deg: 5.0, th_d_second: 0.5, k_deg: 1.0, z_weight: 1.0,
            min_cluster_points: 10, min_gap_m: 0.5,
        }
    }
}

#[derive(Clone, Copy)]
pub struct RangeNode {
    pub x : f32,
    pub y : f32,
    pub z : f32,
    pub range : f32,
    // Calibrated azimuth this point was inserted under, hundredths of a
    // degree, [0, 36000). Kept alongside x/y/z so should_split can compute
    // the *actual* angular gap between two same-ring points, needed for the
    // range*angle expected-distance test -- the (x, y) alone don't recover
    // azimuth without redoing atan2, and re-deriving it would reintroduce
    // exactly the kind of precision loss/edge cases storing it avoids.
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
    // Unlike x_mean/y_mean, nothing read this until node_dist started
    // folding in a weighted dz -- two_layer_seg.cpp's GraphNode never had
    // one either, since its nodeDist/getNeighbors were XY-only too.
    pub z_mean : f32,
    pub alpha : i32,
    pub cluster : i32,
    pub start_pos : i32,
    pub end_pos : i32,
    // indices into G_r that belong to this node
    pub members : Vec<i32>
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
            members : Vec::new()
        }
    }
}

// Must match the decoder's LASER_NUM (rslidar_sdk_node.rs) -- the ring index
// passed into `RangeGraph::insert` is the raw hardware channel index, so the
// grid needs exactly one row per physical channel.
pub const NUM_RINGS: usize = 32;

// Azimuth-column width, in hundredths of a degree (matches
// two_layer_seg.cpp's actual `AZ_RES = deg2rad(0.5)` -- note its comment
// claims 0.4 deg, which is a stale comment/code mismatch in that file, not
// something to copy here).
pub const AZ_RES_CENTIDEG: i32 = 10;
pub const NUM_COLS: usize = (36000 / AZ_RES_CENTIDEG) as usize;

/// `(ring, azimuth-column)` grid of `RangeNode`s -- the "range graph" used
/// for fast neighbor lookups in segmentation. Unlike two_layer_seg.cpp's
/// `constructRangeGraph`, which has to guess a point's ring after the fact
/// by nearest-matching `asin(z/r)` against a fixed angle table, this is
/// populated directly from `RsHeliosDecoder::decode_msop` while it still
/// knows the real hardware channel (`chan`) and calibrated azimuth
/// (`angle_horiz_final`) for each point -- no re-derivation, no guessing.
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

    /// Inserts one decoded point into its `(ring, column)` cell. `azimuth_centideg`
    /// must already be normalized to `[0, 36000)` (see `azimuth_round` at the
    /// decode-loop call site). If another point already landed in this cell
    /// this revolution, keeps whichever is closer to the sensor -- mirrors
    /// `constructRangeGraph`'s `if (!G_r[idx].valid || r < G_r[idx].range)`.
    pub fn insert(
        &mut self,
        ring: usize,
        azimuth_centideg: i32,
        x: f32,
        y: f32,
        z: f32,
        range: f32,
        point_idx: i32,
    ) {
        let col = (azimuth_centideg / AZ_RES_CENTIDEG) as usize % NUM_COLS;
        let idx = ring * NUM_COLS + col;
        let cell = &mut self.nodes[idx];
        if !cell.valid || range < cell.range {
            *cell = RangeNode { x, y, z, range, azimuth_centideg, point_idx, valid: true, alpha: -1, beta: -1 };
        }
    }

    /// Smallest circular difference between two azimuths, hundredths of a
    /// degree, in [0, 18000] -- handles the same 359deg/0deg wraparound
    /// `first_segmentation`'s seam-merge step already has to deal with,
    /// since two real-angle-adjacent points can straddle that boundary.
    fn azimuth_gap_centideg(a: i32, b: i32) -> i32 {
        let raw = (a - b).rem_euclid(36000);
        raw.min(36000 - raw)
    }

    fn point_dist(&mut self, idx_a : usize, idx_b : usize) -> f32 {
        let dx = self.nodes[idx_a].x - self.nodes[idx_b].x;
        let dy = self.nodes[idx_a].y - self.nodes[idx_b].y;
        (dx * dx + dy * dy).sqrt()
    }

    fn get_vecs(&mut self, idx_cur : usize, idx_pre : usize, ring : usize) -> Option<[Vector2<f32>; 2]> {
        // in one horizontal ring, this is represented by one row in G_r: after seg, we get
        // |** * ***|        |*** * ****|, 2 different pcs
        //       idx_pre  idx_cur
        // idx_pre is the last cell of the previous segment (Pre Pos in Alg 1)
        // idx_cur is the first cell of the current segment

        let col_pre = idx_pre % NUM_COLS;
        let col_cur = idx_cur % NUM_COLS;

        // usize wraps the other way from C++'s signed int here: `col_pre - 1`
        // underflows (and panics in debug builds) when col_pre == 0, instead
        // of quietly going negative before `+ NUM_COLS` pulls it back into
        // range. Adding NUM_COLS first avoids ever computing a negative/
        // underflowed intermediate value.
        let col_pre_inner = (col_pre + NUM_COLS - 1) % NUM_COLS;
        let idx_pre_inner = ring * NUM_COLS + col_pre_inner;
        let col_cur_inner = (col_cur + 1) % NUM_COLS;
        let idx_cur_inner = ring * NUM_COLS + col_cur_inner;

        if !self.nodes[idx_pre_inner].valid || !self.nodes[idx_cur_inner].valid {
            return None;
        } 

        let p_pre = Vector2::<f32>::new(self.nodes[idx_pre].x, self.nodes[idx_pre].y);
        let p_pre_in = Vector2::<f32>::new(self.nodes[idx_pre_inner].x, self.nodes[idx_pre_inner].y);
        let v2 = p_pre_in - p_pre;

        let p_cur = Vector2::<f32>::new(self.nodes[idx_cur].x, self.nodes[idx_cur].y);
        let p_cur_in = Vector2::<f32>::new(self.nodes[idx_cur_inner].x, self.nodes[idx_cur_inner].y);
        let v1 = p_cur_in - p_cur;

        Some([v1, v2])
    }

    /// True if `idx_cur`/`idx_pre` should be different segments (the same
    /// distance/angle-bisector test the C++ inlines in `firstSegmentation`),
    /// false if they belong to the same one. Also false when `get_vecs` can't
    /// compute the geometry (not enough neighboring cells to judge) --
    /// mirrors that case just continuing the current segment rather than
    /// forcing a split it has no basis for.
    ///
    /// `th_d` is a dimensionless scale on the *expected* same-ring point
    /// spacing (arc length = average range * azimuth gap in radians) rather
    /// than a flat distance in meters: two points a given azimuth apart are
    /// naturally farther apart in real space the farther they are from the
    /// sensor, so comparing the actual gap to a fixed constant either splits
    /// too eagerly far away or merges too eagerly up close. Comparing it to
    /// a multiple of the purely-angular-resolution expected gap instead
    /// makes the test range-adaptive without needing a second parameter.
    /// `min_gap_m` floors the resulting threshold -- see its doc on
    /// `SegParams` for why the range-scaled threshold alone isn't enough.
    fn should_split(&mut self, idx_cur: usize, idx_pre: usize, ring: usize, th_d: f32, th_z: f32, min_gap_m: f32) -> bool {
        let dist = self.point_dist(idx_cur, idx_pre);
        let (v1, v2) = match self.get_vecs(idx_cur, idx_pre, ring) {
            Some(vecs) => (vecs[0], vecs[1]),
            None => return false,
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
        let avg_range = (self.nodes[idx_cur].range + self.nodes[idx_pre].range) / 2.0;
        let threshold = (th_d * avg_range * az_gap_rad).max(min_gap_m);

        dist > threshold || (angle < th_z && v_mo.dot(&v_anglebisector) > 0.0)
    }

    /// Labels same-ring segments (`RangeNode::alpha`) *and* builds
    /// `set_graph`'s `GraphNode`s for this revolution, in one pass over the
    /// range graph instead of two -- two_layer_seg.cpp (and this file, until
    /// now) did this as firstSegmentation followed by a separate full
    /// buildSetGraph traversal, which is exactly the extra traversal the
    /// paper's "we traverse the two-layer-graph structure twice" design
    /// doesn't call for: one pass over the range graph (labeling and node
    /// construction together), one pass over the set graph
    /// (second_segmentation). Accumulating each segment's GraphNode as its
    /// cells are labeled, rather than re-discovering segments from scratch
    /// afterward, gets down to that.
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
                let j_i32 = j as i32;
                if node.start_pos == -1 || j_i32 < node.start_pos {
                    node.start_pos = j_i32;
                }
                if node.end_pos == -1 || j_i32 > node.end_pos {
                    node.end_pos = j_i32;
                }
            }

            // Column 0 and NUM_COLS-1 are the same azimuth seam on an actual
            // 360deg ring, but the scan above treats them as the two ends of
            // a plain array -- so a segment straddling that seam (e.g.
            // anything spanning the 359deg/0deg boundary, which sits along
            // +X here) always gets cut into two, with no later stage ever
            // rejoining them: get_neighbors only looks at rings above/below,
            // never within the same ring. Re-run the same split test between
            // the ring's last valid column and its first, and merge them
            // back into one segment if the test says they shouldn't have
            // been split. Folding one seg_map entry's already-accumulated
            // sums/members into the other is cheap (bounded by that one
            // segment's size) and avoids re-scanning the whole ring the way
            // relabeling every cell individually would.
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
                    }
                }
            }

            // Drain the map into this ring's set-graph row, turning the
            // running sums above into means, then sort by start_pos so
            // segment order within a ring is deterministic (get_neighbors's
            // early-exit column-overlap check relies on this order).
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

/// Set graph `G_c`: one `Vec<GraphNode>` per ring, where each `GraphNode` is
/// one contiguous same-`alpha` segment from `firstSegmentation`, compressed
/// down to its mean position and the `RangeGraph` indices it covers. Unlike
/// `RangeGraph`, this is jagged (rings have different segment counts) and
/// gets fully rebuilt from a `RangeGraph` every revolution -- there's
/// nothing meaningful to pre-fill, so "empty" is just NUM_RINGS empty rows,
/// mirroring `G_c.assign(NUM_RINGS, {})` in two_layer_seg.cpp.
pub struct SetGraph {
    pub nodes: Vec<Vec<GraphNode>>
}

impl SetGraph {
    pub fn new() -> Self {
        Self { nodes: (0..NUM_RINGS).map(|_| Vec::new()).collect() }
    }

    /// Drops every ring back to empty, ready to be rebuilt by
    /// `RangeGraph::first_segmentation`. `Vec::new()` (in `new()`) and this
    /// both produce the same "NUM_RINGS empty rows" state; which one you
    /// reach for is just whether you already have a `SetGraph` to reuse.
    pub fn clear(&mut self) {
        for ring in self.nodes.iter_mut() {
            ring.clear();
        }
    }

    /// Distance between two segments' mean positions. If the Z separation is
    /// smaller than `dz_gate`, it's dropped entirely (treated as exactly
    /// coplanar) rather than merely down-weighted -- `TwoLayerNode::nodeDist`
    /// was XY-only (no z_mean existed at all) and this preserves that
    /// behavior for small dz, while still letting a *real* vertical gap
    /// (dz >= dz_gate) count fully and block the merge. Callers pass
    /// `min_gap_m * z_weight * expected_gap(...)`, not a flat constant: the
    /// dz two points on the same real surface should show grows with range
    /// for a fixed angular gap (same arc-length logic `expected_gap` already
    /// uses for the overall threshold), so the gate has to grow with range
    /// too, or genuinely-coplanar points stop merging vertically
    /// specifically as they get farther away.
    fn node_dist(a: &GraphNode, b: &GraphNode, dz_gate: f32) -> f32 {
        let dx = a.x_mean - b.x_mean;
        let dy = a.y_mean - b.y_mean;
        let mut dz = a.z_mean - b.z_mean;
        if dz.abs() < dz_gate {
            dz = 0.0;
        }
        if (a.range + b.range / 2.0) > 5.0 {
            (dx * dx + dy * dy + dz * dz).sqrt() / (a.range + b.range / 2.0) 
        } else {
            (dx * dx + dy * dy + dz * dz).sqrt() 
        }
    }

    /// Cross-ring merge threshold: `th_d_second`/`k_deg` are dimensionless
    /// scale factors on the *expected* cross-ring point spacing (arc length =
    /// average range * vertical-angle gap in radians), same idea as
    /// `should_split`'s `th_d` -- `expected` already grows with the real
    /// angular gap between the two rings (this LiDAR's vertical spacing is
    /// far from uniform: ~0.5deg near the horizon, 2.5-3deg at the
    /// extremes), so this replaces what used to be an ad hoc flat-plus-
    /// per-degree-constant approximation with the actual trigonometry.
    /// `th_d_second` is the base scale; `k_deg` adds a bit more scale per
    /// degree of gap on top, in case the base scale alone doesn't widen
    /// enough for the coarsest ring pairs.
    /// Expected cross-ring point spacing for two segments if they lie on one
    /// continuous, roughly-perpendicular-to-the-beam surface: arc length =
    /// average range * vertical-angle gap (radians). This is the one place
    /// range enters the cross-ring model -- both `cross_ring_threshold` and
    /// `dvert` (node_dist's Z-tolerance) build on it, so both grow with
    /// range together instead of only one of them doing so.
    fn expected_gap(ri: usize, rn: usize, a: &GraphNode, b: &GraphNode, ring_vert_deg: &[f32; NUM_RINGS]) -> f32 {
        let gap_rad = (ring_vert_deg[rn] - ring_vert_deg[ri]).abs().to_radians();
        let avg_range = (a.range + b.range) / 2.0;
        avg_range * gap_rad
    }

    fn cross_ring_threshold(
        th_d_second: f32,
        k_deg: f32,
        min_gap_m: f32,
        ri: usize,
        rn: usize,
        a: &GraphNode,
        b: &GraphNode,
        ring_vert_deg: &[f32; NUM_RINGS],
    ) -> f32 {
        let gap_deg = (ring_vert_deg[rn] - ring_vert_deg[ri]).abs();
        let expected = Self::expected_gap(ri, rn, a, b, ring_vert_deg);
        let scale = th_d_second + k_deg * gap_deg;
        (expected * scale).max(min_gap_m)
    }

    /// Candidate segments in the rings directly above/below `(ri, ci)` that
    /// are close enough to merge with it -- mirrors
    /// `TwoLayerNode::getNeighbors`. `th_d`/`k_deg`/`z_weight` have to be
    /// passed in (rather than read off `self`) since, unlike the C++
    /// version, nothing here stores them as fields. Uses `node_dist` (not an
    /// inline recomputation) so the candidate filter here and the actual
    /// merge decision in `second_segmentation` can never disagree.
    fn get_neighbors(
        &self,
        ri: usize,
        ci: usize,
        th_d_second: f32,
        k_deg: f32,
        z_weight: f32,
        min_gap_m: f32,
        ring_vert_deg: &[f32; NUM_RINGS],
    ) -> Vec<(usize, usize)> {
        let mut nbrs = Vec::new();
        let cur = &self.nodes[ri][ci];

        for dr in [-1i32, 1i32] {
            let rn = ri as i32 + dr;
            if rn < 0 || rn as usize >= NUM_RINGS {
                continue;
            }
            let rn = rn as usize;

            // Threshold/dist depend on this specific candidate's range (via
            // cross_ring_threshold's `expected`), not just the ring pair, so
            // this can't be hoisted out of the loop the way a flat threshold
            // could.
            let try_push = |cn: usize, nbrs: &mut Vec<(usize, usize)>| {
                let cand = &self.nodes[rn][cn];
                let threshold = Self::cross_ring_threshold(th_d_second, k_deg, min_gap_m, ri, rn, cur, cand, ring_vert_deg);
                // `min_gap_m * z_weight` scales the same range-scaled
                // expected_gap the overall threshold uses, not just the raw
                // angular gap: the real Z separation between adjacent rings
                // grows with range for a fixed angular gap (same arc-length
                // logic as everywhere else here), so a gate that ignored
                // range would stop zeroing out genuinely-coplanar dz once
                // points get far enough away.
                let dz_gate = min_gap_m * z_weight * Self::expected_gap(ri, rn, cur, cand, ring_vert_deg);
                if Self::node_dist(cur, cand, dz_gate) < th_d_second {
                    nbrs.push((rn, cn));
                }
            };

            // Column spans must overlap ("connected", per the paper) for
            // most candidates; rows are sorted by start_pos
            // (first_segmentation's sort), so once a candidate starts past
            // our end, every later one will too. But per Algorithm 2 / Figure 7, connected
            // candidates aren't the whole story: also check the single
            // nearest non-overlapping ("unconnected") candidate on each
            // side, so a boundary that's off by one discretization step
            // from a true continuation (plausible given real quantization/
            // noise) still gets a chance to merge instead of being silently
            // excluded just for missing the overlap by a hair.
            let mut last_before: Option<usize> = None;
            let mut first_after: Option<usize> = None;

            for cn in 0..self.nodes[rn].len() {
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

    /// Flood-fills adjacent segments (within `cross_ring_threshold`) across
    /// rings into final clusters, then writes each cluster id back onto the
    /// `RangeGraph` cells it came from -- mirrors
    /// `TwoLayerNode::secondSegmentation`. Takes `g_r` mutably because that
    /// final write-back step needs somewhere to put the result; the C++
    /// used a separate `cluster_ids_` array, but `RangeNode` already has a
    /// `beta` field set aside for exactly this.
    pub fn second_segmentation(
        &mut self,
        g_r: &mut RangeGraph,
        th_d_second: f32,
        k_deg: f32,
        z_weight: f32,
        min_cluster_points: u32,
        min_gap_m: f32,
        ring_vert_deg: &[f32; NUM_RINGS],
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
                    for (rn, cn) in self.get_neighbors(ri, ci, th_d_second, k_deg, z_weight, min_gap_m, ring_vert_deg) {
                        if self.nodes[rn][cn].cluster != -1 {
                            continue;
                        }
                        let dz_gate = min_gap_m * z_weight * Self::expected_gap(ri, rn, &self.nodes[ri][ci], &self.nodes[rn][cn], ring_vert_deg);
                        let dist = Self::node_dist(&self.nodes[ri][ci], &self.nodes[rn][cn], dz_gate);
                        let threshold = Self::cross_ring_threshold(
                            th_d_second, k_deg, min_gap_m, ri, rn,
                            &self.nodes[ri][ci], &self.nodes[rn][cn],
                            ring_vert_deg,
                        );
                        if dist < th_d_second {
                            self.nodes[rn][cn].cluster = l_cnt;
                            q.push_back((rn, cn));
                        }
                    }
                }
            }
        }

        // Total point count per cluster, so tiny ones can be dropped next --
        // mirrors two_layer_seg.cpp's extractClusters keeping only clusters
        // with >= 10 points. Without this, isolated range-noise outliers
        // (a single point whose neighbor comparison happened to fail) each
        // survive as their own 1-3 point "cluster" and get a distinct,
        // effectively arbitrary color -- verified this was happening on
        // real data: ~68% of all clusters had 3 points or fewer.
        let mut cluster_sizes: HashMap<i32, usize> = HashMap::new();
        for i in 0..NUM_RINGS {
            for node in &self.nodes[i] {
                *cluster_sizes.entry(node.cluster).or_insert(0) += node.members.len();
            }
        }

        // Propagate final cluster ids back to the RangeGraph cells each
        // GraphNode was built from, so a later extractClusters-equivalent
        // can group raw points by cluster. Clusters under min_cluster_points
        // get beta = -1 (same "no cluster" value RangeNode::new() already
        // uses) instead of their real id, so they render as unclustered
        // noise rather than a real (but meaningless) cluster color.
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