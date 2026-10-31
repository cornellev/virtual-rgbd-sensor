// Rust port of the pieces of ROS navigation's costmap_2d (noetic-devel) that
// turn one LiDAR revolution into an inflated costmap:
//   include/costmap_2d/cost_values.h -> the cost constants below
//   src/costmap_2d.cpp               -> Costmap2D (grid storage + coordinate math)
//   src/layered_costmap.cpp          -> Costmap::update (bounds -> reset -> layers)
// The two layers themselves live in obstacle_layer.rs and inflation_layer.rs.
//
// Everything ROS-specific (costmap_2d_ros, publisher, observation_buffer, TF,
// footprint) is left out: the sensor->map transform is a fixed translation
// (`sensor_x/y/z`), observations don't persist past one revolution, and the
// robot footprint is just `robot_radius`.

use nalgebra::Vector3;

use crate::inflation_layer::InflationLayer;
use crate::obstacle_layer::ObstacleLayer;
use crate::segmentation::RangeGraph;
use crate::transform::Transform;

pub const NO_INFORMATION: u8 = 255;
pub const LETHAL_OBSTACLE: u8 = 254;
pub const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;
pub const FREE_SPACE: u8 = 0;

/// What the obstacle layer forgets each revolution.
/// `Full`: everything -- the grid is exactly the latest scan (local map).
/// `Fov`: only cells the scan observed; the rest keep their last value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClearMode {
    Full,
    Fov,
}

impl std::str::FromStr for ClearMode {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "full" => Ok(Self::Full),
            "fov" => Ok(Self::Fov),
            other => Err(format!("clear_mode must be full or fov, got {other}")),
        }
    }
}
/// Same parameter names/meaning as the nav2 costmap.yaml
/// (costmap_lifecycle/costmap.yaml), flattened.
#[derive(Clone, Copy, Debug)]
pub struct CostmapParams {
    // global costmap
    pub width: f32,  // meters
    pub height: f32, // meters
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub robot_radius: f32,
    pub track_unknown_space: bool,
    pub clear_mode: ClearMode,

    // world-frame global map, fused from the local one every revolution
    pub global_width: f32,  // meters
    pub global_height: f32, // meters
    pub global_origin_x: f32,
    pub global_origin_y: f32,

    // sensor pose in the map frame (map -> base_link -> rslidar, translation only).
    // sensor_z is the LiDAR's mount height above the ground.
    pub sensor_x: f32,
    pub sensor_y: f32,
    pub sensor_z: f32,

    // obstacle_layer; both heights are above the ground, not the LiDAR
    pub min_obstacle_height: f32,
    pub max_obstacle_height: f32,
    pub obstacle_max_range: f32,
    pub obstacle_min_range: f32,
    pub raytrace_max_range: f32,
    pub raytrace_min_range: f32,
    /// Angular width of one clearing sector. Must be >= the LiDAR's horizontal
    /// resolution (0.2 deg for the Helios at 600 rpm) or some sectors never get
    /// a return and are never cleared.
    pub clearing_az_res_deg: f32,

    // inflation_layer
    pub inflation_radius: f32,
    pub cost_scaling_factor: f32,
}

impl Default for CostmapParams {
    fn default() -> Self {
        Self {
            width: 50.0,
            height: 50.0,
            resolution: 0.05,
            origin_x: -25.0,
            origin_y: -25.0,
            robot_radius: 0.3,
            track_unknown_space: false,
            clear_mode: ClearMode::Full,
            global_width: 100.0,
            global_height: 100.0,
            global_origin_x: -50.0,
            global_origin_y: -50.0,
            sensor_x: 0.0,
            sensor_y: 0.0,
            sensor_z: 0.0,
            min_obstacle_height: -3.0,
            max_obstacle_height: 3.0,
            obstacle_max_range: 15.0,
            obstacle_min_range: 0.2,
            raytrace_max_range: 20.0,
            raytrace_min_range: 0.2,
            clearing_az_res_deg: 0.4,
            inflation_radius: 0.3,
            cost_scaling_factor: 10.0,
        }
    }
}

/// Half-open cell rectangle `[min_x, max_x) x [min_y, max_y)`, same convention
/// as Costmap2D::resetMap / CostmapLayer::updateWithMax.
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub min_x: usize,
    pub min_y: usize,
    pub max_x: usize,
    pub max_y: usize,
}

impl Bounds {
    pub fn empty() -> Self {
        Self { min_x: usize::MAX, min_y: usize::MAX, max_x: 0, max_y: 0 }
    }

    pub fn is_empty(&self) -> bool {
        self.min_x >= self.max_x || self.min_y >= self.max_y
    }

    pub fn touch(&mut self, mx: usize, my: usize) {
        self.min_x = self.min_x.min(mx);
        self.min_y = self.min_y.min(my);
        self.max_x = self.max_x.max(mx + 1);
        self.max_y = self.max_y.max(my + 1);
    }

    pub fn union(&self, other: &Bounds) -> Bounds {
        Bounds {
            min_x: self.min_x.min(other.min_x),
            min_y: self.min_y.min(other.min_y),
            max_x: self.max_x.max(other.max_x),
            max_y: self.max_y.max(other.max_y),
        }
    }

    /// Grows by `cells` on every side, clipped to a `size_x` x `size_y` map.
    pub fn expand(&self, cells: usize, size_x: usize, size_y: usize) -> Bounds {
        if self.is_empty() {
            return *self;
        }
        Bounds {
            min_x: self.min_x.saturating_sub(cells),
            min_y: self.min_y.saturating_sub(cells),
            max_x: (self.max_x + cells).min(size_x),
            max_y: (self.max_y + cells).min(size_y),
        }
    }
}

/// costmap_2d.cpp: a row-major grid of u8 costs with world <-> map conversion.
pub struct Costmap2D {
    pub size_x: usize,
    pub size_y: usize,
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub data: Vec<u8>,
}

impl Costmap2D {
    pub fn new(size_x: usize, size_y: usize, resolution: f32, origin_x: f32, origin_y: f32, default_value: u8) -> Self {
        Self { size_x, size_y, resolution, origin_x, origin_y, data: vec![default_value; size_x * size_y] }
    }

    pub fn index(&self, mx: usize, my: usize) -> usize {
        my * self.size_x + mx
    }

    pub fn world_to_map(&self, wx: f32, wy: f32) -> Option<(usize, usize)> {
        if wx < self.origin_x || wy < self.origin_y {
            return None;
        }
        let mx = ((wx - self.origin_x) / self.resolution) as usize;
        let my = ((wy - self.origin_y) / self.resolution) as usize;
        if mx < self.size_x && my < self.size_y { Some((mx, my)) } else { None }
    }

    /// World coordinates of the cell's center.
    pub fn map_to_world(&self, mx: usize, my: usize) -> (f32, f32) {
        (
            self.origin_x + (mx as f32 + 0.5) * self.resolution,
            self.origin_y + (my as f32 + 0.5) * self.resolution,
        )
    }

    pub fn cell_distance(&self, world_dist: f32) -> usize {
        (world_dist / self.resolution).ceil().max(0.0) as usize
    }

    pub fn reset_map(&mut self, bounds: Bounds, value: u8) {
        if bounds.is_empty() {
            return;
        }
        // what this does is that it fills all visible cells at the current
        // lidar frame's FOV as NO_INFORMATION
        for my in bounds.min_y..bounds.max_y {
            let row = self.index(bounds.min_x, my);
            self.data[row..row + (bounds.max_x - bounds.min_x)].fill(value);
        }
    }
}

/// layered_costmap.cpp, hardwired to [obstacle_layer, inflation_layer] like
/// costmap.yaml's `plugins` list.
pub struct Costmap {
    master: Costmap2D,
    default_value: u8,
    obstacles: ObstacleLayer,
    inflation: InflationLayer,
    /// Cells the last `update` rewrote.
    last_bounds: Bounds,
}

impl Costmap {
    pub fn new(params: &CostmapParams) -> Self {
        let size_x = (params.width / params.resolution).round() as usize;
        let size_y = (params.height / params.resolution).round() as usize;
        let default_value = NO_INFORMATION;
        let master = Costmap2D::new(size_x, size_y, params.resolution, params.origin_x, params.origin_y, default_value);
        let obstacles = ObstacleLayer::new(params, &master, default_value);
        let inflation = InflationLayer::new(params, &master);
        Self { master, default_value, obstacles, inflation, last_bounds: Bounds::empty() }
    }

    /// LayeredCostmap::updateMap for one revolution: every layer grows the
    /// update bounds, the master grid is reset inside them, then each layer
    /// writes its costs in plugin order.
    pub fn update(&mut self, graph: &RangeGraph) {
        let bounds = self.obstacles.update_bounds(graph);
        // let bounds = self.inflation.update_bounds(bounds, &self.master);
        self.last_bounds = bounds;
        if bounds.is_empty() {
            return;
        }
        self.master.reset_map(bounds, self.default_value);
        self.obstacles.update_costs(&mut self.master, bounds);
        // self.inflation.update_costs(&mut self.master, bounds);
    }

    pub fn master(&self) -> &Costmap2D {
        &self.master
    }
}

/// World-frame occupancy accumulated over revolutions with FOV clearing:
/// each revolution, world cells the local map observed (FREE or LETHAL) take
/// the new value; cells it didn't observe keep their history.
pub struct GlobalMap {
    grid: Costmap2D,
    /// Where the lidar sits in the local map's own coordinates.
    sensor_x: f32,
    sensor_y: f32,
}

impl GlobalMap {
    pub fn new(params: &CostmapParams) -> Self {
        let size_x = (params.global_width / params.resolution).round() as usize;
        let size_y = (params.global_height / params.resolution).round() as usize;
        Self {
            grid: Costmap2D::new(
                size_x, size_y, params.resolution, params.global_origin_x, params.global_origin_y, NO_INFORMATION,
            ),
            sensor_x: params.sensor_x,
            sensor_y: params.sensor_y,
        }
    }

    pub fn grid(&self) -> &Costmap2D {
        &self.grid
    }

    /// Fuses `local` (lidar frame, just updated) using `tf` = lidar -> world.
    /// Planar: only yaw and x/y of `tf` are used. Iterates world cells and
    /// samples the local grid, so rotated scans leave no holes.
    pub fn fuse(&mut self, local: &Costmap, tf: &Transform) {
        let region = local.last_bounds;
        if region.is_empty() {
            return;
        }
        let l = local.master();
        let heading = tf.rotation * Vector3::x();
        let (sin, cos) = heading.y.atan2(heading.x).sin_cos();
        let (tx, ty) = (tf.translation.x, tf.translation.y);
        let (ox, oy) = (self.sensor_x, self.sensor_y);
        // local map coords -> world: rotate about the sensor, then translate.
        let to_world = |mx: f32, my: f32| {
            let (sx, sy) = (mx - ox, my - oy);
            (cos * sx - sin * sy + tx, sin * sx + cos * sy + ty)
        };

        let g = &mut self.grid;
        let (x0, y0) = (l.origin_x + region.min_x as f32 * l.resolution, l.origin_y + region.min_y as f32 * l.resolution);
        let (x1, y1) = (l.origin_x + region.max_x as f32 * l.resolution, l.origin_y + region.max_y as f32 * l.resolution);
        let corners = [to_world(x0, y0), to_world(x1, y0), to_world(x0, y1), to_world(x1, y1)];
        let cell = |w: f32, origin: f32, size: usize| ((w - origin) / g.resolution).floor().clamp(0.0, size as f32) as usize;
        let gx0 = cell(corners.iter().map(|c| c.0).fold(f32::INFINITY, f32::min), g.origin_x, g.size_x);
        let gx1 = cell(corners.iter().map(|c| c.0).fold(f32::NEG_INFINITY, f32::max), g.origin_x, g.size_x).saturating_add(1).min(g.size_x);
        let gy0 = cell(corners.iter().map(|c| c.1).fold(f32::INFINITY, f32::min), g.origin_y, g.size_y);
        let gy1 = cell(corners.iter().map(|c| c.1).fold(f32::NEG_INFINITY, f32::max), g.origin_y, g.size_y).saturating_add(1).min(g.size_y);

        // world -> local cell coords (inverse of `to_world`) is affine, so walk
        // it incrementally along each row instead of transforming every cell.
        let k = g.resolution / l.resolution;
        let (du, dv) = (cos * k, -sin * k);
        let (u_min, u_max) = (region.min_x as f32, region.max_x as f32);
        let (v_min, v_max) = (region.min_y as f32, region.max_y as f32);
        // Steps i along a row with p0 + i*dp in [lo, hi): the row's only cells
        // that can land inside the local region.
        let steps = |p0: f32, dp: f32, lo: f32, hi: f32| -> (f32, f32) {
            if dp.abs() < 1e-9 {
                return if p0 >= lo && p0 < hi { (0.0, f32::INFINITY) } else { (0.0, 0.0) };
            }
            let (a, b) = ((lo - p0) / dp, (hi - p0) / dp);
            (a.min(b).floor().max(0.0), a.max(b).ceil() + 1.0)
        };
        for gy in gy0..gy1 {
            let (wx, wy) = g.map_to_world(gx0, gy);
            let (dx, dy) = (wx - tx, wy - ty);
            let u0 = (cos * dx + sin * dy + ox - l.origin_x) / l.resolution;
            let v0 = (-sin * dx + cos * dy + oy - l.origin_y) / l.resolution;
            let (ua, ub) = steps(u0, du, u_min, u_max);
            let (va, vb) = steps(v0, dv, v_min, v_max);
            let first = ua.max(va);
            let last = ub.min(vb).min((gx1 - gx0) as f32);
            if first >= last {
                continue;
            }
            let (first, last) = (first as usize, last as usize);
            let (mut u, mut v) = (u0 + first as f32 * du, v0 + first as f32 * dv);
            let row = g.index(0, gy);
            for gx in gx0 + first..gx0 + last {
                if u >= u_min && u < u_max && v >= v_min && v < v_max {
                    let c = l.data[v as usize * l.size_x + u as usize];
                    if c != NO_INFORMATION {
                        g.data[row + gx] = c;
                    }
                }
                u += du;
                v += dv;
            }
        }
    }
}
