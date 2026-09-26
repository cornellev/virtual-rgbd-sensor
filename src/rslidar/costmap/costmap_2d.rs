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

use crate::inflation_layer::InflationLayer;
use crate::obstacle_layer::ObstacleLayer;
use crate::segmentation::RangeGraph;

pub const NO_INFORMATION: u8 = 255;
pub const LETHAL_OBSTACLE: u8 = 254;
pub const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;
pub const FREE_SPACE: u8 = 0;

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

    // sensor pose in the map frame (map -> base_link -> rslidar, translation only)
    pub sensor_x: f32,
    pub sensor_y: f32,
    pub sensor_z: f32,

    // obstacle_layer
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
            sensor_x: 0.0,
            sensor_y: 0.0,
            sensor_z: 0.0,
            min_obstacle_height: -0.2,
            max_obstacle_height: 2.0,
            obstacle_max_range: 15.0,
            obstacle_min_range: 0.0,
            raytrace_max_range: 20.0,
            raytrace_min_range: 0.0,
            clearing_az_res_deg: 0.4,
            inflation_radius: 0.55,
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
}

impl Costmap {
    pub fn new(params: &CostmapParams) -> Self {
        let size_x = (params.width / params.resolution).round() as usize;
        let size_y = (params.height / params.resolution).round() as usize;
        let default_value = if params.track_unknown_space { NO_INFORMATION } else { FREE_SPACE };
        let master = Costmap2D::new(size_x, size_y, params.resolution, params.origin_x, params.origin_y, default_value);
        let obstacles = ObstacleLayer::new(params, &master, default_value);
        let inflation = InflationLayer::new(params, &master);
        Self { master, default_value, obstacles, inflation }
    }

    /// LayeredCostmap::updateMap for one revolution: every layer grows the
    /// update bounds, the master grid is reset inside them, then each layer
    /// writes its costs in plugin order.
    pub fn update(&mut self, graph: &RangeGraph) {
        let bounds = self.obstacles.update_bounds(graph);
        let bounds = self.inflation.update_bounds(bounds, &self.master);
        if bounds.is_empty() {
            return;
        }
        self.master.reset_map(bounds, self.default_value);
        self.obstacles.update_costs(&mut self.master, bounds);
        self.inflation.update_costs(&mut self.master, bounds);
    }

    pub fn master(&self) -> &Costmap2D {
        &self.master
    }
}
