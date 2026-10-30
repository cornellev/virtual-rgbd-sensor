// Rust port of plugins/obstacle_layer.cpp (costmap_2d, noetic-devel), fed
// straight from the RangeGraph instead of a PointCloud2 observation buffer.
//
// Marking is the same as ObstacleLayer::updateBounds: every point whose map
// z is within [min_obstacle_height, max_obstacle_height] and whose range is
// within [obstacle_min_range, obstacle_max_range] sets its cell LETHAL.
//
// Clearing is where this differs. raytraceFreespace walks a Bresenham line
// from the sensor to *every* point. Since the sensor never moves relative to
// the map here, each cell's azimuth sector and distance from the sensor are
// fixed, so they're precomputed once. Per revolution we only need the
// farthest in-height return per sector: every cell in that sector closer
// than it is free. That's one linear pass over the window instead of
// ~100k ray walks, with the same result as long as each sector gets at
// least one return (see CostmapParams::clearing_az_res_deg).

use crate::costmap_2d::{Bounds, ClearMode, Costmap2D, CostmapParams, LETHAL_OBSTACLE, FREE_SPACE, NO_INFORMATION};
use crate::segmentation::{RangeGraph, AZ_RES_CENTIDEG, NUM_COLS};

const NO_SECTOR: u32 = u32::MAX;

/// Clearing sector of the horizontal direction (dx, dy) from the sensor.
/// Shared by the per-cell table and the per-point lookup so the two always
/// agree on which way a ray points.
fn sector_of(dx: f32, dy: f32, cols_per_sector: usize) -> usize {
    let az = ((dy.atan2(dx).to_degrees() * 100.0).round() as i32).rem_euclid(36000);
    (az / AZ_RES_CENTIDEG) as usize % NUM_COLS / cols_per_sector
}

pub struct ObstacleLayer {
    /// The layer's own grid. With `ClearMode::Fov` it persists across
    /// revolutions like ObstacleLayer's costmap_ (a cell stays LETHAL until a
    /// ray clears it); with `ClearMode::Full` it's rebuilt from each scan.
    grid: Costmap2D,
    clear_mode: ClearMode,
    default_value: u8,
    /// Last revolution's marks, which can lie outside `window` when
    /// obstacle_max_range > raytrace_max_range.
    prev_marks: Bounds,

    sensor_x: f32,
    sensor_y: f32,
    sensor_z: f32,
    min_obstacle_height: f32,
    max_obstacle_height: f32,
    obstacle_max_range: f32,
    obstacle_min_range: f32,
    raytrace_min_range: f32, // deadzone of the lidar: 0.2m
    raytrace_max_range: f32,

    /// Cells within raytrace_max_range of the sensor; the only cells clearing
    /// can touch.
    window: Bounds,
    /// Per window cell (row-major over `window`): its clearing sector, or
    /// NO_SECTOR if it's outside [raytrace_min_range, raytrace_max_range].
    cell_sector: Vec<u32>,
    /// Per window cell: horizontal distance from the sensor to its center.
    cell_range: Vec<f32>,
    cols_per_sector: usize,

    // per-revolution scratch, kept to avoid reallocating every frame
    sector_clear_range: Vec<f32>,
    marks: Vec<(usize, usize)>,
}

impl ObstacleLayer {
    pub fn new(params: &CostmapParams, master: &Costmap2D, default_value: u8) -> Self {
        let grid = Costmap2D::new(
            master.size_x, master.size_y, master.resolution, master.origin_x, master.origin_y, default_value,
        );

        let cols_per_sector =
            ((params.clearing_az_res_deg * 100.0 / AZ_RES_CENTIDEG as f32).round() as usize).max(1);
        let num_sectors = NUM_COLS.div_ceil(cols_per_sector);

        // Bounding box of the raytrace circle, clipped to the map.
        let r = params.raytrace_max_range;
        let mut window = Bounds::empty();
        for (wx, wy) in [
            (params.sensor_x - r, params.sensor_y - r),
            (params.sensor_x + r, params.sensor_y + r),
        ] {
            let wx = wx.clamp(grid.origin_x, grid.origin_x + grid.size_x as f32 * grid.resolution - 1e-3);
            let wy = wy.clamp(grid.origin_y, grid.origin_y + grid.size_y as f32 * grid.resolution - 1e-3);
            if let Some((mx, my)) = grid.world_to_map(wx, wy) {
                window.touch(mx, my);
            }
        }

        let mut cell_sector = Vec::new();
        let mut cell_range = Vec::new();
        if !window.is_empty() {
            let n = (window.max_x - window.min_x) * (window.max_y - window.min_y);
            cell_sector.reserve(n);
            cell_range.reserve(n);
            for my in window.min_y..window.max_y {
                for mx in window.min_x..window.max_x {
                    let (wx, wy) = grid.map_to_world(mx, my);
                    let (dx, dy) = (wx - params.sensor_x, wy - params.sensor_y);
                    let range = dx.hypot(dy);
                    let sector = if range < params.raytrace_min_range || range > params.raytrace_max_range {
                        NO_SECTOR
                    } else {
                        sector_of(dx, dy, cols_per_sector) as u32
                    };
                    cell_sector.push(sector);
                    cell_range.push(range);
                }
            }
        }

        Self {
            grid,
            clear_mode: params.clear_mode,
            default_value,
            prev_marks: Bounds::empty(),
            sensor_x: params.sensor_x,
            sensor_y: params.sensor_y,
            sensor_z: params.sensor_z,
            min_obstacle_height: -3.0,
            max_obstacle_height: 3.0,
            obstacle_max_range: params.obstacle_max_range,
            obstacle_min_range: params.obstacle_min_range,
            raytrace_min_range: params.raytrace_min_range,
            raytrace_max_range: params.raytrace_max_range,
            window,
            cell_sector,
            cell_range,
            cols_per_sector,
            sector_clear_range: vec![0.0; num_sectors],
            marks: Vec::new(),
        }
    }

    /// ObstacleLayer::updateBounds: clears then marks this revolution's
    /// returns into the layer grid, returning the cells that may have changed.
    pub fn update_bounds(&mut self, graph: &RangeGraph) -> Bounds {
        self.sector_clear_range.fill(0.0);
        self.marks.clear();

        for node in graph.nodes.iter() {
            if !node.valid {
                continue;
            }
            // ObservationBuffer drops out-of-height points for both marking
            // and clearing, so do the same.
            let wz = node.z + self.sensor_z;
            if wz < self.min_obstacle_height || wz > self.max_obstacle_height {
                continue;
            }

            let range = node.x.hypot(node.y);
            // From the point's position, not its range-graph column: the column
            // is the raw packet azimuth, which disagrees with (x, y) whenever the
            // decoder flips axes (e.g. mirror_y), and then rays would clear the
            // mirror-image sector of where the obstacle is marked.
            let sector = sector_of(node.x, node.y, self.cols_per_sector);
            // raytraceFreespace clips rays longer than raytrace_max_range.
            let clear = range.min(self.raytrace_max_range);
            if node.range > self.raytrace_min_range && clear > self.sector_clear_range[sector] {
                self.sector_clear_range[sector] = clear;
            }

            if range >= self.obstacle_min_range && range <= self.obstacle_max_range {
                if let Some(cell) = self.grid.world_to_map(node.x + self.sensor_x, node.y + self.sensor_y) {
                    self.marks.push(cell);
                }
            }
        }

        // Everything this revolution can touch, plus last revolution's marks so
        // the master grid is reset over them too.
        let bounds = self.window.union(&self.prev_marks);
        if self.clear_mode == ClearMode::Full {
            self.grid.reset_map(bounds, self.default_value);
        }

        // Clear first, then mark, same order as updateBounds, so a ray passing
        // over a low obstacle can't erase it. Unlike raytraceLine this also
        // clears the endpoint cell, which doesn't matter since obstacles are
        // re-marked right after.
        if !self.window.is_empty() {
            let w = self.window.max_x - self.window.min_x;
            for my in self.window.min_y..self.window.max_y {
                let row = self.grid.index(self.window.min_x, my);
                let lut = (my - self.window.min_y) * w;
                for k in 0..w {
                    let sector = self.cell_sector[lut + k];
                    if sector != NO_SECTOR && self.cell_range[lut + k] < self.sector_clear_range[sector as usize] {
                        self.grid.data[row + k] = FREE_SPACE;
                    }
                }
            }
        }

        let mut marked = Bounds::empty();
        for &(mx, my) in &self.marks {
            let idx = self.grid.index(mx, my);
            self.grid.data[idx] = LETHAL_OBSTACLE;
            marked.touch(mx, my);
        }
        self.prev_marks = marked;

        bounds.union(&marked)
    }

    /// CostmapLayer::updateWithMax.
    pub fn update_costs(&self, master: &mut Costmap2D, bounds: Bounds) {
        for my in bounds.min_y..bounds.max_y {
            let row = self.grid.index(0, my);
            for mx in bounds.min_x..bounds.max_x {
                let cost = self.grid.data[row + mx];
                if cost == NO_INFORMATION {
                    continue;
                }
                let old = &mut master.data[row + mx];
                if *old == NO_INFORMATION || *old < cost {
                    *old = cost;
                }
            }
        }
    }
}
