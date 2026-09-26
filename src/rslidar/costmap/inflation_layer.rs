// Rust port of plugins/inflation_layer.cpp (costmap_2d, noetic-devel).
//
// Same algorithm: seed every LETHAL cell, then grow outward in order of
// distance to the nearest obstacle, writing computeCost(distance) with max().
// The distance-ordered queue (std::map<double, vector<CellData>> in ROS1) is
// a Vec of bins keyed on the integer squared cell distance dx*dx + dy*dy,
// which sorts the same way without any float keys.

use crate::costmap_2d::{
    Bounds, Costmap2D, CostmapParams, FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION,
};

#[derive(Clone, Copy)]
struct CellData {
    index: usize,
    x: usize,
    y: usize,
    src_x: usize,
    src_y: usize,
}

pub struct InflationLayer {
    resolution: f32,
    inscribed_radius: f32,
    cost_scaling_factor: f32,
    inflate_unknown: bool,
    cell_inflation_radius: usize,

    /// computeCaches: cost for a cell (dx, dy) away from its source obstacle,
    /// indexed dx * (cell_inflation_radius + 1) + dy.
    cached_costs: Vec<u8>,
    /// Indexed by dx*dx + dy*dy; only squared distances <= radius^2 are used.
    bins: Vec<Vec<CellData>>,
    seen: Vec<bool>,
}

impl InflationLayer {
    pub fn new(params: &CostmapParams, master: &Costmap2D) -> Self {
        let cell_inflation_radius = master.cell_distance(params.inflation_radius);
        let mut layer = Self {
            resolution: master.resolution,
            inscribed_radius: params.robot_radius,
            cost_scaling_factor: params.cost_scaling_factor,
            inflate_unknown: false,
            cell_inflation_radius,
            cached_costs: Vec::new(),
            bins: vec![Vec::new(); cell_inflation_radius * cell_inflation_radius + 1],
            seen: vec![false; master.size_x * master.size_y],
        };

        let n = cell_inflation_radius + 1;
        layer.cached_costs = vec![0; n * n];
        for dx in 0..n {
            for dy in 0..n {
                let distance = (dx as f32).hypot(dy as f32);
                layer.cached_costs[dx * n + dy] = layer.compute_cost(distance);
            }
        }
        layer
    }

    /// InflationLayer::computeCost, `distance` in cells.
    fn compute_cost(&self, distance: f32) -> u8 {
        if distance == 0.0 {
            LETHAL_OBSTACLE
        } else if distance * self.resolution <= self.inscribed_radius {
            INSCRIBED_INFLATED_OBSTACLE
        } else {
            let factor = (-self.cost_scaling_factor * (distance * self.resolution - self.inscribed_radius)).exp();
            ((INSCRIBED_INFLATED_OBSTACLE - 1) as f32 * factor) as u8
        }
    }

    /// InflationLayer::updateBounds: an obstacle change can raise costs up to
    /// inflation_radius away, so those cells have to be reset and redone too.
    pub fn update_bounds(&self, bounds: Bounds, master: &Costmap2D) -> Bounds {
        bounds.expand(self.cell_inflation_radius, master.size_x, master.size_y)
    }

    /// InflationLayer::updateCosts.
    pub fn update_costs(&mut self, master: &mut Costmap2D, bounds: Bounds) {
        let r = self.cell_inflation_radius;
        if r == 0 || bounds.is_empty() {
            return;
        }

        // Obstacles up to r cells outside `bounds` still inflate into it.
        let window = bounds.expand(r, master.size_x, master.size_y);
        for my in window.min_y..window.max_y {
            let row = master.index(window.min_x, my);
            self.seen[row..row + (window.max_x - window.min_x)].fill(false);
        }

        for my in window.min_y..window.max_y {
            for mx in window.min_x..window.max_x {
                let index = master.index(mx, my);
                if master.data[index] == LETHAL_OBSTACLE {
                    self.bins[0].push(CellData { index, x: mx, y: my, src_x: mx, src_y: my });
                }
            }
        }

        let n = r + 1;
        for bin in 0..self.bins.len() {
            while let Some(cell) = self.bins[bin].pop() {
                if self.seen[cell.index] {
                    continue;
                }
                self.seen[cell.index] = true;

                let dx = cell.x.abs_diff(cell.src_x);
                let dy = cell.y.abs_diff(cell.src_y);
                let cost = self.cached_costs[dx * n + dy];
                let old = master.data[cell.index];
                master.data[cell.index] = if old == NO_INFORMATION
                    && (if self.inflate_unknown { cost > FREE_SPACE } else { cost >= INSCRIBED_INFLATED_OBSTACLE })
                {
                    cost
                } else {
                    old.max(cost)
                };

                if cell.x > window.min_x {
                    self.enqueue(cell.index - 1, cell.x - 1, cell.y, cell.src_x, cell.src_y);
                }
                if cell.y > window.min_y {
                    self.enqueue(cell.index - master.size_x, cell.x, cell.y - 1, cell.src_x, cell.src_y);
                }
                if cell.x + 1 < window.max_x {
                    self.enqueue(cell.index + 1, cell.x + 1, cell.y, cell.src_x, cell.src_y);
                }
                if cell.y + 1 < window.max_y {
                    self.enqueue(cell.index + master.size_x, cell.x, cell.y + 1, cell.src_x, cell.src_y);
                }
            }
        }

        // A neighbor can land in a bin we've already passed (a step back
        // toward its source). ROS1 drops those too, but they'd otherwise
        // leak into the next revolution here.
        for bin in &mut self.bins {
            bin.clear();
        }
    }

    fn enqueue(&mut self, index: usize, x: usize, y: usize, src_x: usize, src_y: usize) {
        if self.seen[index] {
            return;
        }
        let dx = x.abs_diff(src_x);
        let dy = y.abs_diff(src_y);
        let dist_sq = dx * dx + dy * dy;
        let r = self.cell_inflation_radius;
        if dist_sq > r * r {
            return;
        }
        self.bins[dist_sq].push(CellData { index, x, y, src_x, src_y });
    }
}
