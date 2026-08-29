// Perception wrapper for camera and LiDAR sensors
// Inputs:
//      ZED camera
//      LiDAR

// Outputs:
//      Camera + LiDAR-fused RGBD
//      LiDAR-based depth map
//      Occupancy grid :: nav2_msgs::msg::Costmap;

// src/main.rs
use rclrs::{Context, QOS_PROFILE_SENSOR_DATA};
use sensor_msgs::msg::{Image, PointCloud2};
use nav2_msgs::msg::{Costmap, CostmapMetaData};
use geometry_msgs::msg::{Pose, Point, Quaternion};
use std::sync::{Arc, Mutex};

// ---- Config ----
const RESOLUTION: f64 = 0.05;
const WIDTH_M: f64 = 50.0;
const HEIGHT_M: f64 = 50.0;
const WIDTH_CELLS: usize = (WIDTH_M / RESOLUTION) as usize;
const HEIGHT_CELLS: usize = (HEIGHT_M / RESOLUTION) as usize;
const ORIGIN_X: f64 = -25.0;
const ORIGIN_Y: f64 = -25.0;

const MIN_OBSTACLE_HEIGHT: f32 = -0.2;
const MAX_OBSTACLE_HEIGHT: f32 = 2.0;
const OBSTACLE_MAX_RANGE: f32 = 15.0;

const ROBOT_RADIUS: f64 = 0.3;
const INFLATION_RADIUS: f64 = 0.55;
const COST_SCALING_FACTOR: f64 = 10.0;

const FREE_SPACE: u8 = 0;
const LETHAL_OBSTACLE: u8 = 254;
const NO_INFORMATION: u8 = 255;
const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;

const SYNC_TOLERANCE_NS: i64 = 50_000_000; // 50ms

fn stamp_to_ns(stamp: &builtin_interfaces::msg::Time) -> i64 {
    stamp.sec as i64 * 1_000_000_000 + stamp.nanosec as i64
}

struct Grid {
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    cells: Vec<u8>,
}

impl Grid {
    fn new(width: usize, height: usize, resolution: f64, origin_x: f64, origin_y: f64) -> Self {
        Self { width, height, resolution, origin_x, origin_y, cells: vec![NO_INFORMATION; width * height] }
    }

    fn world_to_map(&self, wx: f64, wy: f64) -> Option<(usize, usize)> {
        if wx < self.origin_x || wy < self.origin_y { return None; }
        let mx = ((wx - self.origin_x) / self.resolution) as usize;
        let my = ((wy - self.origin_y) / self.resolution) as usize;
        if mx >= self.width || my >= self.height { return None; }
        Some((mx, my))
    }

    fn clear_to_free(&mut self) {
        self.cells.iter_mut().for_each(|c| *c = FREE_SPACE);
    }

    fn mark_obstacle(&mut self, wx: f64, wy: f64) {
        if let Some((mx, my)) = self.world_to_map(wx, wy) {
            self.cells[my * self.width + mx] = LETHAL_OBSTACLE;
        }
    }

    fn inflate(&mut self) {
        let cell_radius = (INFLATION_RADIUS / self.resolution).ceil() as i32;
        let lethal: Vec<(i32, i32)> = self.cells.iter().enumerate()
            .filter(|(_, &c)| c == LETHAL_OBSTACLE)
            .map(|(i, _)| ((i % self.width) as i32, (i / self.width) as i32))
            .collect();

        let mut inflated = self.cells.clone();
        for (lx, ly) in lethal {
            for dy in -cell_radius..=cell_radius {
                for dx in -cell_radius..=cell_radius {
                    let x = lx + dx;
                    let y = ly + dy;
                    if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height { continue; }
                    let dist = ((dx * dx + dy * dy) as f64).sqrt() * self.resolution;
                    if dist > INFLATION_RADIUS { continue; }
                    let i = (y as usize) * self.width + (x as usize);
                    if inflated[i] == LETHAL_OBSTACLE { continue; }
                    let cost = if dist <= ROBOT_RADIUS {
                        INSCRIBED_INFLATED_OBSTACLE
                    } else {
                        let factor = (-COST_SCALING_FACTOR * (dist - ROBOT_RADIUS)).exp();
                        ((INSCRIBED_INFLATED_OBSTACLE as f64 - 1.0) * factor) as u8
                    };
                    if cost > inflated[i] { inflated[i] = cost; }
                }
            }
        }
        self.cells = inflated;
    }

    /// u8 costs -> int8 wire format, same byte reinterpretation nav2 uses
    /// (254 -> -2, 255 -> -1, etc.)
    fn to_i8_data(&self) -> Vec<i8> {
        self.cells.iter().map(|&c| c as i8).collect()
    }
}

fn iter_xyz(cloud: &PointCloud2) -> impl Iterator<Item = (f32, f32, f32)> + '_ {
    let point_step = cloud.point_step as usize;
    let data = &cloud.data;
    let offset_of = |name: &str| cloud.fields.iter().find(|f| f.name == name).map(|f| f.offset as usize);
    let x_off = offset_of("x").unwrap_or(0);
    let y_off = offset_of("y").unwrap_or(4);
    let z_off = offset_of("z").unwrap_or(8);
    let n = if point_step > 0 { data.len() / point_step } else { 0 };
    (0..n).filter_map(move |i| {
        let base = i * point_step;
        let read = |off: usize| -> Option<f32> {
            Some(f32::from_le_bytes(data.get(base + off..base + off + 4)?.try_into().ok()?))
        };
        Some((read(x_off)?, read(y_off)?, read(z_off)?))
    })
}

struct SyncBuffers {
    lidar: Mutex<Vec<PointCloud2>>,
    camera: Mutex<Vec<Image>>,
    grid: Mutex<Grid>,
    publisher: rclrs::Publisher<Costmap>,
}

impl SyncBuffers {
    fn push_lidar(&self, msg: PointCloud2) {
        let mut buf = self.lidar.lock().unwrap();
        buf.push(msg);
        if buf.len() > 30 { buf.remove(0); }
        drop(buf);
        self.try_match();
    }

    fn push_camera(&self, msg: Image) {
        let mut buf = self.camera.lock().unwrap();
        buf.push(msg);
        if buf.len() > 30 { buf.remove(0); }
        drop(buf);
        self.try_match();
    }

    fn try_match(&self) {
        let mut lidar_buf = self.lidar.lock().unwrap();
        let mut camera_buf = self.camera.lock().unwrap();

        let mut best: Option<(usize, usize, i64)> = None;
        for (i, l) in lidar_buf.iter().enumerate() {
            let l_t = stamp_to_ns(&l.header.stamp);
            for (j, c) in camera_buf.iter().enumerate() {
                let diff = (l_t - stamp_to_ns(&c.header.stamp)).abs();
                if diff <= SYNC_TOLERANCE_NS && best.map_or(true, |(_, _, d)| diff < d) {
                    best = Some((i, j, diff));
                }
            }
        }

        if let Some((i, j, _)) = best {
            let lidar_msg = lidar_buf.remove(i);
            let camera_msg = camera_buf.remove(j);
            let l_t = stamp_to_ns(&lidar_msg.header.stamp);
            let c_t = stamp_to_ns(&camera_msg.header.stamp);
            lidar_buf.retain(|m| stamp_to_ns(&m.header.stamp) > l_t);
            camera_buf.retain(|m| stamp_to_ns(&m.header.stamp) > c_t);
            drop(lidar_buf);
            drop(camera_buf);

            self.on_synced_pair(lidar_msg, camera_msg);
        }
    }

    fn on_synced_pair(&self, lidar: PointCloud2, _camera: Image) {
        // NOTE: _camera is unused for now — fusion extension point.
        // e.g. project lidar points into the image, mask by segmentation,
        // or drop points that fall in a "known-ground" image region.
        let mut grid = self.grid.lock().unwrap();
        grid.clear_to_free();

        for (x, y, z) in iter_xyz(&lidar) {
            if z < MIN_OBSTACLE_HEIGHT || z > MAX_OBSTACLE_HEIGHT { continue; }
            if (x * x + y * y).sqrt() > OBSTACLE_MAX_RANGE { continue; }
            grid.mark_obstacle(x as f64, y as f64);
        }
        grid.inflate();

        let msg = Costmap {
            header: std_msgs::msg::Header {
                stamp: lidar.header.stamp.clone(),
                frame_id: lidar.header.frame_id.clone(),
            },
            metadata: CostmapMetaData {
                map_load_time: lidar.header.stamp.clone(),
                update_time: lidar.header.stamp.clone(),
                layer: "fused_perception".to_string(),
                resolution: grid.resolution as f32,
                size_x: grid.width as u32,
                size_y: grid.height as u32,
                origin: Pose {
                    position: Point { x: grid.origin_x, y: grid.origin_y, z: 0.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
            },
            data: grid.to_i8_data(),
        };

        if let Err(e) = self.publisher.publish(&msg) {
            eprintln!("failed to publish costmap: {e}");
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("perception_costmap_node")?;

    let publisher = node.create_publisher::<Costmap>(
        "/perception/costmap_raw",
        QOS_PROFILE_SENSOR_DATA,
    )?;

    let sync = Arc::new(SyncBuffers {
        lidar: Mutex::new(Vec::new()),
        camera: Mutex::new(Vec::new()),
        grid: Mutex::new(Grid::new(WIDTH_CELLS, HEIGHT_CELLS, RESOLUTION, ORIGIN_X, ORIGIN_Y)),
        publisher,
    });

    let sync_lidar = Arc::clone(&sync);
    let _lidar_sub = node.create_subscription::<PointCloud2, _>(
        "/rslidar_points",
        QOS_PROFILE_SENSOR_DATA,
        move |msg: PointCloud2| sync_lidar.push_lidar(msg),
    )?;

    let sync_camera = Arc::clone(&sync);
    let _camera_sub = node.create_subscription::<Image, _>(
        "/camera/image_raw",
        QOS_PROFILE_SENSOR_DATA,
        move |msg: Image| sync_camera.push_camera(msg),
    )?;

    executor.spin(rclrs::SpinOptions::default())?;
    Ok(())
}
