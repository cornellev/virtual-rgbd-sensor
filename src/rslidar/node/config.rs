// ---------------------------------------------------------------------------
// config.yaml loading (only the scalar keys this script needs)
// ---------------------------------------------------------------------------

use crate::costmap_2d;

/// Finds a `<key>: <value>` line anywhere in the file and returns the value
/// with any trailing `# comment` stripped. Good enough for config.yaml's flat
/// key layout; this is not a general YAML parser.
/// also like if "pcap_path" is empty, default to online LiDAR, if that exist
/// if "pcap_path" contains a valid path, default to the offline LiDAR
fn find_scalar(text: &str, key: &str) -> Option<String> {
    let prefix = format!("{key}:");
    for line in text.lines() {
        let trimmed = line.trim_start();
        if let Some(rest) = trimmed.strip_prefix(&prefix) {
            let value = rest.split('#').next().unwrap_or("").trim();
            if !value.is_empty() {
                return Some(value.to_string());
            }
        }
    }
    None
}

fn scalar_or<T: std::str::FromStr>(text: &str, key: &str, default: T) -> T {
    find_scalar(text, key)
        .and_then(|v| v.parse::<T>().ok())
        .unwrap_or(default)
}

#[derive(Clone, Debug)]
pub struct DriverConfig {
    pub host_address: String,
    pub lidar_type: String,
    pub msop_port: u16,
    pub difop_port: u16,
    pub user_layer_bytes: u16,
    pub tail_layer_bytes: u16,
    pub min_distance: f32,
    pub max_distance: f32,
    pub use_lidar_clock: bool,
    pub dense_points: bool,
    pub ts_first_point: bool,
    pub start_angle: f32,
    pub end_angle: f32,
    pub wait_for_difop: bool,
    pub frame_id: String,
    // TEMPORARY: cev-sim encodes MSOP azimuth counterclockwise (hardware is
    // clockwise), so its scans come out mirrored in y. Remove once fixed upstream.
    pub mirror_y: bool,

    // Offline playback: when `pcap_path` is set, MSOP/DIFOP packets are read
    // from that capture instead of live UDP sockets. `pcap_rate` scales
    // playback speed relative to the capture's own timestamps (1.0 =
    // realtime, 2.0 = 2x speed, etc); `pcap_repeat` loops the file forever.
    pub pcap_path: Option<String>,
    pub pcap_rate: f32,
    pub pcap_repeat: bool,
}

impl DriverConfig {
    pub fn parse(text: &str) -> Self {
        Self {
            host_address: find_scalar(text, "host_address").unwrap_or_else(|| "0.0.0.0".into()),
            lidar_type: find_scalar(text, "lidar_type").unwrap_or_else(|| "RSHELIOS".into()),
            msop_port: scalar_or(text, "msop_port", 6699u16),
            difop_port: scalar_or(text, "difop_port", 7788u16),
            user_layer_bytes: scalar_or(text, "user_layer_bytes", 0u16),
            tail_layer_bytes: scalar_or(text, "tail_layer_bytes", 0u16),
            min_distance: scalar_or(text, "min_distance", 0.0f32),
            max_distance: scalar_or(text, "max_distance", 0.0f32),
            use_lidar_clock: scalar_or(text, "use_lidar_clock", false),
            dense_points: scalar_or(text, "dense_points", false),
            ts_first_point: scalar_or(text, "ts_first_point", false),
            start_angle: scalar_or(text, "start_angle", 0.0f32),
            end_angle: scalar_or(text, "end_angle", 360.0f32),
            wait_for_difop: true,
            frame_id: find_scalar(text, "ros_frame_id").unwrap_or_else(|| "rslidar".into()),
            mirror_y: scalar_or(text, "mirror_y", false),
            pcap_path: find_scalar(text, "pcap_path"),
            pcap_rate: scalar_or(text, "pcap_rate", 1.0f32),
            pcap_repeat: scalar_or(text, "pcap_repeat", false),
        }
    }
}

/// Reads the `costmap:` block of config.yaml (nav2 costmap.yaml key names);
/// anything missing keeps CostmapParams::default().
pub fn parse_costmap_params(text: &str) -> costmap_2d::CostmapParams {
    let d = costmap_2d::CostmapParams::default();
    costmap_2d::CostmapParams {
        width: scalar_or(text, "width", d.width),
        height: scalar_or(text, "height", d.height),
        resolution: scalar_or(text, "resolution", d.resolution),
        origin_x: scalar_or(text, "origin_x", d.origin_x),
        origin_y: scalar_or(text, "origin_y", d.origin_y),
        robot_radius: scalar_or(text, "robot_radius", d.robot_radius),
        track_unknown_space: scalar_or(text, "track_unknown_space", d.track_unknown_space),
        sensor_x: scalar_or(text, "sensor_x", d.sensor_x),
        sensor_y: scalar_or(text, "sensor_y", d.sensor_y),
        sensor_z: scalar_or(text, "sensor_z", d.sensor_z),
        min_obstacle_height: scalar_or(text, "min_obstacle_height", d.min_obstacle_height),
        max_obstacle_height: scalar_or(text, "max_obstacle_height", d.max_obstacle_height),
        obstacle_max_range: scalar_or(text, "obstacle_max_range", d.obstacle_max_range),
        obstacle_min_range: scalar_or(text, "obstacle_min_range", d.obstacle_min_range),
        raytrace_max_range: scalar_or(text, "raytrace_max_range", d.raytrace_max_range),
        raytrace_min_range: scalar_or(text, "raytrace_min_range", d.raytrace_min_range),
        clearing_az_res_deg: scalar_or(text, "clearing_az_res_deg", d.clearing_az_res_deg),
        inflation_radius: scalar_or(text, "inflation_radius", d.inflation_radius),
        cost_scaling_factor: scalar_or(text, "cost_scaling_factor", d.cost_scaling_factor),
    }
}