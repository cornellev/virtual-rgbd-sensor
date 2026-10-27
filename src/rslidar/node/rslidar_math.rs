#![allow(dead_code)]

// ---------------------------------------------------------------------------
// PointCloud2-shaped output (no ROS/ROS2 types involved)
// ---------------------------------------------------------------------------

pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

pub struct Header {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub frame_id: String,
}

pub struct PointField {
    pub name: &'static str,
    pub offset: u32,
    pub datatype: u8, // sensor_msgs/PointField: FLOAT32 = 7
    pub count: u32,
}

/// Field-for-field equivalent of sensor_msgs::msg::PointCloud2, with the
/// same wire layout (little-endian data, XYZI point type — the default
/// POINT_TYPE=XYZI build of rslidar_sdk, see rslidar_sdk/CMakeLists.txt) but
/// without depending on rclrs/sensor_msgs at all.
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

pub const POINT_STEP: u32 = 16; // x,y,z,intensity as f32

pub fn build_point_cloud(points: &[Point], ts: f64, frame_id: &str, dense: bool) -> PointCloud2 {
    let width = points.len() as u32;
    let mut data = Vec::with_capacity(points.len() * POINT_STEP as usize);

    for p in points {
        data.extend_from_slice(&p.x.to_le_bytes());
        data.extend_from_slice(&p.y.to_le_bytes());
        data.extend_from_slice(&p.z.to_le_bytes());
        data.extend_from_slice(&p.intensity.to_le_bytes());
    }

    let stamp_sec = ts.floor() as i32;
    let stamp_nanosec = ((ts - stamp_sec as f64) * 1e9).round() as u32;

    PointCloud2 {
        header: Header { stamp_sec, stamp_nanosec, frame_id: frame_id.to_string() },
        height: 1,
        width,
        fields: vec![
            PointField { name: "x", offset: 0, datatype: 7, count: 1 },
            PointField { name: "y", offset: 4, datatype: 7, count: 1 },
            PointField { name: "z", offset: 8, datatype: 7, count: 1 },
            PointField { name: "intensity", offset: 12, datatype: 7, count: 1 },
        ],
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * width,
        data,
        is_dense: dense,
    }
}

pub fn be_u16(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

pub fn be_u32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

pub fn be_u48(b: &[u8]) -> u64 {
    let mut v: u64 = 0;
    for &byte in &b[0..6] {
        v = (v << 8) | byte as u64;
    }
    v
}

pub fn cos_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().cos() as f32
}

pub fn sin_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().sin() as f32
}

pub fn azimuth_round(v: i32) -> i32 {
    ((v % 36000) + 36000) % 36000
}