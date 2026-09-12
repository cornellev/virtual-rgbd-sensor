// implements a Rust version of rslidar_sdk_node.cpp + node_manager.cpp + source_driver.hpp
// from Robosense Lidar rslidar_sdk
#![allow(dead_code)]

// evil
use rclrs;
use builtin_interfaces::msg::Time;
use sensor_msgs::msg::{
    PointCloud2 as RosPointCloud2,
    PointField as RosPointField,
};
use std_msgs::msg::Header as RosHeader;
//

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// RSHELIOS protocol constants (from decoder_RSHELIOS.hpp / decoder_mech.hpp)
// Rn this will only work for our current LiDAR, so like for Ithaca365, need new
// config

const LASER_NUM: usize = 32;
const BLOCKS_PER_PKT: usize = 12;
const MSOP_LEN: usize = 1248;
const DIFOP_LEN: usize = 1248;

const MSOP_ID: [u8; 4] = [0x55, 0xAA, 0x05, 0x5A];
const DIFOP_ID: [u8; 8] = [0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55];
const BLOCK_ID: [u8; 2] = [0xFF, 0xEE];

const MSOP_HEADER_LEN: usize = 42;
const MSOP_BLOCK_LEN: usize = 100; // 2(id) + 2(azimuth) + 32 * 3(channel)

const DISTANCE_RES: f32 = 0.0025;
const DISTANCE_MIN_DEFAULT: f32 = 0.1;
const DISTANCE_MAX_DEFAULT: f32 = 180.0;

// lens center offset (RX/RY/RZ), RY is unused by the mechanical point formula
const RX: f32 = 0.03498;
const RZ: f32 = 0.0;

const BLK_TS_US: f64 = 55.56; // firing duration of one block, in microseconds
const BLOCK_DURATION: f64 = BLK_TS_US / 1_000_000.0; // seconds
const PACKET_DURATION_US: f64 = BLK_TS_US * BLOCKS_PER_PKT as f64;

// per-channel firing offset within a block, in microseconds
const FIRING_TSS_US: [f64; 32] = [
    0.00, 1.73, 3.46, 5.19, 6.92, 8.65, 10.38, 12.11, 13.84, 15.57, 17.3, 19.03, 20.76, 22.49,
    24.22, 25.95, 27.68, 29.41, 31.14, 32.87, 34.6, 36.33, 38.06, 39.79, 41.52, 43.25, 44.98,
    46.71, 48.44, 50.17, 51.9, 53.63,
];

fn chan_tss() -> [f64; 32] {
    let mut out = [0.0f64; 32];
    for i in 0..32 {
        out[i] = FIRING_TSS_US[i] / 1_000_000.0;
    }
    out
}

fn chan_azis() -> [f32; 32] {
    let mut out = [0.0f32; 32];
    for i in 0..32 {
        out[i] = (FIRING_TSS_US[i] / BLK_TS_US) as f32;
    }
    out
}

// DIFOP packet field offsets
const DIFOP_RPM: usize = 8;
const DIFOP_FOV_START: usize = 32;
const DIFOP_FOV_END: usize = 34;
const DIFOP_RETURN_MODE: usize = 300;
const DIFOP_VERT_ANGLE_CALI: usize = 468;
const DIFOP_HORIZ_ANGLE_CALI: usize = 564;

fn be_u16(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

fn be_u32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

fn be_u48(b: &[u8]) -> u64 {
    let mut v: u64 = 0;
    for &byte in &b[0..6] {
        v = (v << 8) | byte as u64;
    }
    v
}

fn cos_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().cos() as f32
}

fn sin_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().sin() as f32
}

fn azimuth_round(v: i32) -> i32 {
    ((v % 36000) + 36000) % 36000
}

// ---------------------------------------------------------------------------
// config.yaml loading (only the scalar keys this script needs)
// ---------------------------------------------------------------------------

/// Finds a `<key>: <value>` line anywhere in the file and returns the value
/// with any trailing `# comment` stripped. Good enough for config.yaml's flat
/// key layout; this is not a general YAML parser.
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
struct DriverConfig {
    host_address: String,
    lidar_type: String,
    msop_port: u16,
    difop_port: u16,
    user_layer_bytes: u16,
    tail_layer_bytes: u16,
    min_distance: f32,
    max_distance: f32,
    use_lidar_clock: bool,
    dense_points: bool,
    ts_first_point: bool,
    start_angle: f32,
    end_angle: f32,
    wait_for_difop: bool,
    frame_id: String,
}

impl DriverConfig {
    fn parse(text: &str) -> Self {
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
        }
    }
}

// ---------------------------------------------------------------------------
// channel calibration (ChanAngles in chan_angles.hpp)
// ---------------------------------------------------------------------------

struct ChanAngles {
    vert: Vec<i32>,   // hundredths of a degree, signed
    horiz: Vec<i32>,  // hundredths of a degree, signed
    user_chan: Vec<u16>, // channel index reordered by ascending vertical angle
}

impl ChanAngles {
    fn empty() -> Self {
        Self { vert: Vec::new(), horiz: Vec::new(), user_chan: Vec::new() }
    }

    /// Mirrors ChanAngles::loadFromDifop: each calibration entry is a sign
    /// byte followed by a big-endian magnitude; sign == 0xFF means "not
    /// calibrated yet", and values must fall in [-90.00, 180.00) degrees.
    fn from_difop(vert_bytes: &[u8], horiz_bytes: &[u8]) -> Option<Self> {
        let angle_ok = |v: i32| (-9000..18000).contains(&v);

        let mut vert = Vec::with_capacity(LASER_NUM);
        let mut horiz = Vec::with_capacity(LASER_NUM);
        for i in 0..LASER_NUM {
            let vsign = vert_bytes[i * 3];
            if vsign == 0xFF {
                return None;
            }
            let vmag = be_u16(&vert_bytes[i * 3 + 1..i * 3 + 3]) as i32;
            let v = if vsign != 0 { -vmag } else { vmag };
            if !angle_ok(v) {
                return None;
            }
            vert.push(v);

            let hsign = horiz_bytes[i * 3];
            let hmag = be_u16(&horiz_bytes[i * 3 + 1..i * 3 + 3]) as i32;
            let h = if hsign != 0 { -hmag } else { hmag };
            if !angle_ok(h) {
                return None;
            }
            horiz.push(h);
        }

        let mut user_chan = vec![0u16; LASER_NUM];
        for i in 0..LASER_NUM {
            user_chan[i] = vert.iter().filter(|&&v| v < vert[i]).count() as u16;
        }

        Some(Self { vert, horiz, user_chan })
    }
}

// ---------------------------------------------------------------------------
// azimuth/distance validity sections (section.hpp)
// ---------------------------------------------------------------------------

struct AzimuthSection {
    full_round: bool,
    start: i32,
    end: i32,
    cross_zero: bool,
}

impl AzimuthSection {
    fn new(start: f32, end: f32) -> Self {
        let start = start as i32;
        let end = end as i32;
        let full_round = azimuth_round(end - start) == 0;
        let s = azimuth_round(start);
        let e = azimuth_round(end);
        Self { full_round, start: s, end: e, cross_zero: s > e }
    }

    fn contains(&self, angle: i32) -> bool {
        if self.full_round {
            return true;
        }
        if self.cross_zero {
            angle >= self.start || angle < self.end
        } else {
            angle >= self.start && angle < self.end
        }
    }
}

struct DistanceSection {
    min: f32,
    max: f32,
}

impl DistanceSection {
    fn new(user_min: f32, user_max: f32) -> Self {
        let user_min = user_min.max(0.0);
        let user_max = user_max.max(0.0);
        if user_min != 0.0 || user_max != 0.0 {
            Self { min: user_min, max: user_max }
        } else {
            Self { min: DISTANCE_MIN_DEFAULT, max: DISTANCE_MAX_DEFAULT }
        }
    }

    fn contains(&self, d: f32) -> bool {
        d >= self.min && d <= self.max
    }
}

// ---------------------------------------------------------------------------
// frame splitting (SplitStrategyByAngle in split_strategy.hpp)
// ---------------------------------------------------------------------------

struct SplitStrategyByAngle {
    split_angle: i32,
    prev_angle: i32,
}

impl SplitStrategyByAngle {
    fn new(split_angle: i32) -> Self {
        Self { split_angle, prev_angle: split_angle }
    }

    fn new_block(&mut self, angle: i32) -> bool {
        if angle < self.prev_angle {
            self.prev_angle -= 36000;
        }
        let split = self.prev_angle < self.split_angle && self.split_angle <= angle;
        self.prev_angle = angle;
        split
    }
}

/// Per-block azimuth step and timestamp offset, mirroring
/// SingleReturnBlockIterator / DualReturnBlockIterator in block_iterator.hpp.
/// `dual` selects the block step (2 for dual-return LiDARs, whose blocks come
/// in same-azimuth pairs; 1 otherwise).
fn compute_block_iter(
    azimuths: &[i32; BLOCKS_PER_PKT],
    block_duration: f64,
    block_az_duration: i32,
    fov_blind_duration: f64,
    dual: bool,
) -> ([i32; BLOCKS_PER_PKT], [f64; BLOCKS_PER_PKT]) {
    let mut az_diffs = [0i32; BLOCKS_PER_PKT];
    let mut tss = [0f64; BLOCKS_PER_PKT];
    let step = if dual { 2 } else { 1 };

    let mut acc = 0f64;
    let mut blk = 0usize;
    while blk < BLOCKS_PER_PKT - step {
        let mut ts_diff = block_duration;
        let mut az_diff = azimuths[blk + step] - azimuths[blk];
        if az_diff < 0 {
            az_diff += 36000;
        }
        if az_diff > 100 {
            // crossed the FOV blind zone
            az_diff = block_az_duration;
            ts_diff = fov_blind_duration;
        }
        for s in 0..step {
            az_diffs[blk + s] = az_diff;
            tss[blk + s] = acc;
        }
        acc += ts_diff;
        blk += step;
    }
    for s in 0..step {
        az_diffs[blk + s] = block_az_duration;
        tss[blk + s] = acc;
    }

    (az_diffs, tss)
}

// ---------------------------------------------------------------------------
// PointCloud2-shaped output (no ROS/ROS2 types involved)
// ---------------------------------------------------------------------------

struct Point {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
}

struct Header {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
}

struct PointField {
    name: &'static str,
    offset: u32,
    datatype: u8, // sensor_msgs/PointField: FLOAT32 = 7
    count: u32,
}

/// Field-for-field equivalent of sensor_msgs::msg::PointCloud2, with the
/// same wire layout (little-endian data, XYZI point type — the default
/// POINT_TYPE=XYZI build of rslidar_sdk, see rslidar_sdk/CMakeLists.txt) but
/// without depending on rclrs/sensor_msgs at all.
struct PointCloud2 {
    header: Header,
    height: u32,
    width: u32,
    fields: Vec<PointField>,
    is_bigendian: bool,
    point_step: u32,
    row_step: u32,
    data: Vec<u8>,
    is_dense: bool,
}

const POINT_STEP: u32 = 16; // x,y,z,intensity as f32

fn build_point_cloud(points: &[Point], ts: f64, frame_id: &str, dense: bool) -> PointCloud2 {
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

//evil
fn to_ros_point_cloud(cloud: &PointCloud2) -> RosPointCloud2 {
    RosPointCloud2 {
        header: RosHeader {
            stamp: Time {
                sec: cloud.header.stamp_sec,
                nanosec: cloud.header.stamp_nanosec,
            },
            frame_id: cloud.header.frame_id.clone(),
        },
        height: cloud.height,
        width: cloud.width,
        fields: cloud.fields.iter().map(|f| {
            RosPointField {
                name: f.name.to_string(),
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            }
        }).collect(),
        is_bigendian: cloud.is_bigendian,
        point_step: cloud.point_step,
        row_step: cloud.row_step,
        data: cloud.data.clone(),
        is_dense: cloud.is_dense,
    }
}
//

// ---------------------------------------------------------------------------
// decoder state machine (Decoder / DecoderMech / DecoderRSHELIOS)
// ---------------------------------------------------------------------------

struct RsHeliosDecoder {
    cfg: DriverConfig,

    angles_ready: bool,
    chan_angles: ChanAngles,
    dual_return: bool,
    block_az_diff_const: i32, // fallback/default step between blocks, from DIFOP rpm
    fov_blind_ts_diff: f64,

    distance_section: DistanceSection,
    scan_section: AzimuthSection,
    split: SplitStrategyByAngle,

    points: Vec<Point>,
    first_point_ts: f64,
    prev_point_ts: f64,

    chan_tss: [f64; 32],
    chan_azis: [f32; 32],
}

impl RsHeliosDecoder {
    fn new(cfg: &DriverConfig) -> Self {
        Self {
            distance_section: DistanceSection::new(cfg.min_distance, cfg.max_distance),
            scan_section: AzimuthSection::new(cfg.start_angle, cfg.end_angle),
            split: SplitStrategyByAngle::new(0), // split_frame_mode=1 (by angle), split_angle=0
            cfg: cfg.clone(),
            angles_ready: false,
            chan_angles: ChanAngles::empty(),
            dual_return: false,
            block_az_diff_const: 20, // matches DecoderMech's initial default
            fov_blind_ts_diff: 0.0,
            points: Vec::new(),
            first_point_ts: 0.0,
            prev_point_ts: 0.0,
            chan_tss: chan_tss(),
            chan_azis: chan_azis(),
        }
    }

    fn decode_difop(&mut self, pkt: &[u8]) {
        if pkt.len() != DIFOP_LEN || &pkt[0..8] != &DIFOP_ID {
            return;
        }

        let rpm = be_u16(&pkt[DIFOP_RPM..DIFOP_RPM + 2]) as u32;
        let mut rps = rpm / 60;
        if rps == 0 {
            rps = 10; // LiDAR RPM is 0, default to 600rpm like the reference decoder
        }
        self.block_az_diff_const =
            (36000.0 * rps as f64 * BLOCK_DURATION).round() as i32;

        let fov_start = be_u16(&pkt[DIFOP_FOV_START..DIFOP_FOV_START + 2]) as i32;
        let fov_end = be_u16(&pkt[DIFOP_FOV_END..DIFOP_FOV_END + 2]) as i32;
        let fov_range = if fov_start < fov_end {
            fov_end - fov_start
        } else {
            fov_end + 36000 - fov_start
        };
        let fov_blind_range = 36000 - fov_range;
        self.fov_blind_ts_diff = fov_blind_range as f64 / (36000.0 * rps as f64);

        let return_mode = pkt[DIFOP_RETURN_MODE];
        self.dual_return = return_mode == 0x00;

        if !self.angles_ready {
            if let Some(ca) = ChanAngles::from_difop(
                &pkt[DIFOP_VERT_ANGLE_CALI..DIFOP_VERT_ANGLE_CALI + LASER_NUM * 3],
                &pkt[DIFOP_HORIZ_ANGLE_CALI..DIFOP_HORIZ_ANGLE_CALI + LASER_NUM * 3],
            ) {
                self.chan_angles = ca;
                self.angles_ready = true;
            }
        }
    }

    /// Feeds one raw MSOP UDP datagram in; returns every frame (point cloud)
    /// that was completed while processing it (usually zero or one).
    fn decode_msop(&mut self, raw: &[u8]) -> Vec<PointCloud2> {
        let mut out = Vec::new();

        let off = self.cfg.user_layer_bytes as usize;
        let tail = self.cfg.tail_layer_bytes as usize;
        if raw.len() < off + tail {
            return out;
        }
        let pkt = &raw[off..raw.len() - tail];
        if pkt.len() != MSOP_LEN || &pkt[0..4] != &MSOP_ID {
            return out;
        }
        if self.cfg.wait_for_difop && !self.angles_ready {
            return out; // no calibration yet, drop packet like the reference decoder does
        }

        let pkt_ts_us: f64 = if self.cfg.use_lidar_clock {
            let sec = be_u48(&pkt[20..26]) as f64;
            let us = be_u32(&pkt[26..30]) as f64;
            sec * 1_000_000.0 + us
        } else {
            let host_us = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_micros() as f64;
            host_us - PACKET_DURATION_US
        };
        let pkt_ts = pkt_ts_us * 1e-6;

        let mut azimuths = [0i32; BLOCKS_PER_PKT];
        for blk in 0..BLOCKS_PER_PKT {
            let base = MSOP_HEADER_LEN + blk * MSOP_BLOCK_LEN;
            azimuths[blk] = be_u16(&pkt[base + 2..base + 4]) as i32;
        }
        let (az_diffs, tss) = compute_block_iter(
            &azimuths,
            BLOCK_DURATION,
            self.block_az_diff_const,
            self.fov_blind_ts_diff,
            self.dual_return,
        );

        for blk in 0..BLOCKS_PER_PKT {
            let base = MSOP_HEADER_LEN + blk * MSOP_BLOCK_LEN;
            if &pkt[base..base + 2] != &BLOCK_ID {
                eprintln!("rslidar: bad block id in MSOP packet, dropping rest of packet");
                break;
            }

            let block_az = azimuths[blk];
            let block_ts = pkt_ts + tss[blk];
            let block_az_diff = az_diffs[blk];

            if self.split.new_block(block_az) {
                let frame_ts = if self.cfg.ts_first_point { self.first_point_ts } else { self.prev_point_ts };
                if !self.points.is_empty() {
                    let cloud = build_point_cloud(&self.points, frame_ts, &self.cfg.frame_id, self.cfg.dense_points);
                    out.push(cloud);
                }
                self.points.clear();
                self.first_point_ts = block_ts;
            }

            for chan in 0..LASER_NUM {
                let cbase = base + 4 + chan * 3;
                let dist_raw = be_u16(&pkt[cbase..cbase + 2]);
                let intensity = pkt[cbase + 2];

                let chan_ts = block_ts + self.chan_tss[chan];
                let angle_horiz =
                    block_az + (block_az_diff as f32 * self.chan_azis[chan]) as i32;
                let angle_vert = self.chan_angles.vert[chan];
                let angle_horiz_final = angle_horiz + self.chan_angles.horiz[chan];
                let distance = dist_raw as f32 * DISTANCE_RES;

                if self.distance_section.contains(distance) && self.scan_section.contains(angle_horiz_final) {
                    let cv = cos_centideg(angle_vert);
                    let sv = sin_centideg(angle_vert);
                    let ch = cos_centideg(angle_horiz_final);
                    let sh = sin_centideg(angle_horiz_final);
                    let ch0 = cos_centideg(angle_horiz);
                    let sh0 = sin_centideg(angle_horiz);

                    let x = distance * cv * ch + RX * ch0;
                    let y = -distance * cv * sh - RX * sh0;
                    let z = distance * sv + RZ;

                    self.points.push(Point { x, y, z, intensity: intensity as f32 });
                } else if !self.cfg.dense_points {
                    self.points.push(Point { x: f32::NAN, y: f32::NAN, z: f32::NAN, intensity: 0.0 });
                }

                self.prev_point_ts = chan_ts;
            }
        }

        out
    }
}

// ---------------------------------------------------------------------------
// networking + main
// ---------------------------------------------------------------------------

enum RawPacket {
    Msop(Vec<u8>),
    Difop(Vec<u8>),
}

fn spawn_udp_reader(sock: UdpSocket, tag: fn(Vec<u8>) -> RawPacket, tx: mpsc::Sender<RawPacket>) {
    thread::spawn(move || {
        let mut buf = [0u8; 2048];
        loop {
            match sock.recv_from(&mut buf) {
                Ok((n, _src)) => {
                    if tx.send(tag(buf[..n].to_vec())).is_err() {
                        return; // receiver gone, shut this thread down
                    }
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {
                    continue;
                }
                Err(e) => {
                    eprintln!("rslidar: udp recv error: {e}");
                    continue;
                }
            }
        }
    });
}

fn print_frame_stats(idx: u64, cloud: &PointCloud2) {
    let (mut minx, mut miny, mut minz) = (f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let (mut maxx, mut maxy, mut maxz) = (f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    let mut valid = 0usize;

    for chunk in cloud.data.chunks_exact(cloud.point_step as usize) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
        if x.is_nan() || y.is_nan() || z.is_nan() {
            continue;
        }
        valid += 1;
        minx = minx.min(x);
        maxx = maxx.max(x);
        miny = miny.min(y);
        maxy = maxy.max(y);
        minz = minz.min(z);
        maxz = maxz.max(z);
    }

    let ts = cloud.header.stamp_sec as f64 + cloud.header.stamp_nanosec as f64 * 1e-9;
    if valid == 0 {
        println!("frame {idx:>6} | ts={ts:.6} | points={:>5} (0 valid)", cloud.width);
    } else {
        println!(
            "frame {idx:>6} | ts={ts:.6} | points={:>5} (valid {valid:>5}) | bbox x[{minx:.2},{maxx:.2}] y[{miny:.2},{maxy:.2}] z[{minz:.2},{maxz:.2}]",
            cloud.width
        );
    }
}

fn default_config_path() -> String {
    let mut p = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("src/rslidar/config/config.yaml");
    p.to_string_lossy().to_string()
}

fn main() {
    let config_path = std::env::args().nth(1).unwrap_or_else(default_config_path);

    let text = match std::fs::read_to_string(&config_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("rslidar: failed to read config {config_path}: {e}");
            std::process::exit(1)
        }
    };
    let cfg = DriverConfig::parse(&text);

    //evil
    let context = rclrs::Context::new(std::env::args())
        .expect("failed to create ROS2 context");

    let node = rclrs::create_node(&context, "rslidar_decoder")
        .expect("failed to create ROS2 node");

    let publisher = node
        .create_publisher::<RosPointCloud2>(
            "/rslidar_points",
            rclrs::QOS_PROFILE_SENSOR_DATA,
        )
        .expect("failed to create PointCloud2 publisher");
    //

    if cfg.lidar_type != "RSHELIOS" {
        eprintln!(
            "rslidar: warning: config.yaml selects lidar_type={}, but this script only \
             implements the RSHELIOS MSOP/DIFOP layout. Decoding will likely fail.",
            cfg.lidar_type
        );
    }

    println!("rslidar: config loaded from {config_path}");
    println!(
        "rslidar: host_address={} msop_port={} difop_port={} lidar_type={}",
        cfg.host_address, cfg.msop_port, cfg.difop_port, cfg.lidar_type
    );

    let msop_sock = UdpSocket::bind((cfg.host_address.as_str(), cfg.msop_port))
        .unwrap_or_else(|e| {
            eprintln!("rslidar: failed to bind msop port {}: {e}", cfg.msop_port);
            std::process::exit(1)
        });
    let difop_sock = UdpSocket::bind((cfg.host_address.as_str(), cfg.difop_port))
        .unwrap_or_else(|e| {
            eprintln!("rslidar: failed to bind difop port {}: {e}", cfg.difop_port);
            std::process::exit(1)
        });
    msop_sock.set_read_timeout(Some(Duration::from_secs(1))).ok();
    difop_sock.set_read_timeout(Some(Duration::from_secs(1))).ok();

    let (tx, rx) = mpsc::channel::<RawPacket>();
    spawn_udp_reader(msop_sock, RawPacket::Msop, tx.clone());
    spawn_udp_reader(difop_sock, RawPacket::Difop, tx);

    let mut decoder = RsHeliosDecoder::new(&cfg);
    let mut frame_count = 0u64;

    println!("rslidar: waiting for DIFOP/MSOP packets from the LiDAR...");
    for msg in rx {
        match msg {
            RawPacket::Difop(buf) => decoder.decode_difop(&buf),
            RawPacket::Msop(buf) => {
                for cloud in decoder.decode_msop(&buf) {
                    frame_count += 1;
                    print_frame_stats(frame_count, &cloud);

                    //evil
                    let ros_cloud = to_ros_point_cloud(&cloud);

                    match publisher.publish(ros_cloud) {
                        Ok(() => println!(" published {frame_count}")),
                        Err(e) => eprintln!("rslidar: failed to publish point cloud: {e}"),
                    }
                    //
                }
            }
        }
    }
}
