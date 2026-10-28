// implements a Rust version of rslidar_sdk_node.cpp + node_manager.cpp + source_driver.hpp
// from Robosense Lidar rslidar_sdk
#![allow(dead_code)]

use crate::segmentation;
use crate::costmap_2d;

pub use crate::config::*;
use crate::rslidar_math::*;
use pcap_file::pcap::PcapReader;

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

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
// actually the range graph structure directly takes in vertical channel angles from 
// UDP packet information, maybe add a way to manually input LiDAR config angles, but 
// not necessary
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
// decoder state machine (Decoder / DecoderMech / DecoderRSHELIOS)
// ---------------------------------------------------------------------------

pub struct RsHeliosDecoder {
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
    range_graph: segmentation::RangeGraph,
    set_graph: segmentation::SetGraph,
    ring_vert_deg: [f32; LASER_NUM],
    seg_params: Arc<Mutex<segmentation::SegParams>>,
    first_point_ts: f64,
    prev_point_ts: f64,

    chan_tss: [f64; 32],
    chan_azis: [f32; 32],
}

impl RsHeliosDecoder {
    pub fn new(cfg: &DriverConfig, seg_params: Arc<Mutex<segmentation::SegParams>>) -> Self {
        Self {
            distance_section: DistanceSection::new(cfg.min_distance, cfg.max_distance),
            // start_angle/end_angle are plain degrees in config.yaml, but
            // AzimuthSection (like the MSOP azimuth field itself) works in
            // hundredths of a degree -- convert or a 0..360 config silently
            // becomes a 0.00..3.60 degree wedge.
            scan_section: AzimuthSection::new(cfg.start_angle * 100.0, cfg.end_angle * 100.0),
            split: SplitStrategyByAngle::new(0), // split_frame_mode=1 (by angle), split_angle=0
            cfg: cfg.clone(),
            angles_ready: false,
            chan_angles: ChanAngles::empty(),
            dual_return: false,
            block_az_diff_const: 20, // matches DecoderMech's initial default
            fov_blind_ts_diff: 0.0,
            points: Vec::new(),
            range_graph: segmentation::RangeGraph::new(),
            set_graph: segmentation::SetGraph::new(),
            ring_vert_deg: [0.0; LASER_NUM],
            seg_params,
            first_point_ts: 0.0,
            prev_point_ts: 0.0,
            chan_tss: chan_tss(),
            chan_azis: chan_azis(),
        }
    }

    pub fn decode_difop(&mut self, pkt: &[u8]) {
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
                // user_chan[chan] is the ring a channel's points get inserted
                // under (see RangeGraph::insert's call site below), so invert
                // that same mapping here to get each ring's real vertical
                // angle in degrees, ready for second_segmentation.
                for chan in 0..LASER_NUM {
                    let ring = self.chan_angles.user_chan[chan] as usize;
                    self.ring_vert_deg[ring] = self.chan_angles.vert[chan] as f32 / 100.0;
                }
            }
        }
    }

    /// Feeds one raw MSOP UDP datagram in; returns every frame (point cloud,
    /// paired with the range graph built alongside it) that was completed
    /// while processing it (usually zero or one).
    pub fn decode_msop(&mut self, raw: &[u8]) -> Vec<(PointCloud2, segmentation::RangeGraph)> {
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

        // Read once per packet (not per point/channel) to avoid locking the
        // mutex 12*32 times over -- these only need to be about as fresh as
        // the other seg_params read once per revolution below.
        let (height_filter_enabled, min_height) = {
            let p = self.seg_params.lock().unwrap();
            // (p.height_filter_enabled, p.min_height)
            (true, -0.6)

        };

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
                // Swap in a fresh graph exactly when `self.points` resets, so
                // the finished one always corresponds to the same revolution
                // as the cloud it's paired with.
                let mut finished_graph =
                    std::mem::replace(&mut self.range_graph, segmentation::RangeGraph::new());
                if !self.points.is_empty() {
                    let (seg_enabled, th_d, th_z_deg, th_v_base, th_v_slope, min_cluster_points, min_gap_m) = {
                        let p = self.seg_params.lock().unwrap();
                        (p.seg_enabled, p.th_d, p.th_z_deg, p.th_v_base, p.th_v_slope, p.min_cluster_points, p.min_gap_m)
                    };
                    if seg_enabled {
                        finished_graph.first_segmentation(&mut self.set_graph, th_d, th_z_deg.to_radians(), min_gap_m);
                        self.set_graph.second_segmentation(&mut finished_graph, th_v_base, th_v_slope, min_cluster_points);
                    }

                    let cloud = build_point_cloud(&self.points, frame_ts, &self.cfg.frame_id, self.cfg.dense_points);
                    out.push((cloud, finished_graph));
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
                // Computed before the filter check (not just inside it, like
                // x/y) since z alone -- not x or y -- is what the height
                // filter below needs to decide inclusion.
                let z = distance * sin_centideg(angle_vert) + RZ;

                if self.distance_section.contains(distance)
                    && self.scan_section.contains(angle_horiz_final)
                    && (!height_filter_enabled || z >= min_height)
                {
                    let cv = cos_centideg(angle_vert);
                    let ch = cos_centideg(angle_horiz_final);
                    let sh = sin_centideg(angle_horiz_final);
                    let ch0 = cos_centideg(angle_horiz);
                    let sh0 = sin_centideg(angle_horiz);

                    let x = distance * cv * ch + RX * ch0;
                    let y = -distance * cv * sh - RX * sh0; // spherical to cartesian
                    let y = if self.cfg.mirror_y { -y } else { y };

                    let ring = self.chan_angles.user_chan[chan] as usize;
                    let point_idx = self.points.len() as i32;
                    self.range_graph.insert(
                        ring,
                        azimuth_round(angle_horiz_final),
                        x, y, z, intensity.into(),
                        distance,
                        point_idx,
                    );
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

pub enum RawPacket {
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

// parses an ethernet frame -> udp payload
fn udp_payload_from_eth_frame(frame: &[u8]) -> Option<(u16, &[u8])> {
    if frame.len() < 14 {
        return None;
    }
    let mut off = 12; // skip destination + source MAC
    let mut ethertype = be_u16(&frame[off..off + 2]);
    off += 2;
    if ethertype == 0x8100 {
        // 802.1Q tag: 2 bytes tag control + 2 bytes real ethertype
        if frame.len() < off + 4 {
            return None;
        }
        ethertype = be_u16(&frame[off + 2..off + 4]);
        off += 4;
    }
    if ethertype != 0x0800 {
        return None; // not IPv4
    }

    let ip = frame.get(off..)?;
    if ip.len() < 20 {
        return None;
    }
    let ihl = (ip[0] & 0x0F) as usize * 4;
    if ihl < 20 || ip.len() < ihl + 8 || ip[9] != 17 {
        return None; // malformed header, or not UDP
    }

    let udp = &ip[ihl..];
    let dst_port = be_u16(&udp[2..4]);
    Some((dst_port, &udp[8..]))
}

// replays a pcap capture
fn spawn_pcap_reader(
    path: String,
    msop_port: u16,
    difop_port: u16,
    rate: f32,
    repeat: bool,
    tx: mpsc::Sender<RawPacket>,
) {
    thread::spawn(move || {
        let rate = if rate > 0.0 { rate } else { 1.0 };
        loop {
            let file = match std::fs::File::open(&path) {
                Ok(f) => f,
                Err(e) => {
                    eprintln!("rslidar: failed to open pcap {path}: {e}");
                    return;
                }
            };
            let mut reader = match PcapReader::new(file) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("rslidar: failed to parse pcap {path}: {e}");
                    return;
                }
            };

            let playback_start = Instant::now();
            let mut pcap_t0: Option<Duration> = None;
            let mut sent = 0u64;

            while let Some(pkt) = reader.next_packet() {
                let pkt = match pkt {
                    Ok(p) => p,
                    Err(e) => {
                        eprintln!("rslidar: pcap read error: {e}");
                        break;
                    }
                };

                let t0 = *pcap_t0.get_or_insert(pkt.timestamp);
                let target = pkt.timestamp.saturating_sub(t0).div_f32(rate);
                let elapsed = playback_start.elapsed();
                if target > elapsed {
                    thread::sleep(target - elapsed);
                }

                let Some((dst_port, payload)) = udp_payload_from_eth_frame(&pkt.data) else {
                    continue;
                };
                let raw = if dst_port == msop_port {
                    RawPacket::Msop(payload.to_vec())
                } else if dst_port == difop_port {
                    RawPacket::Difop(payload.to_vec())
                } else {
                    continue;
                };
                if tx.send(raw).is_err() {
                    return; // receiver gone, shut this thread down
                }
                sent += 1;
            }

            println!(
                "rslidar: pcap playback finished ({sent} lidar packets from {path}); \
                 leaving the last decoded frame on screen"
            );
            if !repeat {
                return;
            }
        }
    });
}

/// Sets up the MSOP/DIFOP packet source described by `cfg`: either live UDP
/// sockets, or offline pcap replay when `cfg.pcap_path` is set.
pub fn spawn_packet_source(cfg: &DriverConfig, tx: mpsc::Sender<RawPacket>) {
    if let Some(path) = &cfg.pcap_path {
        println!(
            "rslidar: replaying pcap {path} (rate={}, repeat={})",
            cfg.pcap_rate, cfg.pcap_repeat
        );
        spawn_pcap_reader(path.clone(), cfg.msop_port, cfg.difop_port, cfg.pcap_rate, cfg.pcap_repeat, tx);
        return;
    }

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

    spawn_udp_reader(msop_sock, RawPacket::Msop, tx.clone());
    spawn_udp_reader(difop_sock, RawPacket::Difop, tx);
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

pub fn default_config_path() -> String {
    let mut p = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("src/rslidar/config/config.yaml");
    p.to_string_lossy().to_string()
}
