//! Byte formats published by the driver, all little-endian. Each `*_into`
//! fills a buffer of exactly `*_len` bytes (e.g. a shared-memory frame); the
//! plain `encode_*` wrappers allocate one.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::costmap_2d::Costmap2D;
use crate::rslidar_math::{Header, POINT_STEP};
use crate::segmentation::{RangeGraph, AZ_RES_CENTIDEG, NUM_COLS, NUM_RINGS};
use crate::transform::Transform;

/// XYZI (f32) + cluster/track id (i32).
pub const SEG_POINT_STEP: u32 = 20;
pub const POINTS_HEADER_LEN: usize = 16;
pub const RANGEGRAPH_HEADER_LEN: usize = 32;
/// x, y, z, intensity, range (f32) + cluster/track id (i32).
pub const RANGEGRAPH_RECORD_LEN: usize = 24;
pub const COSTMAP_HEADER_LEN: usize = 32;
pub const POSE_LEN: usize = 40;

fn stamp_into(hdr: &mut [u8], header: &Header) {
    hdr[0..4].copy_from_slice(&header.stamp_sec.to_le_bytes());
    hdr[4..8].copy_from_slice(&header.stamp_nanosec.to_le_bytes());
}

fn alloc(len: usize, fill: impl FnOnce(&mut [u8])) -> Vec<u8> {
    let mut buf = vec![0u8; len];
    fill(&mut buf);
    buf
}

/// stamp_sec i32, stamp_nanosec u32, width u32, point_step u32, then
/// `width * point_step` point bytes.
pub fn points_len(data: &[u8]) -> usize {
    POINTS_HEADER_LEN + data.len()
}

pub fn encode_points_into(buf: &mut [u8], header: &Header, point_step: u32, data: &[u8]) {
    let (hdr, body) = buf.split_at_mut(POINTS_HEADER_LEN);
    stamp_into(hdr, header);
    hdr[8..12].copy_from_slice(&(data.len() as u32 / point_step).to_le_bytes());
    hdr[12..16].copy_from_slice(&point_step.to_le_bytes());
    body.copy_from_slice(data);
}

pub fn encode_points(header: &Header, point_step: u32, data: &[u8]) -> Vec<u8> {
    alloc(points_len(data), |b| encode_points_into(b, header, point_step, data))
}

/// Same header as points, with `point_step = SEG_POINT_STEP`: each XYZI point
/// followed by its id from `point_cluster` (-1 = unclustered).
pub fn segmented_len(point_cluster: &[i32]) -> usize {
    POINTS_HEADER_LEN + point_cluster.len() * SEG_POINT_STEP as usize
}

pub fn encode_segmented_into(buf: &mut [u8], header: &Header, cloud_data: &[u8], point_cluster: &[i32]) {
    let (hdr, body) = buf.split_at_mut(POINTS_HEADER_LEN);
    stamp_into(hdr, header);
    hdr[8..12].copy_from_slice(&(point_cluster.len() as u32).to_le_bytes());
    hdr[12..16].copy_from_slice(&SEG_POINT_STEP.to_le_bytes());
    let step = POINT_STEP as usize;
    for ((out, pt), &cluster) in body
        .chunks_exact_mut(SEG_POINT_STEP as usize)
        .zip(cloud_data.chunks_exact(step))
        .zip(point_cluster)
    {
        out[..step].copy_from_slice(pt);
        out[step..].copy_from_slice(&cluster.to_le_bytes());
    }
}

pub fn encode_segmented(header: &Header, cloud_data: &[u8], point_cluster: &[i32]) -> Vec<u8> {
    alloc(segmented_len(point_cluster), |b| encode_segmented_into(b, header, cloud_data, point_cluster))
}

/// stamp_sec i32, stamp_nanosec u32, rows u32, cols u32, az_res_centideg u32,
/// record_len u32, reserved u64, then `rows * cols` records indexed
/// `ring * cols + col`. Empty cells have range 0.0 and id -1.
pub fn rangegraph_len(graph: &RangeGraph) -> usize {
    RANGEGRAPH_HEADER_LEN + graph.nodes.len() * RANGEGRAPH_RECORD_LEN
}

pub fn encode_rangegraph_into(buf: &mut [u8], header: &Header, graph: &RangeGraph) {
    let (hdr, body) = buf.split_at_mut(RANGEGRAPH_HEADER_LEN);
    stamp_into(hdr, header);
    hdr[8..12].copy_from_slice(&(NUM_RINGS as u32).to_le_bytes());
    hdr[12..16].copy_from_slice(&(NUM_COLS as u32).to_le_bytes());
    hdr[16..20].copy_from_slice(&(AZ_RES_CENTIDEG as u32).to_le_bytes());
    hdr[20..24].copy_from_slice(&(RANGEGRAPH_RECORD_LEN as u32).to_le_bytes());
    hdr[24..32].copy_from_slice(&0u64.to_le_bytes());
    for (rec, n) in body.chunks_exact_mut(RANGEGRAPH_RECORD_LEN).zip(&graph.nodes) {
        let (range, cluster) = if n.valid { (n.range, n.beta) } else { (0.0, -1) };
        for (i, v) in [n.x, n.y, n.z, n.intensity, range].into_iter().enumerate() {
            rec[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        rec[20..24].copy_from_slice(&cluster.to_le_bytes());
    }
}

pub fn encode_rangegraph(header: &Header, graph: &RangeGraph) -> Vec<u8> {
    alloc(rangegraph_len(graph), |b| encode_rangegraph_into(b, header, graph))
}

/// stamp_sec i32, stamp_nanosec u32, size_x u32, size_y u32, resolution f32,
/// origin_x f32, origin_y f32, reserved u32, then `size_x * size_y` row-major
/// u8 costs (0 free .. 253 inscribed, 254 lethal, 255 unknown).
pub fn costmap_len(map: &Costmap2D) -> usize {
    COSTMAP_HEADER_LEN + map.data.len()
}

pub fn encode_costmap_into(buf: &mut [u8], header: &Header, map: &Costmap2D) {
    let (hdr, body) = buf.split_at_mut(COSTMAP_HEADER_LEN);
    stamp_into(hdr, header);
    hdr[8..12].copy_from_slice(&(map.size_x as u32).to_le_bytes());
    hdr[12..16].copy_from_slice(&(map.size_y as u32).to_le_bytes());
    hdr[16..20].copy_from_slice(&map.resolution.to_le_bytes());
    hdr[20..24].copy_from_slice(&map.origin_x.to_le_bytes());
    hdr[24..28].copy_from_slice(&map.origin_y.to_le_bytes());
    hdr[28..32].copy_from_slice(&0u32.to_le_bytes());
    body.copy_from_slice(&map.data);
}

pub fn encode_costmap(header: &Header, map: &Costmap2D) -> Vec<u8> {
    alloc(costmap_len(map), |b| encode_costmap_into(b, header, map))
}

/// stamp_sec i32, stamp_nanosec u32, reserved u32, then tx, ty, tz, qx, qy,
/// qz, qw as f32.
pub fn encode_pose_into(buf: &mut [u8], header: &Header, tf: &Transform) {
    stamp_into(buf, header);
    buf[8..12].copy_from_slice(&0u32.to_le_bytes());
    let q = tf.rotation.quaternion();
    for (i, v) in [tf.translation.x, tf.translation.y, tf.translation.z, q.i, q.j, q.k, q.w].into_iter().enumerate() {
        buf[12 + i * 4..16 + i * 4].copy_from_slice(&v.to_le_bytes());
    }
}

pub fn encode_pose(header: &Header, tf: &Transform) -> Vec<u8> {
    alloc(POSE_LEN, |b| encode_pose_into(b, header, tf))
}

/// Returns (stamp_ns, transform).
pub fn decode_pose(payload: &[u8]) -> Option<(i64, Transform)> {
    if payload.len() < POSE_LEN {
        return None;
    }
    let f = |i: usize| f32::from_le_bytes(payload[i..i + 4].try_into().unwrap());
    let sec = i32::from_le_bytes(payload[0..4].try_into().unwrap()) as i64;
    let nanosec = u32::from_le_bytes(payload[4..8].try_into().unwrap()) as i64;
    Some((sec * 1_000_000_000 + nanosec, Transform {
        translation: Vector3::new(f(12), f(16), f(20)),
        rotation: UnitQuaternion::from_quaternion(Quaternion::new(f(36), f(24), f(28), f(32))),
    }))
}
