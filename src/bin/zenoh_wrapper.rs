#![allow(dead_code)]

use zenoh::{
    Wait,
    pubsub::Publisher,
    qos::CongestionControl,
    // add shared memory import here, the goal is that instead of publisher we do SHM
    // only one version of rangegraph & costmap is shared across anybody who wants it
    shm::{PosixShmProviderBackend, ShmProvider},
};

use nalgebra::{Quaternion as NQuaternion, UnitQuaternion as NUnitQuaternion, Vector3 as NVector3};

type RslidarShmProvider = ShmProvider<PosixShmProviderBackend>;
const SHM_POOL_SIZE: usize = 64 * 1024 * 1024;
const POSE_KEY: &str = "rslidar/pose_world";
const POSE_BUFFER_LEN: usize = 1024;

use anyhow::Result;
use virtual_rgbd_sensor::costmap_2d;
use virtual_rgbd_sensor::segmentation;
use virtual_rgbd_sensor::visualize::*;
use virtual_rgbd_sensor::tracking::{aggregate_clusters, relabel, Tracker};
use virtual_rgbd_sensor::rslidar_math::*;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    DriverConfig, RawPacket, RsHeliosDecoder, default_config_path,
    parse_costmap_params, spawn_packet_source,
};
use virtual_rgbd_sensor::transform::{interpolate_pose, PoseBuffer, Transform};

use std::collections::VecDeque;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Instant;

const SEG_POINT_STEP: u32 = 20; // x, y, z, intensity (f32) + cluster_id (i32)

// zenoh stuff
fn publisher<'a>(session: &'a zenoh::Session, key: &'static str) -> Result<Publisher<'a>> {
    session
        .declare_publisher(key)
        .congestion_control(CongestionControl::Drop)
        .wait()
        .map_err(|error| anyhow::anyhow!("declare {key} publisher: {error}"))
}


// encode points as bits: header: u16 + 
// stamp_sec: i32 + stamp_nanosec: u32 + width: u32 + point_step: u32 + 
// data: width * point_step`
fn encode_points(header: &Header, point_step: u32, data: &[u8]) -> Vec<u8> {
    let width = data.len() as u32 / point_step;
    let mut buf = Vec::with_capacity(16 + data.len());
    buf.extend_from_slice(&header.stamp_sec.to_le_bytes());
    buf.extend_from_slice(&header.stamp_nanosec.to_le_bytes());
    buf.extend_from_slice(&width.to_le_bytes());
    buf.extend_from_slice(&point_step.to_le_bytes());
    buf.extend_from_slice(data);
    buf
}

const RANGEGRAPH_RECORD_LEN: usize = 24; // x, y, z, intensity, range (f32) + cluster_id (i32)

// encode rangegraph as bits: header: u32 + stamp_sec: i32 + stamp_nanosec: u32 + rows: u32
// + cols: u32 + az_res_centideg: u32 + record_len: u32 + reserved: u64 + 
/// records: rows * cols (indexed by ring * cols + col)
fn encode_rangegraph(header: &Header, graph: &segmentation::RangeGraph) -> Vec<u8> {
    let mut buf = Vec::with_capacity(32 + graph.nodes.len() * RANGEGRAPH_RECORD_LEN);
    buf.extend_from_slice(&header.stamp_sec.to_le_bytes());
    buf.extend_from_slice(&header.stamp_nanosec.to_le_bytes());
    buf.extend_from_slice(&(segmentation::NUM_RINGS as u32).to_le_bytes());
    buf.extend_from_slice(&(segmentation::NUM_COLS as u32).to_le_bytes());
    buf.extend_from_slice(&(segmentation::AZ_RES_CENTIDEG as u32).to_le_bytes());
    buf.extend_from_slice(&(RANGEGRAPH_RECORD_LEN as u32).to_le_bytes());
    buf.extend_from_slice(&0u64.to_le_bytes());
    for n in &graph.nodes {
        let (range, cluster) = if n.valid { (n.range, n.beta) } else { (0.0, -1) };
        for v in [n.x, n.y, n.z, n.intensity, range] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        buf.extend_from_slice(&cluster.to_le_bytes());
    }
    buf
}

// encode costmap as bits: header: u2 + stamp_sec: i32 + stamp_nanosec: u32 + size_x: u32
// + size_y: u32 + resolution: f32 + origin_x: f32 + origin_y: f32 + reserved: u32 +
// costs: size_x * size_y * u8 (0 free, 254 lethal, 255 unknown)
fn encode_costmap(header: &Header, map: &costmap_2d::Costmap2D) -> Vec<u8> {
    let mut buf = Vec::with_capacity(32 + map.data.len());
    buf.extend_from_slice(&header.stamp_sec.to_le_bytes());
    buf.extend_from_slice(&header.stamp_nanosec.to_le_bytes());
    buf.extend_from_slice(&(map.size_x as u32).to_le_bytes());
    buf.extend_from_slice(&(map.size_y as u32).to_le_bytes());
    buf.extend_from_slice(&map.resolution.to_le_bytes());
    buf.extend_from_slice(&map.origin_x.to_le_bytes());
    buf.extend_from_slice(&map.origin_y.to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes());
    buf.extend_from_slice(&map.data);
    buf
}

/// `cloud_data`'s XYZI points (see `POINT_STEP`) with each point's cluster id
/// from `point_cluster` appended as a trailing little-endian i32 (-1 for
/// points that never won a range-graph cell this revolution).
fn build_segmented_data(cloud_data: &[u8], point_cluster: &[i32]) -> Vec<u8> {
    let mut data = Vec::with_capacity(cloud_data.len() + point_cluster.len() * 4);
    for (chunk, &cluster) in cloud_data.chunks_exact(POINT_STEP as usize).zip(point_cluster) {
        data.extend_from_slice(chunk);
        data.extend_from_slice(&cluster.to_le_bytes());
    }
    data
}

fn decode_pose(payload: &[u8]) -> Option<(i64, Transform)> {
    if payload.len() < 40 {
        return None;
    }
    let f32_at = |i: usize| f32::from_le_bytes(payload[i..i + 4].try_into().unwrap());
    let sec = i32::from_le_bytes(payload[0..4].try_into().unwrap()) as i64;
    let nanosec = u32::from_le_bytes(payload[4..8].try_into().unwrap()) as i64;
    Some((sec * 1_000_000_000 + nanosec, Transform {
        translation: NVector3::new(f32_at(12), f32_at(16), f32_at(20)),
        rotation: NUnitQuaternion::from_quaternion(NQuaternion::new(
            f32_at(36), f32_at(24), f32_at(28), f32_at(32),
        )),
    }))
}

fn encode_pose(header: &Header, tf: &Transform) -> Vec<u8> {
    let q = tf.rotation.quaternion();
    let mut buf = Vec::with_capacity(40);
    buf.extend_from_slice(&header.stamp_sec.to_le_bytes());
    buf.extend_from_slice(&header.stamp_nanosec.to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes());
    for v in [tf.translation.x, tf.translation.y, tf.translation.z, q.i, q.j, q.k, q.w] {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    buf
}

fn to_world(tf: &Transform, cloud_data: &mut [u8], graph: &mut segmentation::RangeGraph) {
    for p in cloud_data.chunks_exact_mut(POINT_STEP as usize) {
        let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
        let (x, y, z) = (f(0), f(4), f(8));
        if x.is_nan() {
            continue;
        }
        let w = tf.apply(NVector3::new(x, y, z));
        p[0..4].copy_from_slice(&w.x.to_le_bytes());
        p[4..8].copy_from_slice(&w.y.to_le_bytes());
        p[8..12].copy_from_slice(&w.z.to_le_bytes());
    }
    for n in graph.nodes.iter_mut().filter(|n| n.valid) {
        let w = tf.apply(NVector3::new(n.x, n.y, n.z));
        (n.x, n.y, n.z) = (w.x, w.y, w.z);
    }
}


// ---------------------------------------------------------------------------
// decoder thread + main
// ---------------------------------------------------------------------------

/// Runs the UDP capture + decode loop, publishing each completed frame (raw
/// and segmented) over zenoh. Headless -- visualization lives in a separate
/// process (zenoh_test.rs) that subscribes to these keys.
fn run_decoder(config_path: String, seg_params: Arc<Mutex<segmentation::SegParams>>) {
    let text = match std::fs::read_to_string(&config_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("rslidar: failed to read config {config_path}: {e}");
            std::process::exit(1)
        }
    };
    let cfg = DriverConfig::parse(&text);
    let costmap_params = parse_costmap_params(&text);

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

    let (tx, rx) = mpsc::channel::<RawPacket>();
    spawn_packet_source(&cfg, tx);

    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("rslidar: failed to open zenoh session: {error}");
            std::process::exit(1)
        });
    let pub_raw = publisher(&session, "rslidar/points/raw").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });
    let pub_seg = publisher(&session, "rslidar/points/segmented").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let pub_costmap = publisher(&session, "rslidar/costmap").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let pub_rangegraph = publisher(&session, "rslidar/points/rangegraph").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let pub_pose_frame = publisher(&session, "rslidar/pose_frame").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let poses: PoseBuffer = Arc::new(Mutex::new(VecDeque::new()));
    let poses_in = Arc::clone(&poses);
    let _pose_subscriber = session
        .declare_subscriber(POSE_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            if let Some(pose) = decode_pose(&payload) {
                let mut buf = poses_in.lock().unwrap();
                if buf.len() == POSE_BUFFER_LEN {
                    buf.pop_front();
                }
                buf.push_back(pose);
            } else {
                eprintln!(
                    "zenoh_test: dropping malformed pose ({} bytes)",
                    payload.len()
                );
            }
        })
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to subscribe to {POSE_KEY}: {error}");
            std::process::exit(1)
        });

    let mut costmap = costmap_2d::Costmap::new(&costmap_params);
    let mut decoder = RsHeliosDecoder::new(&cfg, seg_params);
    let mut tracker = Tracker::new(0.5, 5);
    let mut frame_count = 0u64;
    let image_dir = std::path::Path::new("range_images");
    std::fs::create_dir_all(image_dir).expect("create range_images dir");

    println!("rslidar: waiting for DIFOP/MSOP packets...");
    for msg in rx {
        match msg {
            RawPacket::Difop(buf) => decoder.decode_difop(&buf),
            RawPacket::Msop(buf) => {
                for (mut cloud, mut range_graph) in decoder.decode_msop(&buf) {
                    frame_count += 1;

                    let stamp_ns = cloud.header.stamp_sec as i64 * 1_000_000_000
                        + cloud.header.stamp_nanosec as i64;
                    let tf = interpolate_pose(&poses.lock().unwrap(), stamp_ns)
                        .unwrap_or_else(Transform::identity);
                    to_world(&tf, &mut cloud.data, &mut range_graph);

                    let ids = tracker.update(&aggregate_clusters(&range_graph));
                    relabel(&mut range_graph, &ids);
                    if let Err(error) = pub_pose_frame.put(encode_pose(&cloud.header, &tf)).wait() {
                        eprintln!("rslidar: publish frame pose: {error}");
                    }
                    let mut point_cluster = vec![-1i32; cloud.width as usize];
                    for node in range_graph.nodes.iter() {
                        if node.valid && node.point_idx >= 0 {
                            // here, this assigns cluster_ids generated by range_graph
                            // to actual points in point_cluster
                            // so segemented point cloud is stored in point_cluster
                            point_cluster[node.point_idx as usize] = node.beta;
                        }
                    }

                    let cloud_start = Instant::now();
                    let raw_payload = encode_points(&cloud.header, cloud.point_step, &cloud.data);
                    let raw_bytes = raw_payload.len();
                    if let Err(error) = pub_raw.put(raw_payload).wait() {
                        eprintln!("rslidar: publish raw cloud: {error}");
                    }
                    let seg_data = build_segmented_data(&cloud.data, &point_cluster);
                    let seg_payload = encode_points(&cloud.header, SEG_POINT_STEP, &seg_data);
                    let seg_bytes = seg_payload.len();
                    if let Err(error) = pub_seg.put(seg_payload).wait() {
                        eprintln!("rslidar: publish segmented cloud: {error}");
                    }
                    let cloud_ms = cloud_start.elapsed().as_secs_f64() * 1e3;
                    println!(
                        "point cloud: {} pts, raw {:.1} KB, seg {:.1} KB ({cloud_ms:.2} ms)",
                        cloud.width,
                        raw_bytes as f64 / 1024.0,
                        seg_bytes as f64 / 1024.0,
                    );

                    let costmap_start = Instant::now();
                    costmap.update(&range_graph);
                    let costmap_ms = costmap_start.elapsed().as_secs_f64() * 1e3;
                    if let Err(error) = pub_costmap
                        .put(encode_costmap(&cloud.header, costmap.master()))
                        .wait() {
                        eprintln!("rslidar: publish costmap: {error}");
                    }
                    if let Err(error) = pub_rangegraph
                        .put(encode_rangegraph(&cloud.header, &range_graph))
                        .wait() {
                        eprintln!("rslidar: publish range graph: {error}");
                    }
                    if let Err(error) = write_images(&range_graph, image_dir, frame_count, 0, segmentation::NUM_COLS) {
                        eprintln!("rslidar: write range images: {error}");
                    }
                    println!("output (costmap update {costmap_ms:.2} ms)")
                }
            }
        }
    }
}

fn main() {
    let config_path = std::env::args().nth(1).unwrap_or_else(default_config_path);
    let seg_params = Arc::new(Mutex::new(segmentation::SegParams::default()));
    run_decoder(config_path, seg_params);
}