// zenoh_wrapper for protobuf stuff
// cev/sensors/lidar/raw_points             | PointCloud (step 16)| world   |
// cev/sensors/lidar/segmented_points       | PointCloud (step 20)| world   |
// cev/sensors/lidar/range_graph            | RangeGraph          | world   |
// cev/sensors/lidar/occupancy_grid/local   | OccupancyGrid       | lidar   |
// cev/sensors/lidar/occupancy_grid/global  | OccupancyGrid       | world   |
// cev/sensors/lidar/occupancy_grid/diff    | OccupancyGridDiff   | world   |
// cev/sensors/pose_frame                   | Transform           | world   |
// cev/sensors/pose_world (subscribed)      | Transform           | world   |
#![allow(dead_code)]

use zenoh::{Wait, pubsub::Publisher, qos::CongestionControl};

use nalgebra::{Quaternion as NQuaternion, UnitQuaternion as NUnitQuaternion, Vector3 as NVector3};

use anyhow::Result;
use virtual_rgbd_sensor::cev_proto::{self as pb, Message};
use virtual_rgbd_sensor::costmap_2d::{self, Diff};
use virtual_rgbd_sensor::rslidar_math::*;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    DriverConfig, RawPacket, RsHeliosDecoder, default_config_path,
    parse_costmap_params, parse_seg_params, spawn_packet_source,
};
use virtual_rgbd_sensor::segmentation;
use virtual_rgbd_sensor::tracking::{aggregate_clusters, relabel, Tracker};
use virtual_rgbd_sensor::transform::{interpolate_pose, PoseBuffer, Transform};
use virtual_rgbd_sensor::visualize::*;

use std::collections::VecDeque;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Instant;

const POSE_KEY: &str = "cev/sensors/pose_world";
const POSE_BUFFER_LEN: usize = 1024;
/// The global map changes slowly and is large, so it goes out at 1 Hz.
const GLOBAL_PUBLISH_EVERY: u64 = 10;
const SEG_POINT_STEP: u32 = 20; // x, y, z, intensity (f32) + cluster_id (i32)
/// Frame id for everything `to_world` has moved into the world frame.
const WORLD_FRAME: &str = "world";

fn publisher<'a>(session: &'a zenoh::Session, key: &'static str) -> Publisher<'a> {
    session
        .declare_publisher(key)
        .congestion_control(CongestionControl::Drop)
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_protobuf: declare {key} publisher: {error}");
            std::process::exit(1)
        })
}

/// A publisher plus its `Header.seq` counter (sequential per topic).
struct Topic<'a> {
    key: &'static str,
    publisher: Publisher<'a>,
    seq: u64,
}

impl<'a> Topic<'a> {
    fn new(session: &'a zenoh::Session, key: &'static str) -> Self {
        Self { key, publisher: publisher(session, key), seq: 0 }
    }

    /// Header for the next message on this topic.
    fn header(&mut self, stamp_ns: u64, frame_id: &str) -> pb::Header {
        let header = pb::Header::new(stamp_ns, frame_id, self.seq);
        self.seq += 1;
        header
    }

    /// Encodes and publishes `msg`, returning the encoded size in bytes.
    fn put(&self, msg: &impl Message) -> usize {
        let payload = msg.encode_to_vec();
        let len = payload.len();
        if let Err(error) = self.publisher.put(payload).wait() {
            eprintln!("zenoh_protobuf: publish {}: {error}", self.key);
        }
        len
    }
}

// ---------------------------------------------------------------------------
// encoders: internal types -> lidar.proto messages
// ---------------------------------------------------------------------------

fn encode_points(header: pb::Header, point_step: u32, data: Vec<u8>) -> pb::PointCloud {
    pb::PointCloud {
        header: Some(header),
        width: data.len() as u32 / point_step,
        point_step,
        data: data.into(),
    }
}

fn encode_rangegraph(header: pb::Header, graph: &segmentation::RangeGraph) -> pb::RangeGraph {
    pb::RangeGraph {
        header: Some(header),
        num_rings: segmentation::NUM_RINGS as u32,
        num_cols: segmentation::NUM_COLS as u32,
        azimuth_resolution_centideg: segmentation::AZ_RES_CENTIDEG as u32,
        nodes: graph
            .nodes
            .iter()
            .map(|n| pb::RangeNode {
                x: n.x,
                y: n.y,
                z: n.z,
                intensity: n.intensity,
                range: n.range,
                azimuth_centideg: n.azimuth_centideg,
                point_idx: n.point_idx,
                valid: n.valid,
                alpha: n.alpha,
                beta: n.beta,
            })
            .collect(),
    }
}

fn encode_costmap(header: pb::Header, map: &costmap_2d::Costmap2D) -> pb::OccupancyGrid {
    pb::OccupancyGrid {
        header: Some(header),
        size_x: map.size_x as u32,
        size_y: map.size_y as u32,
        resolution_m: map.resolution,
        origin_x: map.origin_x,
        origin_y: map.origin_y,
        data: map.data.clone().into(),
    }
}

fn encode_diff(header: pb::Header, diff: &[Diff]) -> pb::OccupancyGridDiff {
    pb::OccupancyGridDiff {
        header: Some(header),
        diffs: diff
            .iter()
            .map(|d| pb::Diff { mx: d.mx as u32, my: d.my as u32, old: d.old as u32, new: d.new as u32 })
            .collect(),
    }
}

fn encode_pose(header: pb::Header, tf: &Transform) -> pb::Transform {
    let t = tf.translation;
    let q = tf.rotation.quaternion();
    pb::Transform {
        header: Some(header),
        translation: Some(pb::Vector3 { x: t.x as f64, y: t.y as f64, z: t.z as f64 }),
        rotation: Some(pb::Quaternion { x: q.i as f64, y: q.j as f64, z: q.k as f64, w: q.w as f64 }),
    }
}

/// `cev/sensors/pose_world` as a protobuf `Transform`. Missing translation
/// defaults to zero; missing rotation to identity.
fn decode_pose(payload: &[u8]) -> Option<(i64, Transform)> {
    let msg = pb::Transform::decode(payload).ok()?;
    let stamp_ns = msg.header.as_ref().map_or(0, |h| h.stamp_ns) as i64;
    let t = msg.translation.unwrap_or_default();
    let q = msg.rotation.unwrap_or(pb::Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 });
    Some((stamp_ns, Transform {
        translation: NVector3::new(t.x as f32, t.y as f32, t.z as f32),
        rotation: NUnitQuaternion::from_quaternion(NQuaternion::new(
            q.w as f32, q.x as f32, q.y as f32, q.z as f32,
        )),
    }))
}

/// `cloud_data`'s XYZI points (see `POINT_STEP`) with each point's cluster id
/// from `point_cluster` appended as a trailing little-endian i32.
fn build_segmented_data(cloud_data: &[u8], point_cluster: &[i32]) -> Vec<u8> {
    let mut data = Vec::with_capacity(cloud_data.len() + point_cluster.len() * 4);
    for (chunk, &cluster) in cloud_data.chunks_exact(POINT_STEP as usize).zip(point_cluster) {
        data.extend_from_slice(chunk);
        data.extend_from_slice(&cluster.to_le_bytes());
    }
    data
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

fn run_decoder(
    config_path: String,
    seg_params: Arc<Mutex<segmentation::SegParams>>,
    global_map: &mut costmap_2d::GlobalMap,
    range_images: bool,
) -> Result<()> {
    let text = std::fs::read_to_string(&config_path)
        .map_err(|e| anyhow::anyhow!("read config {config_path}: {e}"))?;
    let cfg = DriverConfig::parse(&text);
    let costmap_params = parse_costmap_params(&text);

    if cfg.lidar_type != "RSHELIOS" {
        eprintln!(
            "zenoh_protobuf: warning: config.yaml selects lidar_type={}, but only the \
             RSHELIOS MSOP/DIFOP layout is implemented. Decoding will likely fail.",
            cfg.lidar_type
        );
    }
    println!("zenoh_protobuf: config loaded from {config_path}");
    println!(
        "zenoh_protobuf: host_address={} msop_port={} difop_port={} lidar_type={}",
        cfg.host_address, cfg.msop_port, cfg.difop_port, cfg.lidar_type
    );

    let (tx, rx) = mpsc::channel::<RawPacket>();
    spawn_packet_source(&cfg, tx);

    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .map_err(|e| anyhow::anyhow!("open zenoh session: {e}"))?;
    let mut raw = Topic::new(&session, "cev/sensors/lidar/raw_points");
    let mut seg = Topic::new(&session, "cev/sensors/lidar/segmented_points");
    let mut local = Topic::new(&session, "cev/sensors/lidar/occupancy_grid/local");
    let mut rangegraph = Topic::new(&session, "cev/sensors/lidar/range_graph");
    let mut global = Topic::new(&session, "cev/sensors/lidar/occupancy_grid/global");
    let mut diff_topic = Topic::new(&session, "cev/sensors/lidar/occupancy_grid/diff");
    let mut pose_frame = Topic::new(&session, "cev/sensors/pose_frame");

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
                let hint = if payload.len() == 40 {
                    " -- looks like zenoh_wrapper's raw pose layout; run replay.py with --proto"
                } else {
                    ""
                };
                eprintln!("zenoh_protobuf: dropping malformed pose ({} bytes){hint}", payload.len());
            }
        })
        .wait()
        .map_err(|e| anyhow::anyhow!("subscribe to {POSE_KEY}: {e}"))?;

    let mut costmap = costmap_2d::Costmap::new(&costmap_params);
    let mut decoder = RsHeliosDecoder::new(&cfg, seg_params);
    let mut tracker = Tracker::new(0.5, 5);
    let mut frame_count = 0u64;
    let image_dir = range_images.then(|| {
        let dir = std::path::Path::new("range_images");
        std::fs::create_dir_all(dir).expect("create range_images dir");
        dir
    });

    println!("zenoh_protobuf: waiting for DIFOP/MSOP packets...");
    for msg in rx {
        match msg {
            RawPacket::Difop(buf) => decoder.decode_difop(&buf),
            RawPacket::Msop(buf) => {
                for (mut cloud, mut range_graph) in decoder.decode_msop(&buf) {
                    frame_count += 1;
                    let stamp = pb::stamp_ns(cloud.header.stamp_sec, cloud.header.stamp_nanosec);
                    let lidar_frame = cloud.header.frame_id.clone();

                    let tf = interpolate_pose(&poses.lock().unwrap(), stamp as i64)
                        .unwrap_or_else(Transform::identity);

                    // local costmap, lidar frame
                    let costmap_start = Instant::now();
                    costmap.update(&range_graph);
                    let costmap_ms = costmap_start.elapsed().as_secs_f64() * 1e3;
                    let h = local.header(stamp, &lidar_frame);
                    local.put(&encode_costmap(h, costmap.master()));

                    to_world(&tf, &mut cloud.data, &mut range_graph);

                    let ids = tracker.update(&aggregate_clusters(&range_graph));
                    relabel(&mut range_graph, &ids);
                    let h = pose_frame.header(stamp, WORLD_FRAME);
                    pose_frame.put(&encode_pose(h, &tf));

                    let mut point_cluster = vec![-1i32; cloud.width as usize];
                    for node in range_graph.nodes.iter() {
                        if node.valid && node.point_idx >= 0 {
                            point_cluster[node.point_idx as usize] = node.beta;
                        }
                    }

                    let cloud_start = Instant::now();
                    let seg_data = build_segmented_data(&cloud.data, &point_cluster);
                    let h = raw.header(stamp, WORLD_FRAME);
                    let raw_bytes = raw.put(&encode_points(h, cloud.point_step, cloud.data));
                    let h = seg.header(stamp, WORLD_FRAME);
                    let seg_bytes = seg.put(&encode_points(h, SEG_POINT_STEP, seg_data));
                    let h = rangegraph.header(stamp, WORLD_FRAME);
                    let rg_bytes = rangegraph.put(&encode_rangegraph(h, &range_graph));
                    let cloud_ms = cloud_start.elapsed().as_secs_f64() * 1e3;
                    println!(
                        "zenoh_protobuf: frame {frame_count}: {} pts, raw {:.1} KB, seg {:.1} KB, \
                         range graph {:.1} KB ({cloud_ms:.2} ms)",
                        cloud.width,
                        raw_bytes as f64 / 1024.0,
                        seg_bytes as f64 / 1024.0,
                        rg_bytes as f64 / 1024.0,
                    );

                    if let Some(dir) = image_dir {
                        if let Err(error) = write_images(&range_graph, dir, frame_count, 0, segmentation::NUM_COLS) {
                            eprintln!("zenoh_protobuf: write range images: {error}");
                        }
                    }

                    // global costmap in world frame
                    let global_start = Instant::now();
                    global_map.fuse(&costmap, &tf);
                    let global_ms = global_start.elapsed().as_secs_f64() * 1e3;
                    if frame_count % GLOBAL_PUBLISH_EVERY == 0 {
                        let h = global.header(stamp, WORLD_FRAME);
                        global.put(&encode_costmap(h, global_map.grid()));
                        let diff = global_map.take_diff();
                        let h = diff_topic.header(stamp, WORLD_FRAME);
                        diff_topic.put(&encode_diff(h, &diff));
                    }
                    println!("output (costmap update {costmap_ms:.2} ms, global fuse {global_ms:.2} ms)")
                }
            }
        }
    }
    Ok(())
}

/// Usage: zenoh_protobuf [CONFIG_PATH] [--range-images]
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let range_images = args.iter().any(|a| a == "--range-images");
    let config_path = args.iter().find(|a| !a.starts_with("--")).cloned().unwrap_or_else(default_config_path);
    let text = std::fs::read_to_string(&config_path).unwrap_or_else(|e| {
        eprintln!("zenoh_protobuf: failed to read config {config_path}: {e}");
        std::process::exit(1)
    });
    let seg_params = Arc::new(Mutex::new(parse_seg_params(&text)));
    let mut global_map = costmap_2d::GlobalMap::new(&parse_costmap_params(&text));
    if let Err(error) = run_decoder(config_path, seg_params, &mut global_map, range_images) {
        eprintln!("zenoh_protobuf: {error}");
        std::process::exit(1)
    }
}
