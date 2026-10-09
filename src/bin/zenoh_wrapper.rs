#![allow(dead_code)]

use zenoh::{
    Wait,
    pubsub::Publisher,
    qos::CongestionControl,
    // add shared memory import here, the goal is that instead of publisher we do SHM
    // only one version of rangegraph & costmap is shared across anybody who wants it
    shm::{
        AllocAlignment, GarbageCollect, MemoryLayout, PosixShmProviderBackend, ShmProvider,
        ShmProviderBuilder, ZShmMut,
    },
};

type RslidarShmProvider = ShmProvider<PosixShmProviderBackend>;
const SHM_POOL_SIZE: usize = 64 * 1024 * 1024;

use anyhow::Result;
use virtual_rgbd_sensor::costmap_2d;
use virtual_rgbd_sensor::segmentation;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    DriverConfig, Header, POINT_STEP, RawPacket, RsHeliosDecoder, default_config_path,
    parse_costmap_params, spawn_packet_source,
};

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

fn create_shm_provider(capacity: usize) -> Result<RslidarShmProvider> {
    let layout = MemoryLayout::new(capacity, AllocAlignment::ALIGN_8_BYTES)?;
    ShmProviderBuilder::default_backend(layout)
        .wait()
        .map_err(|error| anyhow::anyhow!("create Zenoh SHM provider: {error}"))
}

fn allocate_frame(provider: &RslidarShmProvider, len: usize) -> Result<ZShmMut> {
    let layout = MemoryLayout::new(len, AllocAlignment::ALIGN_8_BYTES)?;
    provider
        .alloc(layout)
        .with_policy::<GarbageCollect>()
        .wait()
        .map_err(|error| anyhow::anyhow!("allocate Zenoh SHM frame: {error}"))
}

fn publish(
    publisher: &Publisher<'_>,
    provider: &RslidarShmProvider,
    len: usize,
    fill: impl FnOnce(&mut [u8]),
) -> Result<()> {
    let mut frame = allocate_frame(provider, len)?;
    fill(&mut frame);
    publisher
        .put(frame)
        .wait()
        .map_err(|error| anyhow::anyhow!("publish frame: {error}"))
}

/// Wire format published on both point-cloud keys: a 16-byte header
/// (stamp_sec: i32, stamp_nanosec: u32, width: u32, point_step: u32, all
/// little-endian) followed by `width * point_step` bytes of point data.
/// `frame_id` isn't included since it's constant per key/session, not
/// per-frame.
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

// fn encode_rangegraph(header: &Header, rangegraph: &) {
//     let mut buf = Vec::with_capacity(...);
//     buf.extend_from_slice(&header.stamp.sec.to_le_bytes());
//     buf.extend_from_slice(&header.stamp_nanosec.to_le_bytes());
//     buf
// }

/// Wire format published on `rslidar/costmap`: a 32-byte header (stamp_sec:
/// i32, stamp_nanosec: u32, size_x: u32, size_y: u32, resolution: f32,
/// origin_x: f32, origin_y: f32, reserved: u32, all little-endian) followed by
/// `size_x * size_y` row-major u8 costs (0 free .. 253 inscribed, 254 lethal,
/// 255 unknown), cell (0, 0) at (origin_x, origin_y).
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

    let mut costmap = costmap_2d::Costmap::new(&costmap_params);
    let mut decoder = RsHeliosDecoder::new(&cfg, seg_params);
    let mut frame_count = 0u64;
    let image_dir = std::path::Path::new("range_images");
    std::fs::create_dir_all(image_dir).expect("create range_images dir");

    println!("rslidar: waiting for DIFOP/MSOP packets...");
    for msg in rx {
        match msg {
            RawPacket::Difop(buf) => decoder.decode_difop(&buf),
            RawPacket::Msop(buf) => {
                for (cloud, range_graph) in decoder.decode_msop(&buf) {
                    frame_count += 1;

                    // `RangeNode::point_idx` is the index a point had in
                    // decode_msop's flat per-frame point buffer, which is the
                    // exact same order `cloud.data` was serialized in -- so
                    // this just inverts that mapping to go from a point's
                    // position in `cloud.data` back to its final cluster id
                    // (`beta`, written by second_segmentation). Defaults to -1
                    // for points that never won a range-graph cell this
                    // revolution (see RangeGraph::insert's "keep the closer
                    // point" rule).
                    let mut point_cluster = vec![-1i32; cloud.width as usize];
                    for node in range_graph.nodes.iter() {
                        if node.valid && node.point_idx >= 0 {
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
                    if let Err(error) = range_graph.write_images(image_dir, frame_count, 0, segmentation::NUM_COLS) {
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
