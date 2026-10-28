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
const SHM_POOL_SIZE: usize = 32 * 1024 * 1024;

use anyhow::Result;
use virtual_rgbd_sensor::costmap_2d;
use virtual_rgbd_sensor::segmentation;
use virtual_rgbd_sensor::rslidar_math::*;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    DriverConfig, RawPacket, RsHeliosDecoder, default_config_path,
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

const POINTS_HEADER_LEN: usize = 16;

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

fn encode_points_into(buf: &mut [u8], header: &Header, point_step: u32, data: &[u8]) {
    let (hdr, body) = buf.split_at_mut(POINTS_HEADER_LEN);
    let width = data.len() as u32 / point_step;
    hdr[0..4].copy_from_slice(&header.stamp_sec.to_le_bytes());
    hdr[4..8].copy_from_slice(&header.stamp_nanosec.to_le_bytes());
    hdr[8..12].copy_from_slice(&width.to_le_bytes());
    hdr[12..16].copy_from_slice(&point_step.to_le_bytes());
    body.copy_from_slice(data);
}

const RANGEGRAPH_RECORD_LEN: usize = 24; // x, y, z, intensity, range (f32) + cluster_id (i32)
const RANGEGRAPH_HEADER_LEN: usize = 32;

fn rangegraph_len(graph: &segmentation::RangeGraph) -> usize {
    RANGEGRAPH_HEADER_LEN + graph.nodes.len() * RANGEGRAPH_RECORD_LEN
}

/// Wire format published on `rslidar/points/rangegraph`: a 32-byte header
/// (stamp_sec: i32, stamp_nanosec: u32, rows: u32, cols: u32,
/// az_res_centideg: u32, record_len: u32, reserved: u64, all little-endian)
/// followed by `rows * cols` records indexed `ring * cols + col`. Empty cells
/// have range 0.0 and cluster_id -1.
fn encode_rangegraph_into(buf: &mut [u8], header: &Header, graph: &segmentation::RangeGraph) {
    let (hdr, body) = buf.split_at_mut(RANGEGRAPH_HEADER_LEN);
    hdr[0..4].copy_from_slice(&header.stamp_sec.to_le_bytes());
    hdr[4..8].copy_from_slice(&header.stamp_nanosec.to_le_bytes());
    hdr[8..12].copy_from_slice(&(segmentation::NUM_RINGS as u32).to_le_bytes());
    hdr[12..16].copy_from_slice(&(segmentation::NUM_COLS as u32).to_le_bytes());
    hdr[16..20].copy_from_slice(&(segmentation::AZ_RES_CENTIDEG as u32).to_le_bytes());
    hdr[20..24].copy_from_slice(&(RANGEGRAPH_RECORD_LEN as u32).to_le_bytes());
    hdr[24..32].copy_from_slice(&0u64.to_le_bytes());
    for (rec, n) in body.chunks_exact_mut(RANGEGRAPH_HEADER_LEN).zip(&graph.nodes) {
        let (range, cluster) = if n.valid { (n.range, n.beta) } else { (0.0, -1) };
        for (i, v) in [n.x, n.y, n.z, n.intensity, range].into_iter().enumerate() {
            rec[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        rec[20..24].copy_from_slice(&cluster.to_le_bytes());
    }
}

const COSTMAP_HEADER_LEN: usize = 32;

fn costmap_len(map: &costmap_2d::Costmap2D) -> usize {
    COSTMAP_HEADER_LEN + map.data.len()
}

fn encode_costmap_into(buf: &mut[u8], header: &Header, map: &costmap_2d::Costmap2D) {
    let (hdr, body) = buf.split_at_mut(COSTMAP_HEADER_LEN);
    hdr[0..4].copy_from_slice(&header.stamp_sec.to_le_bytes());
    hdr[4..8].copy_from_slice(&header.stamp_nanosec.to_le_bytes());
    hdr[8..12].copy_from_slice(&(map.size_x as u32).to_le_bytes());
    hdr[12..16].copy_from_slice(&(map.size_y as u32).to_le_bytes());
    hdr[16..20].copy_from_slice(&map.resolution.to_le_bytes());
    hdr[20..24].copy_from_slice(&map.origin_x.to_le_bytes());
    hdr[24..28].copy_from_slice(&map.origin_y.to_le_bytes());
    hdr[28..32].copy_from_slice(&0u32.to_le_bytes());
    body.copy_from_slice(&map.data);
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

fn encode_segmented_into(buf: &mut [u8], header: &Header, cloud_data: &[u8], point_cluster: &[i32]) {
    let (hdr, body) = buf.split_at_mut(POINTS_HEADER_LEN);
    hdr[0..4].copy_from_slice(&header.stamp_sec.to_le_bytes());
    hdr[4..8].copy_from_slice(&header.stamp_nanosec.to_le_bytes());
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

    let shm = create_shm_provider(SHM_POOL_SIZE).unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let mut config = zenoh::Config::default();
    config.insert_json5("transport/shared_memory/transport_optimization/enabled", "false").unwrap();
    let session = zenoh::open(config)
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

    let pub_occupancy = publisher(&session, "rslidar/occupancy").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    let pub_rangegraph = publisher(&session, "rslidar/points/rangegraph").unwrap_or_else(|error| {
        eprintln!("rslidar: {error}");
        std::process::exit(1)
    });

    // Live tuning from zenoh_viz; the decoder rereads seg_params every revolution.
    let shared_params = seg_params.clone();
    let _seg_params_sub = session
        .declare_subscriber(segmentation::SEG_PARAMS_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            match segmentation::SegParams::from_bytes(&payload) {
                Some(params) => {
                    println!("rslidar: seg params updated (seg_enabled={})", params.seg_enabled);
                    *shared_params.lock().unwrap() = params;
                }
                None => eprintln!("rslidar: dropping malformed seg params ({} bytes)", payload.len()),
            }
        })
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("rslidar: failed to subscribe to {}: {error}", segmentation::SEG_PARAMS_KEY);
            std::process::exit(1)
        });

    let mut costmap = costmap_2d::Costmap::new(&costmap_params);
    let mut decoder = RsHeliosDecoder::new(
        &cfg, seg_params
    );
    let image_dir = std::path::Path::new("range_images");
    std::fs::create_dir_all(image_dir).expect("create range_images dir");

    println!("rslidar: waiting for DIFOP/MSOP packets...");
    for msg in rx {
        match msg {
            RawPacket::Difop(buf) => decoder.decode_difop(&buf),
            RawPacket::Msop(buf) => {
                for (cloud, range_graph) in decoder.decode_msop(&buf) {
                    let mut point_cluster = vec![-1i32; cloud.width as usize];
                    for node in range_graph.nodes.iter() {
                        if node.valid && node.point_idx >= 0 {
                            point_cluster[node.point_idx as usize] = node.beta;
                        }
                    }

                    let raw_bytes = POINTS_HEADER_LEN + cloud.data.len();
                    if let Err(error) = publish(&pub_raw, &shm, raw_bytes, |buf| {
                        encode_points_into(buf, &cloud.header, cloud.point_step, &cloud.data)
                    }) {
                        eprintln!("rslidar: publish raw cloud: {error}");
                    }

                    let seg_bytes = POINTS_HEADER_LEN + point_cluster.len() * SEG_POINT_STEP as usize;
                    if let Err(error) = publish(&pub_seg, &shm, seg_bytes, |buf| {
                        encode_segmented_into(buf, &cloud.header, &cloud.data, &point_cluster)
                    }) {
                        eprintln!("rslidar: publish segmented cloud: {error}");
                    }

                    let costmap_start = Instant::now();
                    costmap.update(&range_graph);
                    let costmap_ms = costmap_start.elapsed().as_secs_f64() * 1e3;

                    let map = costmap.master();
                    if let Err(error) = publish(&pub_occupancy, &shm, costmap_len(map), |buf| {
                        encode_costmap_into(buf, &cloud.header, map)
                    }) {
                        eprintln!("rslidar: publish costmap: {error}");
                    }

                    if let Err(error) = publish(&pub_rangegraph, &shm, rangegraph_len(&range_graph), |buf| {
                        encode_rangegraph_into(buf, &cloud.header, &range_graph)
                    }) {
                        eprintln!("rslidar: publish range graph: {error}");
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
