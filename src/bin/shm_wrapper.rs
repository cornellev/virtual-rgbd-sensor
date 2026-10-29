//! Same driver as zenoh_wrapper, but each output is written straight into a
//! Zenoh shared-memory buffer instead of a heap allocation.

use std::sync::{mpsc, Arc, Mutex};

use anyhow::{anyhow, Result};
use virtual_rgbd_sensor::pipeline::Pipeline;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    default_config_path, parse_costmap_params, spawn_packet_source, DriverConfig, RawPacket,
};
use virtual_rgbd_sensor::segmentation::SegParams;
use virtual_rgbd_sensor::tracking::Tracker;
use virtual_rgbd_sensor::wire;
use virtual_rgbd_sensor::zenoh_io::{self, *};
use zenoh::{
    pubsub::Publisher,
    shm::{AllocAlignment, GarbageCollect, MemoryLayout, PosixShmProviderBackend, ShmProvider, ShmProviderBuilder},
    Wait,
};

const SHM_POOL_SIZE: usize = 32 * 1024 * 1024;

fn fail(e: impl std::fmt::Display) -> ! {
    eprintln!("rslidar: {e}");
    std::process::exit(1)
}

fn publish(
    publisher: &Publisher<'_>,
    shm: &ShmProvider<PosixShmProviderBackend>,
    len: usize,
    fill: impl FnOnce(&mut [u8]),
) -> Result<()> {
    let mut frame = shm
        .alloc(MemoryLayout::new(len, AllocAlignment::ALIGN_8_BYTES)?)
        .with_policy::<GarbageCollect>()
        .wait()
        .map_err(|e| anyhow!("allocate SHM frame: {e}"))?;
    fill(&mut frame);
    publisher.put(frame).wait().map_err(|e| anyhow!("publish {}: {e}", publisher.key_expr()))
}

fn main() {
    let config_path = std::env::args().nth(1).unwrap_or_else(default_config_path);
    let text = std::fs::read_to_string(&config_path).unwrap_or_else(|e| fail(format!("read {config_path}: {e}")));
    let cfg = DriverConfig::parse(&text);
    let seg_params = Arc::new(Mutex::new(SegParams::default()));
    let mut pipeline = Pipeline::new(&cfg, &parse_costmap_params(&text), Arc::clone(&seg_params), Tracker::new(0.5, 5));

    let layout = MemoryLayout::new(SHM_POOL_SIZE, AllocAlignment::ALIGN_8_BYTES).unwrap_or_else(|e| fail(e));
    let shm = ShmProviderBuilder::default_backend(layout).wait().unwrap_or_else(|e| fail(e));
    let mut config = zenoh::Config::default();
    config.insert_json5("transport/shared_memory/transport_optimization/enabled", "false").unwrap();
    let session = zenoh::open(config).wait().unwrap_or_else(|e| fail(e));
    let [pub_raw, pub_seg, pub_graph, pub_costmap, pub_pose] =
        [POINTS_RAW_KEY, POINTS_SEG_KEY, RANGEGRAPH_KEY, COSTMAP_KEY, POSE_FRAME_KEY]
            .map(|key| zenoh_io::publisher(&session, key).unwrap_or_else(|e| fail(e)));
    let (pose_tx, pose_rx) = mpsc::channel();
    let _poses = subscribe_poses(&session, pose_tx).unwrap_or_else(|e| fail(e));
    let _seg = subscribe_seg_params(&session, seg_params).unwrap_or_else(|e| fail(e));

    let (pkt_tx, pkt_rx) = mpsc::channel::<RawPacket>();
    spawn_packet_source(&cfg, pkt_tx);

    println!("rslidar: waiting for DIFOP/MSOP packets...");
    for pkt in pkt_rx {
        for (stamp_ns, pose) in pose_rx.try_iter() {
            pipeline.push_pose(stamp_ns, pose);
        }
        pipeline.push_packet(&pkt, |f| {
            let h = &f.cloud.header;
            let results = [
                publish(&pub_pose, &shm, wire::POSE_LEN, |b| wire::encode_pose_into(b, h, &f.tf)),
                publish(&pub_raw, &shm, wire::points_len(&f.cloud.data), |b| {
                    wire::encode_points_into(b, h, f.cloud.point_step, &f.cloud.data)
                }),
                publish(&pub_seg, &shm, wire::segmented_len(f.point_cluster), |b| {
                    wire::encode_segmented_into(b, h, &f.cloud.data, f.point_cluster)
                }),
                publish(&pub_costmap, &shm, wire::costmap_len(f.costmap), |b| wire::encode_costmap_into(b, h, f.costmap)),
                publish(&pub_graph, &shm, wire::rangegraph_len(f.range_graph), |b| {
                    wire::encode_rangegraph_into(b, h, f.range_graph)
                }),
            ];
            for e in results.into_iter().filter_map(Result::err) {
                eprintln!("rslidar: {e}");
            }
        });
    }
}
