//! Live driver: UDP/pcap scans + `rslidar/pose_world` in, perception out over Zenoh.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Instant;

use virtual_rgbd_sensor::pipeline::Pipeline;
use virtual_rgbd_sensor::rslidar_sdk_node::{
    default_config_path, parse_costmap_params, spawn_packet_source, DriverConfig, RawPacket,
};
use virtual_rgbd_sensor::segmentation::{RangeGraph, SegParams, NUM_COLS};
use virtual_rgbd_sensor::tracking::Tracker;
use virtual_rgbd_sensor::visualize::write_images;
use virtual_rgbd_sensor::wire;
use virtual_rgbd_sensor::zenoh_io::{self, *};
use zenoh::Wait;

fn fail(e: impl std::fmt::Display) -> ! {
    eprintln!("rslidar: {e}");
    std::process::exit(1)
}

/// Writes range images off the hot path, skipping frames while a write is in
/// flight so disk I/O never delays publishing.
fn spawn_image_writer() -> impl FnMut(&RangeGraph, u64) {
    let dir = std::path::PathBuf::from("range_images");
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| fail(e));
    let busy = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::sync_channel::<(RangeGraph, u64)>(1);
    let done = Arc::clone(&busy);
    std::thread::spawn(move || {
        for (graph, frame) in rx {
            if let Err(e) = write_images(&graph, &dir, frame, 0, NUM_COLS) {
                eprintln!("rslidar: write range images: {e}");
            }
            done.store(false, Ordering::Release);
        }
    });
    move |graph, frame| {
        if !busy.swap(true, Ordering::Acquire) {
            let _ = tx.try_send((graph.clone(), frame));
        }
    }
}

fn main() {
    let config_path = std::env::args().nth(1).unwrap_or_else(default_config_path);
    let text = std::fs::read_to_string(&config_path).unwrap_or_else(|e| fail(format!("read {config_path}: {e}")));
    let cfg = DriverConfig::parse(&text);
    if cfg.lidar_type != "RSHELIOS" {
        eprintln!("rslidar: warning: lidar_type={} but only RSHELIOS is implemented", cfg.lidar_type);
    }
    println!("rslidar: {config_path}: msop {} difop {} {}", cfg.msop_port, cfg.difop_port, cfg.pcap_path.as_deref().unwrap_or("live UDP"));

    let seg_params = Arc::new(Mutex::new(SegParams::default()));
    let mut pipeline = Pipeline::new(&cfg, &parse_costmap_params(&text), Arc::clone(&seg_params), Tracker::new(0.5, 5));

    let session = zenoh::open(zenoh::Config::default()).wait().unwrap_or_else(|e| fail(e));
    let [pub_raw, pub_seg, pub_graph, pub_costmap, pub_pose] =
        [POINTS_RAW_KEY, POINTS_SEG_KEY, RANGEGRAPH_KEY, COSTMAP_KEY, POSE_FRAME_KEY]
            .map(|key| zenoh_io::publisher(&session, key).unwrap_or_else(|e| fail(e)));
    let (pose_tx, pose_rx) = mpsc::channel();
    let _poses = subscribe_poses(&session, pose_tx).unwrap_or_else(|e| fail(e));
    let _seg = subscribe_seg_params(&session, seg_params).unwrap_or_else(|e| fail(e));

    let (pkt_tx, pkt_rx) = mpsc::channel::<RawPacket>();
    spawn_packet_source(&cfg, pkt_tx);
    let mut write_range_images = spawn_image_writer();
    let mut frame_count = 0u64;

    println!("rslidar: waiting for DIFOP/MSOP packets...");
    for pkt in pkt_rx {
        let started = Instant::now();
        for (stamp_ns, pose) in pose_rx.try_iter() {
            pipeline.push_pose(stamp_ns, pose);
        }
        pipeline.push_packet(&pkt, |f| {
            frame_count += 1;
            let h = &f.cloud.header;
            let puts = [
                (&pub_pose, wire::encode_pose(h, &f.tf)),
                (&pub_raw, wire::encode_points(h, f.cloud.point_step, &f.cloud.data)),
                (&pub_seg, wire::encode_segmented(h, &f.cloud.data, f.point_cluster)),
                (&pub_costmap, wire::encode_costmap(h, f.costmap)),
                (&pub_graph, wire::encode_rangegraph(h, f.range_graph)),
            ];
            for (publisher, payload) in puts {
                if let Err(e) = publisher.put(payload).wait() {
                    eprintln!("rslidar: publish {}: {e}", publisher.key_expr());
                }
            }
            write_range_images(f.range_graph, frame_count);
            println!(
                "frame {frame_count}: {} pts, {:.2} ms{}",
                f.cloud.width,
                started.elapsed().as_secs_f64() * 1e3,
                if f.pose_bracketed { "" } else { " (pose not bracketed: held/clamped)" },
            );
        });
    }
}
