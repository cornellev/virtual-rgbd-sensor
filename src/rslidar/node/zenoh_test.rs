// Subscribes to the point clouds `rslidar_sdk_node` publishes over zenoh and
// renders them with Bevy. This is now the only place in the project that
// opens a window -- `rslidar_sdk_node` is a headless decode+publish process.
#![allow(dead_code)]

mod segmentation;
mod visualization;
use visualization::{
    draw_origin_axes, drain_latest_cloud, orbit_camera, setup_scene, CloudChannel, OrbitCamera,
    VizPoint,
};

use zenoh::Wait;

use bevy::prelude::*;
use bevy_points::prelude::PointsPlugin;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::Mutex;

const POINTS_KEY: &str = "rslidar/points/segmented";
static FRAME_COUNT: AtomicU64 = AtomicU64::new(0);

/// Mirrors `encode_points`/`build_segmented_data` in rslidar_sdk_node.rs: a
/// 16-byte header (stamp_sec: i32, stamp_nanosec: u32, width: u32,
/// point_step: u32, all little-endian) followed by `width` records of
/// `point_step` bytes each -- XYZI (16 bytes, `rslidar/points/raw`) or XYZI +
/// a trailing i32 cluster id (20 bytes, `rslidar/points/segmented`).
fn decode_points(payload: &[u8]) -> Option<Vec<VizPoint>> {
    if payload.len() < 16 {
        return None;
    }
    let width = u32::from_le_bytes(payload[8..12].try_into().ok()?) as usize;
    let point_step = u32::from_le_bytes(payload[12..16].try_into().ok()?) as usize;
    if point_step < 16 {
        return None;
    }
    let data = &payload[16..];
    if data.len() < width * point_step {
        return None;
    }
    let has_cluster = point_step >= 20;

    Some(
        data.chunks_exact(point_step)
            .take(width)
            .filter_map(|rec| {
                let x = f32::from_le_bytes(rec[0..4].try_into().ok()?);
                let y = f32::from_le_bytes(rec[4..8].try_into().ok()?);
                let z = f32::from_le_bytes(rec[8..12].try_into().ok()?);
                if x.is_nan() || y.is_nan() || z.is_nan() {
                    return None;
                }
                let cluster = if has_cluster {
                    i32::from_le_bytes(rec[16..20].try_into().ok()?)
                } else {
                    -1
                };
                Some(VizPoint { x, y, z, cluster })
            })
            .collect(),
    )
}

fn main() {
    let (viz_tx, viz_rx) = mpsc::channel::<Vec<VizPoint>>();

    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to open zenoh session: {error}");
            std::process::exit(1)
        });

    // Kept alive for the whole process: dropping it would cancel the
    // subscription.
    let _subscriber = session
        .declare_subscriber(POINTS_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            if let Some(points) = decode_points(&payload) {
                let n = FRAME_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
                if n % 30 == 1 {
                    println!("zenoh_test: frame {n} ({} points)", points.len());
                }
                // Ignore send errors: they just mean the Bevy window closed,
                // at which point we keep receiving harmlessly until exit.
                let _ = viz_tx.send(points);
            } else {
                eprintln!("zenoh_test: dropping malformed sample ({} bytes)", payload.len());
            }
        })
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to subscribe to {POINTS_KEY}: {error}");
            std::process::exit(1)
        });

    println!("zenoh_test: subscribed to {POINTS_KEY}, waiting for frames...");

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PointsPlugin)
        .insert_resource(CloudChannel(Mutex::new(viz_rx)))
        .insert_resource(OrbitCamera::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Update, (drain_latest_cloud, orbit_camera, draw_origin_axes))
        .run();
}
