// Subscribes to the point clouds `rslidar_sdk_node` publishes over zenoh and
// renders them with Bevy. This is now the only place in the project that
// opens a window -- `rslidar_sdk_node` is a headless decode+publish process.
#![allow(dead_code)]

use virtual_rgbd_sensor::costmap_2d::Diff;
use virtual_rgbd_sensor::segmentation;
use virtual_rgbd_sensor::config::{parse_costmap_params, parse_seg_params};
use virtual_rgbd_sensor::rslidar_sdk_node::default_config_path;
mod visualization;
use visualization::{
    draw_origin_axes, drain_latest_cloud, drain_latest_costmap, drain_latest_pose, orbit_camera, setup_scene,
    toggle_color_mode, toggle_costmap, ShowCostmap, draw_vehicle,
    tune_seg_params, CloudChannel, CloudColorMode,
    CostmapChannel, CostmapKind, CostmapView, LidarMountHeight, LidarPose, OrbitCamera, PoseChannel, SegParamsHandle, SegParamsPublisher,
    VizCostmap, VizPoint,
    HeaderCheck, proto_decode_costmap, proto_decode_diff, proto_decode_points, proto_decode_pose,
    proto_decode_range_graph,
};

use virtual_rgbd_sensor::cev_proto as pb;
use zenoh::Wait;

use bevy::prelude::*;
use bevy_points::prelude::PointsPlugin;

use nalgebra::{Quaternion as NQuaternion, UnitQuaternion as NUnitQuaternion, Vector3 as NVector3};

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

const POINTS_KEY: &str = "cev/sensors/lidar/segmented_points";
const LOCAL_OCC_KEY: &str = "cev/sensors/lidar/occupancy_grid/local";
const GLOBAL_OCC_KEY: &str = "cev/sensors/lidar/occupancy_grid/global";
const POSE_KEY: &str = "cev/sensors/pose_frame";
const RANGE_GRAPH_KEY: &str = "cev/sensors/lidar/range_graph";
const DIFF_KEY: &str = "cev/sensors/lidar/occupancy_grid/diff";
static FRAME_COUNT: AtomicU64 = AtomicU64::new(0);

/// Fixed axis remap, lidar (REP-103: x-forward, y-left, z-up) -> Bevy
/// (y-up, -z-forward): `(x, y, z) -> (x, z, -y)`. Verified as a -90 deg
/// rotation about the lidar's own X axis -- same remap `drain_latest_cloud`
/// already applies to points, pulled out here so the pose can reuse it.
fn lidar_to_bevy_rotation() -> NUnitQuaternion<f32> {
    NUnitQuaternion::from_axis_angle(&NVector3::x_axis(), -std::f32::consts::FRAC_PI_2)
}

/// Mirrors `encode_points`/`build_segmented_data` in rslidar_sdk_node.rs: a
/// 16-byte header (stamp_sec: i32, stamp_nanosec: u32, width: u32,
/// point_step: u32, all little-endian) followed by `width` records of
/// `point_step` bytes each -- XYZI (16 bytes, `cev/sensors/lidar/raw_points`) or XYZI +
/// a trailing i32 cluster id (20 bytes, `cev/sensors/lidar/segmented_points`).
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
                let intensity = f32::from_le_bytes(rec[12..16].try_into().ok()?);
                if x.is_nan() || y.is_nan() || z.is_nan() {
                    return None;
                }
                let cluster = if has_cluster {
                    i32::from_le_bytes(rec[16..20].try_into().ok()?)
                } else {
                    -1
                };
                Some(VizPoint { x, y, z, cluster, intensity })
            })
            .collect(),
    )
}

/// Mirrors `encode_costmap` in rslidar_sdk_node.rs: a 32-byte header
/// (stamp_sec: i32, stamp_nanosec: u32, size_x: u32, size_y: u32,
/// resolution: f32, origin_x: f32, origin_y: f32, reserved: u32, all
/// little-endian) followed by `size_x * size_y` row-major u8 costs.
fn decode_costmap(payload: &[u8]) -> Option<VizCostmap> {
    if payload.len() < 32 {
        return None;
    }
    let u32_at = |i: usize| u32::from_le_bytes(payload[i..i + 4].try_into().unwrap());
    let f32_at = |i: usize| f32::from_le_bytes(payload[i..i + 4].try_into().unwrap());
    let size_x = u32_at(8) as usize;
    let size_y = u32_at(12) as usize;
    let data = &payload[32..];
    if size_x == 0 || size_y == 0 || data.len() < size_x * size_y {
        return None;
    }
    Some(VizCostmap {
        size_x,
        size_y,
        resolution: f32_at(16),
        origin_x: f32_at(20),
        origin_y: f32_at(24),
        data: data[..size_x * size_y].to_vec(),
    })
}

fn decode_diff(payload: &[u8]) -> Option<Vec<u8>> {
    if payload.len() < 16 {
        return None; // means that there is no header and stamp info
    }
    let len = u32::from_le_bytes(payload[8..12].try_into().ok()?) as usize;
    let data = &payload[12..];

    Some(data[..len].to_vec())
}

/// The pose zenoh_wrapper applied to the latest cloud, so the vehicle is
/// drawn at the same instant as the points. Mirrors `encode_pose` in
/// zenoh_wrapper.rs: a 12-byte header (stamp_sec:
/// i32, stamp_nanosec: u32, reserved: u32) followed by translation (x, y, z:
/// f32) and rotation (qx, qy, qz, qw: f32), all little-endian, 40 bytes
/// total. Both get the same lidar -> Bevy axis remap `drain_latest_cloud`
/// applies to points -- a direct remap for translation, a conjugation by
/// `lidar_to_bevy_rotation()` for the rotation (changing the axes a rotation
/// is expressed in isn't the same as remapping a point).
fn decode_pose(payload: &[u8]) -> Option<(Vec3, Quat)> {
    if payload.len() < 40 {
        return None;
    }
    let f32_at = |i: usize| f32::from_le_bytes(payload[i..i + 4].try_into().unwrap());
    let t_lidar = NVector3::new(f32_at(12), f32_at(16), f32_at(20));
    let r_lidar = NUnitQuaternion::from_quaternion(NQuaternion::new(
        f32_at(36), f32_at(24), f32_at(28), f32_at(32),
    ));
    Some(pose_to_bevy(t_lidar, r_lidar))
}

/// Remaps a lidar-axes (REP-103) pose into Bevy's axes; see `decode_pose`.
fn pose_to_bevy(t_lidar: NVector3<f32>, r_lidar: NUnitQuaternion<f32>) -> (Vec3, Quat) {
    let p = lidar_to_bevy_rotation();
    let t_bevy = p.transform_vector(&t_lidar);
    let r_bevy = p * r_lidar * p.inverse();
    let q = r_bevy.quaternion();

    (
        Vec3::new(t_bevy.x, t_bevy.y, t_bevy.z),
        Quat::from_xyzw(q.i, q.j, q.k, q.w),
    )
}

/// A payload decoder for one topic: the legacy raw layout or protobuf.
type Decoder<T> = Box<dyn Fn(&[u8]) -> Result<T, String> + Send + Sync>;

/// Wraps a legacy `Option`-returning decoder, or a protobuf decoder whose
/// header is run through a per-topic `HeaderCheck`.
fn decoder<T: 'static, P: 'static>(
    proto: bool,
    key: &'static str,
    legacy: fn(&[u8]) -> Option<T>,
    decode_proto: fn(&[u8]) -> Result<(Option<pb::Header>, P), String>,
    from_proto: fn(P) -> T,
) -> Decoder<T> {
    if proto {
        let check = HeaderCheck::new(key);
        Box::new(move |payload| {
            let (header, value) = decode_proto(payload)?;
            check.check(header.as_ref());
            Ok(from_proto(value))
        })
    } else {
        Box::new(move |payload| legacy(payload).ok_or_else(|| "malformed".to_string()))
    }
}

/// Usage: zenoh_viz [CONFIG_PATH] [--proto]. Reads the same config.yaml as the wrapper
/// for the LiDAR mount height and the starting seg params.
///
/// `--proto` decodes the cev-interfaces lidar.proto messages `zenoh_protobuf`
/// publishes instead of `zenoh_wrapper`'s raw layouts, and also subscribes to
/// the range graph and occupancy diff (logged, not drawn) so every message
/// type gets exercised.
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let proto = args.iter().any(|a| a == "--proto");
    let config_path = args.iter().find(|a| !a.starts_with("--")).cloned().unwrap_or_else(default_config_path);
    let config_text = std::fs::read_to_string(&config_path).unwrap_or_else(|e| {
        eprintln!("zenoh_test: failed to read config {config_path}: {e}");
        std::process::exit(1)
    });
    let mount_height = parse_costmap_params(&config_text).sensor_z;
    let seg_params = parse_seg_params(&config_text);

    let (viz_tx, viz_rx) = mpsc::channel::<Vec<VizPoint>>();
    let (costmap_tx, costmap_rx) = mpsc::channel::<(CostmapKind, VizCostmap)>();
    let (pose_tx, pose_rx) = mpsc::channel::<(Vec3, Quat)>();

    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to open zenoh session: {error}");
            std::process::exit(1)
        });

    let decode_points = decoder(proto, POINTS_KEY, decode_points, proto_decode_points, |p| p);
    // Kept alive for the whole process: dropping it would cancel the
    // subscription.
    let _points_subscriber = session
        .declare_subscriber(POINTS_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            match decode_points(&payload) { Ok(points) => {
                let n = FRAME_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
                if n % 30 == 1 {
                    println!("zenoh_test: frame {n} ({} points)", points.len());
                }
                // Ignore send errors: they just mean the Bevy window closed,
                // at which point we keep receiving harmlessly until exit.
                let _ = viz_tx.send(points);
            }
            Err(error) => eprintln!("zenoh_test: dropping sample on {POINTS_KEY} ({} bytes): {error}", payload.len()),
            }
        })
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to subscribe to {POINTS_KEY}: {error}");
            std::process::exit(1)
        });

    let _costmap_subscribers = [(LOCAL_OCC_KEY, CostmapKind::Local), (GLOBAL_OCC_KEY, CostmapKind::Global)]
        .map(|(key, kind)| {
            let tx = costmap_tx.clone();
            let decode_costmap = decoder(proto, key, decode_costmap, proto_decode_costmap, |c| c);
            session
                .declare_subscriber(key)
                .callback(move |sample| {
                    let payload = sample.payload().to_bytes();
                    match decode_costmap(&payload) {
                        Ok(costmap) => { let _ = tx.send((kind, costmap)); }
                        Err(error) => eprintln!("zenoh_test: dropping costmap on {key} ({} bytes): {error}", payload.len()),
                    }
                })
                .wait()
                .unwrap_or_else(|error| {
                    eprintln!("zenoh_test: failed to subscribe to {key}: {error}");
                    std::process::exit(1)
                })
        });

    let decode_pose = decoder(proto, POSE_KEY, decode_pose, proto_decode_pose, |(t, r)| pose_to_bevy(t, r));
    let _pose_subscriber = session
        .declare_subscriber(POSE_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            match decode_pose(&payload) {
                Ok(pose) => { let _ = pose_tx.send(pose); }
                Err(error) => eprintln!("zenoh_test: dropping pose ({} bytes): {error}", payload.len()),
            }
        })
        .wait()
        .unwrap_or_else(|error| {
            eprintln!("zenoh_test: failed to subscribe to {POSE_KEY}: {error}");
            std::process::exit(1)
        });

    // Protobuf-only extras: nothing draws these yet, they're decoded and
    // logged so their messages get checked too.
    let _proto_subscribers = proto.then(|| {
        let check = HeaderCheck::new(RANGE_GRAPH_KEY);
        let count = AtomicU64::new(0);
        let range_graph = session
            .declare_subscriber(RANGE_GRAPH_KEY)
            .callback(move |sample| {
                let payload = sample.payload().to_bytes();
                match proto_decode_range_graph(&payload) {
                    Ok((header, s)) => {
                        check.check(header.as_ref());
                        if count.fetch_add(1, Ordering::Relaxed) % 30 == 0 {
                            println!(
                                "proto [{RANGE_GRAPH_KEY}]: {}x{} @ {} centideg, {} valid nodes, {} clusters ({:.1} KB)",
                                s.num_rings, s.num_cols, s.azimuth_resolution_centideg, s.valid, s.clusters,
                                payload.len() as f64 / 1024.0
                            );
                        }
                    }
                    Err(error) => eprintln!("zenoh_test: dropping range graph ({} bytes): {error}", payload.len()),
                }
            })
            .wait()
            .unwrap_or_else(|error| {
                eprintln!("zenoh_test: failed to subscribe to {RANGE_GRAPH_KEY}: {error}");
                std::process::exit(1)
            });
        let check = HeaderCheck::new(DIFF_KEY);
        let diff = session
            .declare_subscriber(DIFF_KEY)
            .callback(move |sample| {
                let payload = sample.payload().to_bytes();
                match proto_decode_diff(&payload) {
                    Ok((header, diffs)) => {
                        check.check(header.as_ref());
                        let lethal = diffs.iter().filter(|d| d.3 == 254).count();
                        let freed = diffs.iter().filter(|d| d.3 == 0).count();
                        println!(
                            "proto [{DIFF_KEY}]: {} changed cells ({lethal} -> lethal, {freed} -> free)",
                            diffs.len()
                        );
                    }
                    Err(error) => eprintln!("zenoh_test: dropping diff ({} bytes): {error}", payload.len()),
                }
            })
            .wait()
            .unwrap_or_else(|error| {
                eprintln!("zenoh_test: failed to subscribe to {DIFF_KEY}: {error}");
                std::process::exit(1)
            });
        (range_graph, diff)
    });

    println!("zenoh_test: decoding {}", if proto { "protobuf (lidar.proto)" } else { "raw layouts" });
    println!(
        "zenoh_test: subscribed to {POINTS_KEY}, {LOCAL_OCC_KEY}, {GLOBAL_OCC_KEY} and {POSE_KEY}, \
         waiting for frames... (C show/hide costmap, V local/global)"
    );

    App::new()
        .add_plugins(DefaultPlugins.build().disable::<bevy::audio::AudioPlugin>())
        .add_plugins(PointsPlugin)
        .insert_resource(CloudChannel(Mutex::new(viz_rx)))
        .insert_resource(CostmapChannel(Mutex::new(costmap_rx)))
        .insert_resource(PoseChannel(Mutex::new(pose_rx)))
        .insert_resource(OrbitCamera::default())
        .insert_resource(SegParamsHandle(Arc::new(Mutex::new(seg_params))))
        .insert_resource(LidarMountHeight(mount_height))
        .insert_resource(SegParamsPublisher(session.clone()))
        .init_resource::<CloudColorMode>()
        .init_resource::<ShowCostmap>()
        .init_resource::<CostmapView>()
        .init_resource::<LidarPose>()
        .add_systems(Startup, setup_scene)
        .add_systems(Update, (toggle_color_mode, toggle_costmap, tune_seg_params, drain_latest_cloud, drain_latest_costmap.after(drain_latest_pose), drain_latest_pose, orbit_camera, draw_origin_axes, draw_vehicle))
        .run();
}
