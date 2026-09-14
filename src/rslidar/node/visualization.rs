// Bevy visualizer: renders the point clouds `run_decoder` (in the parent
// module) produces. Nothing in here knows about MSOP/DIFOP, UDP, or pcap --
// it only consumes `VizPoint`s handed over the `CloudChannel`. The one
// exception is `SegParamsHandle`/`tune_seg_params`: those touch
// `segmentation::SegParams` so the thresholds can be dialed in live from the
// keyboard while watching the cluster coloring update.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy_points::prelude::*;
use bevy_points::material::PointsShaderSettings;

use crate::segmentation::SegParams;

use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;

/// A single decoded point, stripped of everything ROS-specific -- just
/// enough to hand off to Bevy for rendering. `cluster` is the segmentation
/// output (`RangeNode::beta` from segmentation.rs's second_segmentation) --
/// `-1` means the point never ended up representing any range-graph cell
/// this revolution (see the note on `drain_latest_cloud`'s coloring below).
#[derive(Clone, Copy)]
pub struct VizPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub cluster: i32,
}

/// Wraps the receiving end of the decoder -> visualizer channel as a Bevy
/// resource. `mpsc::Receiver` isn't `Sync`, so it's wrapped in a `Mutex`;
/// only `drain_latest_cloud` ever touches it.
#[derive(Resource)]
pub struct CloudChannel(pub Mutex<mpsc::Receiver<Vec<VizPoint>>>);

/// Marks the one entity that holds the current point-cloud mesh, so
/// `drain_latest_cloud` knows what to swap out each time a new frame lands.
/// Public only because it appears in `drain_latest_cloud`'s signature (a
/// `Query` type parameter) -- nothing outside this module constructs one.
#[derive(Component)]
pub struct PointCloudEntity;

pub fn setup_scene(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<PointsMaterial>>) {
    // Initial pose only -- `orbit_camera` recomputes this every frame from
    // the `OrbitCamera` resource, which starts at this same position.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // One entity holds the whole point cloud; its mesh gets replaced every
    // time a new frame arrives.
    commands.spawn((
        Mesh3d(meshes.add(PointsMesh::from_iter(std::iter::empty::<Vec3>()))),
        MeshMaterial3d(materials.add(PointsMaterial {
            settings: PointsShaderSettings {
                point_size: 0.01,
                // White so it doesn't tint the per-vertex distance colors
                // `drain_latest_cloud` assigns (the shader multiplies this
                // uniform color by each point's vertex color).
                color: Color::WHITE.into(),
                ..default()
            },
            perspective: true,
            circle: true,
            ..default()
        })),
        PointCloudEntity,
    ));
}

const AXIS_LENGTH: f32 = 1.0;

/// Draws an RGB (X/Y/Z) axis triad at the LiDAR origin every frame. Gizmos
/// are immediate-mode, so this has to run each frame rather than spawning a
/// one-off entity in `setup_scene`.
pub fn draw_origin_axes(mut gizmos: Gizmos) {
    gizmos.arrow(Vec3::ZERO, Vec3::X * AXIS_LENGTH, Color::srgb(1.0, 0.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::Y * AXIS_LENGTH, Color::srgb(0.0, 1.0, 0.0));
    gizmos.arrow(Vec3::ZERO, Vec3::Z * AXIS_LENGTH, Color::srgb(0.0, 0.4, 1.0));
}

/// Pulls the most recently completed cloud off the channel (dropping any
/// older ones that piled up while the app was busy rendering) and rebuilds
/// the point mesh from it.
pub fn drain_latest_cloud(
    channel: Res<CloudChannel>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut query: Query<&mut Mesh3d, With<PointCloudEntity>>,
) {
    let latest = {
        let rx = channel.0.lock().unwrap();
        let mut latest = None;
        while let Ok(cloud) = rx.try_recv() {
            latest = Some(cloud);
        }
        latest
    };

    let Some(cloud) = latest else { return };
    let Ok(mut mesh3d) = query.single_mut() else { return };

    // The decoder emits points in the LiDAR/ROS convention (REP-103:
    // x-forward, y-left, z-up), but Bevy is Y-up (+Y up, -Z forward). Passing
    // (x, y, z) straight through renders LiDAR-up as depth, which looks like
    // the whole cloud is tipped onto its side. Remap axes instead: Bevy's Y
    // becomes LiDAR's z (up stays up), and Bevy's Z becomes -LiDAR's y (a
    // proper rotation, not a mirror, so left/right stay consistent).
    let vertices: Vec<Vec3> = cloud.iter().map(|p| Vec3::new(p.x, p.z, -p.y)).collect();

    // Golden-angle hue spread: successive cluster ids land ~137.5deg apart
    // in hue, which (unlike e.g. `id * 30`) never lines up into a short
    // repeating cycle of similar colors even for many clusters, so adjacent
    // cluster ids still read as visually distinct. Points with no cluster
    // (-1) -- e.g. one of two points that landed in the same range-graph
    // cell this revolution, since RangeGraph::insert keeps only the closer
    // one, so the loser never entered the segmentation pipeline at all --
    // are colored a flat gray instead of a hue, so "unclustered" is visually
    // unambiguous rather than landing on some arbitrary hue by coincidence.
    const GOLDEN_ANGLE_DEG: f32 = 137.507_76;
    let colors: Vec<Color> = cloud
        .iter()
        .map(|p| {
            if p.cluster < 0 {
                Color::srgb(0.4, 0.4, 0.4)
            } else {
                let hue = (p.cluster as f32 * GOLDEN_ANGLE_DEG) % 360.0;
                Color::hsl(hue, 0.85, 0.55)
            }
        })
        .collect();

    let mut points_mesh = PointsMesh::from_iter(vertices);
    points_mesh.colors = Some(colors);
    mesh3d.0 = meshes.add(points_mesh);
}

/// Orbit-camera state: the camera always looks at `target` from `distance`
/// away, at the given `yaw`/`pitch` around it (spherical coordinates).
#[derive(Resource)]
pub struct OrbitCamera {
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        // Matches the camera's original fixed start position of (5, 5, 5)
        // looking at the origin, just expressed in spherical terms.
        Self {
            target: Vec3::ZERO,
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: (1.0f32 / 3.0f32.sqrt()).asin(),
            distance: 75.0f32.sqrt(),
        }
    }
}

const ORBIT_SENSITIVITY: f32 = 0.005;
const ZOOM_SENSITIVITY: f32 = 0.5;
const MIN_DISTANCE: f32 = 0.5;
const MAX_DISTANCE: f32 = 500.0;
const PITCH_LIMIT: f32 = 1.5; // radians; just short of straight up/down to avoid a gimbal flip

/// Orbits the camera around `OrbitCamera::target` (the LiDAR origin by
/// default): left-drag (or the arrow keys) rotates around it, the scroll
/// wheel zooms, and WASD/QE re-center the target so you're not stuck
/// orbiting one fixed point forever.
pub fn orbit_camera(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut orbit: ResMut<OrbitCamera>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut transform) = query.single_mut() else { return };
    let dt = time.delta_secs();

    if mouse_buttons.pressed(MouseButton::Left) {
        orbit.yaw -= mouse_motion.delta.x * ORBIT_SENSITIVITY;
        orbit.pitch = (orbit.pitch - mouse_motion.delta.y * ORBIT_SENSITIVITY)
            .clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }
    if keys.pressed(KeyCode::ArrowLeft) { orbit.yaw += 1.5 * dt; }
    if keys.pressed(KeyCode::ArrowRight) { orbit.yaw -= 1.5 * dt; }
    if keys.pressed(KeyCode::ArrowUp) {
        orbit.pitch = (orbit.pitch + 1.0 * dt).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }
    if keys.pressed(KeyCode::ArrowDown) {
        orbit.pitch = (orbit.pitch - 1.0 * dt).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    orbit.distance = (orbit.distance - mouse_scroll.delta.y * ZOOM_SENSITIVITY)
        .clamp(MIN_DISTANCE, MAX_DISTANCE);

    // Pan along the view's flattened (yaw-only) basis so WASD/QE re-centers
    // the orbit target without also fighting the current pitch.
    let yaw_rot = Quat::from_rotation_y(orbit.yaw);
    let forward_flat = yaw_rot * Vec3::NEG_Z;
    let right_flat = yaw_rot * Vec3::X;
    let mut pan = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) { pan += forward_flat; }
    if keys.pressed(KeyCode::KeyS) { pan -= forward_flat; }
    if keys.pressed(KeyCode::KeyA) { pan -= right_flat; }
    if keys.pressed(KeyCode::KeyD) { pan += right_flat; }
    if keys.pressed(KeyCode::KeyE) { pan += Vec3::Y; }
    if keys.pressed(KeyCode::KeyQ) { pan -= Vec3::Y; }
    let pan_speed = orbit.distance.max(1.0) * 0.5;
    orbit.target += pan * pan_speed * dt;

    let dir = Vec3::new(
        orbit.pitch.cos() * orbit.yaw.sin(),
        orbit.pitch.sin(),
        orbit.pitch.cos() * orbit.yaw.cos(),
    );
    transform.translation = orbit.target + dir * orbit.distance;
    transform.look_at(orbit.target, Vec3::Y);
}

/// Shared, thread-safe handle to `first_segmentation`'s thresholds. The
/// decoder thread reads it fresh at the start of every revolution (see the
/// `decode_msop` frame-boundary block in rslidar_sdk_node.rs), so a value
/// changed here shows up in the clustering within about one revolution --
/// no restart needed.
#[derive(Resource, Clone)]
pub struct SegParamsHandle(pub Arc<Mutex<SegParams>>);

/// Dials the segmentation thresholds up or down with the keyboard so you can
/// watch clustering change live instead of editing code and rebuilding:
/// '['/']' step first_segmentation's `th_d` (scale on the expected same-ring
/// gap), ';'/''' step its `th_z` (degrees), '-'/'=' step second_segmentation's
/// `th_d_second` (scale on the expected cross-ring gap), ','/'.' step
/// `k_deg` (extra per-degree scale on top of `th_d_second`), '`'/'\' step
/// `z_weight` (how much a dz contributes to node_dist relative to dx/dy),
/// '9'/'0' step `min_cluster_points` (clusters smaller than this get
/// discarded as noise instead of a real color), 'n'/'m' step `min_gap_m`
/// (the absolute floor under both thresholds, so close-range comparisons
/// don't get a smaller window than the sensor's actual noise floor -- it
/// also doubles as the fraction used in node_dist's dz-gate, see its doc).
/// Each tap is one step (not held-repeat), so nudging is deliberate rather
/// than racing past the value you wanted.
pub fn tune_seg_params(keys: Res<ButtonInput<KeyCode>>, handle: Res<SegParamsHandle>) {
    const TH_D_STEP: f32 = 0.5;
    const TH_Z_STEP_DEG: f32 = 0.5;
    const TH_D_SECOND_STEP: f32 = 0.01;
    const K_DEG_STEP: f32 = 0.05;
    const Z_WEIGHT_STEP: f32 = 0.05;
    const MIN_CLUSTER_STEP: u32 = 1;
    const MIN_GAP_STEP: f32 = 0.01;

    let mut params = handle.0.lock().unwrap();
    let mut changed = false;

    if keys.just_pressed(KeyCode::BracketLeft) {
        params.th_d = (params.th_d - TH_D_STEP).max(0.01);
        changed = true;
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        params.th_d += TH_D_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Semicolon) {
        params.th_z_deg = (params.th_z_deg - TH_Z_STEP_DEG).max(0.05);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Quote) {
        params.th_z_deg += TH_Z_STEP_DEG;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Minus) {
        params.th_d_second = (params.th_d_second - TH_D_SECOND_STEP).max(0.01);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Equal) {
        params.th_d_second += TH_D_SECOND_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Comma) {
        params.k_deg = (params.k_deg - K_DEG_STEP).max(0.0);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Period) {
        params.k_deg += K_DEG_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Backquote) {
        params.z_weight -= Z_WEIGHT_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Backslash) {
        params.z_weight += Z_WEIGHT_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Digit9) {
        params.min_cluster_points = params.min_cluster_points.saturating_sub(MIN_CLUSTER_STEP);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Digit0) {
        params.min_cluster_points += MIN_CLUSTER_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::KeyN) {
        params.min_gap_m = (params.min_gap_m - MIN_GAP_STEP).max(0.0);
        changed = true;
    }
    if keys.just_pressed(KeyCode::KeyM) {
        params.min_gap_m += MIN_GAP_STEP;
        changed = true;
    }
    if changed {
        println!(
            "rslidar: th_d={:.3}m th_z={:.2}deg th_d_second={:.3}m k_deg={:.3} z_weight={:.3} \
             min_cluster_points={} min_gap_m={:.3}  \
             ('[' ']' th_d, ';' ''' th_z, '-' '=' th_d_second, ',' '.' k_deg, '`' '\\' z_weight, \
             '9' '0' min_cluster_points, 'n' 'm' min_gap_m)",
            params.th_d, params.th_z_deg, params.th_d_second, params.k_deg, params.z_weight,
            params.min_cluster_points, params.min_gap_m
        );
    }
}
