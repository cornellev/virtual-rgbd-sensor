// like the lightweight bevy visualizer for lidar related outputs, expected that
// these outputs will eventually be visualized in the sim and autonomy dash

use bevy::asset::RenderAssetUsages;
use bevy::image::ImageSampler;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy_points::prelude::*;
use bevy_points::material::PointsShaderSettings;

use crate::segmentation::{SEG_PARAMS_KEY, SegParams};
use zenoh::Wait;

use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;

pub const NO_INFORMATION: usize = 255;
pub const LETHAL_OBSTACLE: usize = 254;
pub const INSCRIBED_INFLATED_OBSTACLE: usize = 253;
pub const FREE_SPACE: usize = 0;

/// one clustered point from the point cloud
#[derive(Clone, Copy)]
pub struct VizPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub cluster: i32,
    pub intensity: f32
}

/// Wraps the receiving end of the decoder -> visualizer channel as a Bevy
/// resource. `mpsc::Receiver` isn't `Sync`, so it's wrapped in a `Mutex`;
/// only `drain_latest_cloud` ever touches it.
#[derive(Resource)]
pub struct CloudChannel(pub Mutex<mpsc::Receiver<Vec<VizPoint>>>);

#[derive(Component)]
pub struct PointCloudEntity;

/// Wraps the receiving end of the pose (`rslidar/pose_frame`) channel, same
/// pattern as `CloudChannel`/`CostmapChannel`.
#[derive(Resource)]
pub struct PoseChannel(pub Mutex<mpsc::Receiver<(Vec3, Quat)>>);

/// The LiDAR's current world-frame pose (already lidar -> Bevy axis
/// remapped by `decode_pose` in main.rs), read by `draw_vehicle` instead of
/// assuming the LiDAR sits fixed at the scene origin. Defaults to the
/// origin/identity, matching the old fixed-at-origin behavior until the
/// first pose sample arrives.
#[derive(Resource, Default)]
pub struct LidarPose {
    pub translation: Vec3,
    pub rotation: Quat,
}

/// Pulls the newest pose off the channel (dropping any older ones), same
/// drain-to-latest pattern as `drain_latest_cloud`/`drain_latest_costmap`.
pub fn drain_latest_pose(channel: Res<PoseChannel>, mut pose: ResMut<LidarPose>) {
    let latest = {
        let rx = channel.0.lock().unwrap();
        let mut latest = None;
        while let Ok(p) = rx.try_recv() {
            latest = Some(p);
        }
        latest
    };
    if let Some((translation, rotation)) = latest {
        pose.translation = translation;
        pose.rotation = rotation;
    }
}

// roll: about the LiDAR x (forward) axis, which is Bevy +X.
const CLOUD_ROLL_DEG: f32 = 0.0; //-10.0;
// pitch: about the LiDAR y (left) axis, which is Bevy -Z
const CLOUD_PITCH_DEG: f32 = 0.0; //-2.0;

/// Roll first, then pitch, both about the fixed (parent) axes. In a
/// quaternion product the right-hand rotation is applied first.
fn cloud_tilt() -> Quat {
    Quat::from_rotation_z(CLOUD_PITCH_DEG.to_radians())
        * Quat::from_rotation_x(CLOUD_ROLL_DEG.to_radians())
}

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
                point_size: 0.05,
                color: Color::WHITE.into(),
                ..default()
            },
            perspective: true,
            circle: true,
            ..default()
        })),
        Transform::from_rotation(cloud_tilt()),
        PointCloudEntity,
    ));
}

/// One costmap as published on `rslidar/costmap` (see `encode_costmap` in
/// rslidar_sdk_node.rs): row-major u8 costs, cell (0, 0) at
/// (`origin_x`, `origin_y`) in the LiDAR frame.
pub struct VizCostmap {
    pub size_x: usize,
    pub size_y: usize,
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub data: Vec<u8>,
}

#[derive(Resource)]
pub struct CostmapChannel(pub Mutex<mpsc::Receiver<VizCostmap>>);

/// The textured plane the costmap is drawn on. Spawned by
/// `drain_latest_costmap` on the first costmap, since the grid size isn't
/// known before then; the handles let it rewrite the texture in place after.
#[derive(Component)]
pub struct CostmapPlane {
    image: Handle<Image>,
    material: Handle<StandardMaterial>,
    size_x: usize,
    size_y: usize,
}

/// Height (Bevy Y, i.e. LiDAR z) the costmap plane is drawn at. Just under
/// the LiDAR by default; set it to minus the mount height to lay it on the
/// ground points instead.
const COSTMAP_PLANE_Y: f32 = -0.05;

/// Overall opacity of the costmap plane, like RViz's Map display "Alpha"
/// (0.7 by default).
const COSTMAP_ALPHA: u8 = 179;

/// sRGB RGBA for each cost value, matching RViz's "costmap" color scheme as
/// it shows a Nav2 costmap: free is transparent, the inflation falloff
/// (1..=252) runs blue -> red, inscribed (253) is cyan, lethal (254) magenta,
/// and unknown (255) the gray-green RViz uses for -1.
fn costmap_colors() -> [[u8; 4]; 256] {
    let mut lut = [[0u8; 4]; 256];
    for cost in 1..=252usize {
        // Nav2 publishes 1..=252 as occupancy 1..=98, and RViz colors
        // occupancy v as (255*v/100, 0, 255 - 255*v/100).
        let occupancy = 1 + (97 * (cost - 1)) / 251;
        let v = (255 * occupancy / 100) as u8;
        lut[cost] = [v, 0, 255 - v, COSTMAP_ALPHA];
    }

    lut[INSCRIBED_INFLATED_OBSTACLE] = [0, 255, 255, COSTMAP_ALPHA];
    lut[LETHAL_OBSTACLE] = [255, 0, 255, COSTMAP_ALPHA];
    // lut[NO_INFORMATION] = [0, 0, 0, COSTMAP_ALPHA];
    lut[FREE_SPACE] = [255, 255, 255, COSTMAP_ALPHA];
    lut
}

/// Pulls the newest costmap off the channel and repaints the plane's texture
/// with it, spawning (or resizing) the plane first if needed.
pub fn drain_latest_costmap(
    mut commands: Commands,
    channel: Res<CostmapChannel>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut query: Query<(Entity, &CostmapPlane, &mut Transform)>,
    show: Res<ShowCostmap>,
) {
    let latest = {
        let rx = channel.0.lock().unwrap();
        let mut latest = None;
        while let Ok(costmap) = rx.try_recv() {
            latest = Some(costmap);
        }
        latest
    };
    // Keep draining while hidden so the channel doesn't grow, but skip the repaint.
    if !show.0 {
        return;
    }
    let Some(costmap) = latest else { return };

    let width = costmap.size_x as f32 * costmap.resolution;
    let height = costmap.size_y as f32 * costmap.resolution;
    // Same (x, z, -y) LiDAR -> Bevy remap as the point cloud.
    let center = Vec3::new(
        costmap.origin_x + width / 2.0,
        COSTMAP_PLANE_Y,
        -(costmap.origin_y + height / 2.0),
    );

    let existing = query.single_mut().ok();
    let (image_handle, material_handle) = match existing {
        Some((_, plane, mut transform))
            if plane.size_x == costmap.size_x && plane.size_y == costmap.size_y =>
        {
            transform.translation = center;
            (plane.image.clone(), plane.material.clone())
        }
        other => {
            if let Some((entity, _, _)) = other {
                commands.entity(entity).despawn();
            }
            let mut image = Image::new_fill(
                Extent3d { width: costmap.size_x as u32, height: costmap.size_y as u32, depth_or_array_layers: 1 },
                TextureDimension::D2,
                &[0, 0, 0, 0],
                TextureFormat::Rgba8UnormSrgb,
                RenderAssetUsages::default(),
            );
            // Nearest, so each cell stays a crisp square instead of blurring
            // into its neighbors.
            image.sampler = ImageSampler::nearest();
            let image = images.add(image);
            let material = materials.add(StandardMaterial {
                base_color_texture: Some(image.clone()),
                unlit: true,
                alpha_mode: AlphaMode::Blend,
                cull_mode: None, // still visible when orbiting underneath
                ..default()
            });
            commands.spawn((
                Mesh3d(meshes.add(Plane3d::default().mesh().size(width, height))),
                MeshMaterial3d(material.clone()),
                Transform::from_translation(center),
                CostmapPlane { image: image.clone(), material: material.clone(), size_x: costmap.size_x, size_y: costmap.size_y },
            ));
            (image, material)
        }
    };

    let Some(image) = images.get_mut(&image_handle) else { return };
    let Some(pixels) = image.data.as_mut() else { return };
    // Plane3d's UVs run u along +X and v along +Z, so texture row 0 is the
    // -Z edge -- which, after the (x, z, -y) remap, is the costmap's
    // highest-y row. Hence the flip.
    let lut = costmap_colors();
    for (row, pixel_row) in pixels.chunks_exact_mut(costmap.size_x * 4).enumerate() {
        let my = costmap.size_y - 1 - row;
        let costs = &costmap.data[my * costmap.size_x..(my + 1) * costmap.size_x];
        for (pixel, &cost) in pixel_row.chunks_exact_mut(4).zip(costs) {
            pixel.copy_from_slice(&lut[cost as usize]);
        }
    }
    // Touch the material too so its bind group picks up the re-uploaded
    // texture; some Bevy versions don't notice an image change on its own.
    materials.get_mut(&material_handle);
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

const INCH: f32 = 0.0254;
/// Vehicle footprint and height (length along forward x, width, height).
const CAR_LENGTH: f32 = 43.0 * INCH;
const CAR_WIDTH: f32 = 22.5 * INCH;
const CAR_HEIGHT: f32 = 15.5 * INCH;
/// LiDAR mount in the car frame, metres: x forward, y up, z lateral (the same
/// axes as Bevy). Car frame origin is the centre of the car's footprint on the
/// ground.
const LIDAR_IN_CAR: Vec3 = Vec3::new(-0.355, 0.609, 0.0);

/// Draws a wireframe box for the car and marks the LiDAR, both placed at the
/// live `LidarPose` (world-frame, from `rslidar/pose_frame`) instead of
/// assuming the LiDAR sits fixed at the scene origin. `LIDAR_IN_CAR` is still
/// the car's rigid, unchanging offset from the LiDAR -- it's just rotated and
/// translated by the current pose now, rather than being the whole answer.
pub fn draw_vehicle(mut gizmos: Gizmos, pose: Res<LidarPose>) {
    let lidar_pos = pose.translation;
    let lidar_rot = pose.rotation;
    let car_origin = lidar_pos + lidar_rot * (-LIDAR_IN_CAR);
    let box_center = car_origin + Vec3::Y * (CAR_HEIGHT / 2.0);
    gizmos.cuboid(
        Transform::from_translation(box_center)
            .with_rotation(lidar_rot)
            .with_scale(Vec3::new(CAR_LENGTH, CAR_HEIGHT, CAR_WIDTH)),
        Color::srgb(1.0, 0.85, 0.2),
    );

    // LiDAR marker at its current world pose, plus a mast down to the car's footprint.
    let lidar_color = Color::srgb(1.0, 0.3, 0.9);
    gizmos.sphere(Isometry3d::from_translation(lidar_pos), 0.05, lidar_color);
    gizmos.line(lidar_pos, car_origin, lidar_color);
}

/// Whether the costmap plane is drawn; C toggles it (`toggle_costmap`).
#[derive(Resource)]
pub struct ShowCostmap(pub bool);

impl Default for ShowCostmap {
    fn default() -> Self {
        Self(true)
    }
}

pub fn toggle_costmap(
    keys: Res<ButtonInput<KeyCode>>,
    mut show: ResMut<ShowCostmap>,
    mut planes: Query<&mut Visibility, With<CostmapPlane>>,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        show.0 = !show.0;
        println!("visualization: costmap {}", if show.0 { "shown" } else { "hidden" });
    }
    let vis = if show.0 { Visibility::Inherited } else { Visibility::Hidden };
    for mut v in &mut planes {
        v.set_if_neq(vis);
    }
}

/// How `drain_latest_cloud` colors points; Space cycles it (`toggle_color_mode`).
#[derive(Resource, Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CloudColorMode {
    /// One hue per cluster, unclustered points gray.
    #[default]
    Cluster,
    /// Red near -> blue far, ignoring clusters.
    Range,
    Intensity
}

pub fn toggle_color_mode(keys: Res<ButtonInput<KeyCode>>, mut mode: ResMut<CloudColorMode>) {
    if keys.just_pressed(KeyCode::Space) {
        *mode = match *mode {
            CloudColorMode::Cluster => CloudColorMode::Range,
            CloudColorMode::Range => CloudColorMode::Intensity,
            CloudColorMode::Intensity => CloudColorMode::Cluster,
        };
        println!("visualization: color mode = {:?}", *mode);
    }
}

/// Ranges (meters from the LiDAR) the range gradient spans;
/// anything closer/farther clamps to the end colors.
const RANGE_NEAR_M: f32 = 0.5;
const RANGE_FAR_M: f32 = 20.0;

/// Red when close, sweeping through the hues to blue when far.
fn range_color(p: &VizPoint) -> Color {
    let range = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
    let t = ((range - RANGE_NEAR_M) / (RANGE_FAR_M - RANGE_NEAR_M)).clamp(0.0, 1.0);
    Color::hsl(240.0 * t, 0.9, 0.5)
}

fn intensity_color(p: &VizPoint) -> Color {
    let t = (p.intensity / 255.0).clamp(0.0, 1.0);
    Color::hsl(0.0, 0.0, 0.15 + 0.85 * t)
}

/// Pulls the most recently completed cloud off the channel (dropping any
/// older ones that piled up while the app was busy rendering) and rebuilds
/// the point mesh from it.
pub fn drain_latest_cloud(
    channel: Res<CloudChannel>,
    mode: Res<CloudColorMode>,
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

    // LiDAR frame is REP-103: (x-forward, y-left, z-up),
    // Bevy frame is: Y-up (+Y up, -Z forward)
    // need to transform points from LiDAR to Bevy or else lopsided.
    // `rslidar` publishes these sensor-local now (it has no pose awareness
    // at all), so the world-frame placement happens here, client-side, using
    // the same LidarPose draw_vehicle already reads -- otherwise the points
    // stay pinned at the Bevy origin while the vehicle correctly drives
    // around, and the two drift apart exactly as they were doing.
    let vertices: Vec<Vec3> = cloud
        .iter()
        .map(|p| Vec3::new(p.x, p.z, -p.y))
        .collect();

    const GOLDEN_ANGLE_DEG: f32 = 137.507_76;
    let colors: Vec<Color> = cloud
        .iter()
        .map(|p| {
            if *mode == CloudColorMode::Intensity {
                intensity_color(p)
            } else if *mode == CloudColorMode::Range {
                range_color(p)
            } else if p.cluster < 0 {
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

/// The visualizer's copy of the segmentation thresholds. Every change is
/// published on `SEG_PARAMS_KEY`; zenoh_wrapper copies it into the decoder's
/// params, which are reread every revolution, so it shows up in the
/// clustering within about one revolution -- no restart needed.
#[derive(Resource, Clone)]
pub struct SegParamsHandle(pub Arc<Mutex<SegParams>>);

#[derive(Resource)]
pub struct SegParamsPublisher(pub zenoh::Session);

// this is like the scary parameter tuning stuff, delete later, very ew
pub fn tune_seg_params(
    keys: Res<ButtonInput<KeyCode>>,
    handle: Res<SegParamsHandle>,
    publisher: Res<SegParamsPublisher>,
) {
    const TH_D_STEP: f32 = 0.5;
    const TH_Z_STEP_DEG: f32 = 0.5;
    const TH_V_BASE_STEP: f32 = 0.01;
    const TH_V_SLOPE_STEP: f32 = 0.002;
    const Z_WEIGHT_STEP: f32 = 0.05;
    const MIN_CLUSTER_STEP: u32 = 1;
    const MIN_GAP_STEP: f32 = 0.01;
    const MIN_HEIGHT_STEP: f32 = 0.05;

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
        params.th_v_base = (params.th_v_base - TH_V_BASE_STEP).max(0.01);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Equal) {
        params.th_v_base += TH_V_BASE_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::Comma) {
        params.th_v_slope = (params.th_v_slope - TH_V_SLOPE_STEP).max(0.0);
        changed = true;
    }
    if keys.just_pressed(KeyCode::Period) {
        params.th_v_slope += TH_V_SLOPE_STEP;
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
    if keys.just_pressed(KeyCode::KeyG) {
        params.seg_enabled = !params.seg_enabled;
        changed = true;
    }
    if keys.just_pressed(KeyCode::KeyH) {
        params.height_filter_enabled = !params.height_filter_enabled;
        changed = true;
    }
    if keys.just_pressed(KeyCode::KeyJ) {
        params.min_height -= MIN_HEIGHT_STEP;
        changed = true;
    }
    if keys.just_pressed(KeyCode::KeyK) {
        params.min_height += MIN_HEIGHT_STEP;
        changed = true;
    }
    if changed {
        println!(
            "rslidar: seg_enabled={} height_filter_enabled={} min_height={:.2}m th_d={:.3}m th_z={:.2}deg \
             th_v_base={:.3}m th_v_slope={:.3} z_weight={:.3} min_cluster_points={} min_gap_m={:.3}  \
             ('g' seg_enabled, 'h' height_filter_enabled, 'j' 'k' min_height, '[' ']' th_d, ';' ''' th_z, \
             '-' '=' th_v_base, ',' '.' th_v_slope, '`' '\\' z_weight, '9' '0' min_cluster_points, 'n' 'm' min_gap_m)",
            params.seg_enabled, params.height_filter_enabled, params.min_height,
            params.th_d, params.th_z_deg, params.th_v_base, params.th_v_slope, params.z_weight,
            params.min_cluster_points, params.min_gap_m
        );
        if let Err(error) = publisher.0.put(SEG_PARAMS_KEY, params.to_bytes()).wait() {
            eprintln!("visualization: publish seg params: {error}");
        }
    }
}
