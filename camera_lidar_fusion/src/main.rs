//! Very basic camera-LiDAR fusion for already time-synchronized frames.
//!
//! Given one camera image, one LiDAR point cloud, and the known
//! LiDAR -> image transform for a frame, this projects every point into
//! the image plane and writes, per frame:
//!   - `<id>_rgb.png`   : the camera image, unmodified
//!   - `<id>_depth.png` : 16-bit depth map in the camera's FOV (KITTI
//!                        convention: pixel value / 256.0 = depth in metres,
//!                        0 = no LiDAR return)
//!   - `<id>_rgbd.bin`  : the two combined as a flat f32 array
//!                        [height, width, 4] (R, G, B in 0..1, depth in
//!                        metres), prefixed with a 3 x u32 LE shape header.
//!   - `<id>_depth_gray.png`     : normalized 8-bit grayscale visualization
//!                                 (closer = brighter).
//!   - `<id>_depth_heatmap.png`  : color visualization, closer = red,
//!                                 farther = blue.
//!
//! The two visualizations lightly dilate each point to a small disc so
//! sparse returns are visible at a glance; `<id>_depth.png` and
//! `<id>_rgbd.bin` stay pixel-exact (no dilation).
//!
//! No undistortion, rectification, or interpolation: this assumes the
//! supplied transform already maps raw LiDAR points straight to rectified
//! image pixels (true for the KITTI `lidar2img` matrices this was built
//! against). Depth is a single nearest-point z-buffer per pixel.

use image::{ImageBuffer, Luma, Rgb};
use std::fs;
use std::path::{Path, PathBuf};

type Mat4 = [[f32; 4]; 4];

/// Radius (pixels) each point is dilated to in the visualization-only
/// outputs, so sparse LiDAR returns are visible against the image.
const VIS_DILATE_RADIUS: i32 = 2;

/// A frame is one synchronized (image, point cloud, transform) triple.
struct Frame {
    id: String,
    image_path: PathBuf,
    points_path: PathBuf,
    calib_path: PathBuf,
}

fn discover_frames(data_dir: &Path) -> Vec<Frame> {
    let mut frames = Vec::new();
    let Ok(entries) = fs::read_dir(data_dir) else {
        return frames;
    };
    let mut ids: Vec<String> = entries
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let path = e.path();
            if path.extension().and_then(|s| s.to_str()) == Some("bin") {
                path.file_stem().map(|s| s.to_string_lossy().into_owned())
            } else {
                None
            }
        })
        .collect();
    ids.sort();
    for id in ids {
        let image_path = data_dir.join(format!("{id}.png"));
        let points_path = data_dir.join(format!("{id}.bin"));
        let calib_path = data_dir.join(format!("{id}_lidar2img.txt"));
        if image_path.exists() && calib_path.exists() {
            frames.push(Frame { id, image_path, points_path, calib_path });
        } else {
            eprintln!("skipping {id}: missing image or calibration file next to it");
        }
    }
    frames
}

fn load_calib(path: &Path) -> Mat4 {
    let text = fs::read_to_string(path).expect("read calibration file");
    let values: Vec<f32> = text
        .split_whitespace()
        .map(|t| t.parse().expect("calibration file must contain 16 numbers"))
        .collect();
    assert_eq!(values.len(), 16, "expected a flattened 4x4 lidar->image matrix");
    let mut m = [[0.0f32; 4]; 4];
    for row in 0..4 {
        for col in 0..4 {
            m[row][col] = values[row * 4 + col];
        }
    }
    m
}

/// Load a KITTI-format Velodyne scan: flat float32 (x, y, z, reflectance).
fn load_points(path: &Path) -> Vec<[f32; 3]> {
    let bytes = fs::read(path).expect("read point cloud file");
    assert_eq!(bytes.len() % 16, 0, "point file size must be a multiple of 16 bytes (x,y,z,reflectance as f32)");
    bytes
        .chunks_exact(16)
        .map(|c| {
            let x = f32::from_le_bytes(c[0..4].try_into().unwrap());
            let y = f32::from_le_bytes(c[4..8].try_into().unwrap());
            let z = f32::from_le_bytes(c[8..12].try_into().unwrap());
            [x, y, z]
        })
        .collect()
}

/// Project one LiDAR point through the combined lidar->image matrix.
/// Returns `(u, v, depth)` in pixel coordinates and metres, or `None` if
/// the point projects behind the camera.
fn project(m: &Mat4, p: [f32; 3]) -> Option<(f32, f32, f32)> {
    let [x, y, z] = p;
    let row = |r: usize| m[r][0] * x + m[r][1] * y + m[r][2] * z + m[r][3];
    let depth = row(2);
    if depth <= 0.0 {
        return None;
    }
    let u = row(0) / depth;
    let v = row(1) / depth;
    Some((u, v, depth))
}

/// Build a sparse depth map by projecting every point and keeping, per
/// pixel, the nearest return (a simple z-buffer).
fn build_depth_map(points: &[[f32; 3]], calib: &Mat4, width: u32, height: u32) -> (Vec<f32>, usize) {
    let mut depth_map = vec![0.0f32; (width * height) as usize];
    let mut hits = 0usize;
    for &p in points {
        let Some((u, v, depth)) = project(calib, p) else { continue };
        if u < 0.0 || v < 0.0 || u >= width as f32 || v >= height as f32 {
            continue;
        }
        let (ui, vi) = (u as u32, v as u32);
        let idx = (vi * width + ui) as usize;
        if depth_map[idx] == 0.0 || depth < depth_map[idx] {
            depth_map[idx] = depth;
        }
        hits += 1;
    }
    (depth_map, hits)
}

/// Grow each non-zero depth sample to a small disc (nearest value wins on
/// overlap), purely so single-pixel LiDAR returns are visible in a
/// human-facing render.
fn dilate(depth_map: &[f32], width: u32, height: u32, radius: i32) -> Vec<f32> {
    let (w, h) = (width as i32, height as i32);
    let mut out = depth_map.to_vec();
    for v in 0..h {
        for u in 0..w {
            let d = depth_map[(v * w + u) as usize];
            if d <= 0.0 {
                continue;
            }
            for dv in -radius..=radius {
                for du in -radius..=radius {
                    let (nu, nv) = (u + du, v + dv);
                    if nu < 0 || nv < 0 || nu >= w || nv >= h {
                        continue;
                    }
                    let nidx = (nv * w + nu) as usize;
                    if out[nidx] <= 0.0 || d < out[nidx] {
                        out[nidx] = d;
                    }
                }
            }
        }
    }
    out
}

/// Normalized grayscale value for one depth sample: closer is brighter.
/// Valid depths map to `40..=255` so the farthest return stays visibly
/// distinct from "no return" (which is `0`).
fn gray_value(d: f32, min: f32, max: f32) -> u8 {
    let t = if max > min { ((d - min) / (max - min)).clamp(0.0, 1.0) } else { 0.0 };
    (255.0 - t * 215.0).round() as u8
}

/// Closer-is-red, farther-is-blue color for one depth sample: a 5-stop
/// red -> yellow -> green -> cyan -> blue gradient (a reversed "jet"),
/// which keeps the requested red/blue endpoints but reads much better
/// than a flat two-color lerp (which would wash out to grey/purple in
/// the middle of the range).
fn heatmap_color(d: f32, min: f32, max: f32) -> Rgb<u8> {
    const STOPS: [(f32, [u8; 3]); 5] = [
        (0.00, [255, 0, 0]),
        (0.25, [255, 255, 0]),
        (0.50, [0, 255, 0]),
        (0.75, [0, 255, 255]),
        (1.00, [0, 0, 255]),
    ];
    let t = if max > min { ((d - min) / (max - min)).clamp(0.0, 1.0) } else { 0.0 };
    for i in 0..STOPS.len() - 1 {
        let (t0, c0) = STOPS[i];
        let (t1, c1) = STOPS[i + 1];
        if t <= t1 {
            let f = (t - t0) / (t1 - t0);
            let lerp = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * f).round() as u8;
            return Rgb([lerp(c0[0], c1[0]), lerp(c0[1], c1[1]), lerp(c0[2], c1[2])]);
        }
    }
    Rgb(STOPS[STOPS.len() - 1].1)
}

fn write_depth_visualizations(
    gray_path: &Path,
    heatmap_path: &Path,
    depth_map: &[f32],
    min: f32,
    max: f32,
    width: u32,
    height: u32,
) {
    let vis = dilate(depth_map, width, height, VIS_DILATE_RADIUS);

    let mut gray: ImageBuffer<Luma<u8>, Vec<u8>> = ImageBuffer::new(width, height);
    for (i, px) in gray.pixels_mut().enumerate() {
        *px = Luma([if vis[i] > 0.0 { gray_value(vis[i], min, max) } else { 0 }]);
    }
    gray.save(gray_path).expect("write depth grayscale png");

    let mut heatmap: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(width, height);
    for (i, px) in heatmap.pixels_mut().enumerate() {
        *px = if vis[i] > 0.0 { heatmap_color(vis[i], min, max) } else { Rgb([0, 0, 0]) };
    }
    heatmap.save(heatmap_path).expect("write depth heatmap png");
}

fn write_depth_png(path: &Path, depth_map: &[f32], width: u32, height: u32) {
    let mut img: ImageBuffer<Luma<u16>, Vec<u16>> = ImageBuffer::new(width, height);
    for (i, px) in img.pixels_mut().enumerate() {
        // KITTI depth-completion convention: stored value / 256 = metres.
        let scaled = (depth_map[i] * 256.0).round().clamp(0.0, u16::MAX as f32);
        *px = Luma([scaled as u16]);
    }
    img.save(path).expect("write depth png");
}

fn write_rgbd_bin(path: &Path, rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>, depth_map: &[f32], width: u32, height: u32) {
    let mut out = Vec::with_capacity(12 + (width * height * 4) as usize * 4);
    out.extend_from_slice(&height.to_le_bytes());
    out.extend_from_slice(&width.to_le_bytes());
    out.extend_from_slice(&4u32.to_le_bytes());
    for (i, px) in rgb.pixels().enumerate() {
        out.extend_from_slice(&(px[0] as f32 / 255.0).to_le_bytes());
        out.extend_from_slice(&(px[1] as f32 / 255.0).to_le_bytes());
        out.extend_from_slice(&(px[2] as f32 / 255.0).to_le_bytes());
        out.extend_from_slice(&depth_map[i].to_le_bytes());
    }
    fs::write(path, out).expect("write rgbd.bin");
}

fn main() {
    let data_dir = Path::new("data");
    let out_dir = Path::new("out");
    fs::create_dir_all(out_dir).expect("create out/ directory");

    let frames = discover_frames(data_dir);
    if frames.is_empty() {
        eprintln!("no frames found in {}", data_dir.display());
        std::process::exit(1);
    }

    for frame in frames {
        let rgb = image::open(&frame.image_path)
            .unwrap_or_else(|e| panic!("failed to open {}: {e}", frame.image_path.display()))
            .to_rgb8();
        let (width, height) = rgb.dimensions();
        let calib = load_calib(&frame.calib_path);
        let points = load_points(&frame.points_path);
        let (depth_map, hits) = build_depth_map(&points, &calib, width, height);

        let depths: Vec<f32> = depth_map.iter().copied().filter(|&d| d > 0.0).collect();
        let (min, max) = depths
            .iter()
            .fold((f32::MAX, f32::MIN), |(mn, mx), &d| (mn.min(d), mx.max(d)));

        rgb.save(out_dir.join(format!("{}_rgb.png", frame.id)))
            .expect("write rgb png");
        write_depth_png(&out_dir.join(format!("{}_depth.png", frame.id)), &depth_map, width, height);
        write_rgbd_bin(&out_dir.join(format!("{}_rgbd.bin", frame.id)), &rgb, &depth_map, width, height);
        write_depth_visualizations(
            &out_dir.join(format!("{}_depth_gray.png", frame.id)),
            &out_dir.join(format!("{}_depth_heatmap.png", frame.id)),
            &depth_map,
            min,
            max,
            width,
            height,
        );
        println!(
            "{}: {}x{}, {} points, {} in camera FOV ({} unique pixels hit), depth {:.2}-{:.2} m",
            frame.id,
            width,
            height,
            points.len(),
            hits,
            depths.len(),
            min,
            max
        );
    }
}
