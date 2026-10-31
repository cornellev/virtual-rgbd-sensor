# camera_lidar_fusion

Very basic LiDAR -> camera fusion for already time-synchronized frames with
a known transform. No undistortion, rectification, interpolation, or
occlusion reasoning beyond a single nearest-point z-buffer per pixel.

Input is deliberately plain so it's easy to swap in your own frames:
each frame is one `<id>.png` (camera image), one `<id>.bin` (flat
float32 LiDAR scan, `x, y, z, reflectance` per point, KITTI Velodyne
layout), and one `<id>_lidar2img.txt` (16 whitespace-separated numbers,
the combined `Tr_velo_to_cam` + `R0_rect` + camera projection matrix as a
row-major 4x4). The crate scans `data/` for every `<id>.bin` and processes
it if a matching image and transform exist.

The one bundled frame (`data/000008.*`) is a real KITTI object-detection
sample frame (redistributed via mmdetection3d's demo data), used here just
to exercise the pipeline end to end.

## Running

```sh
cargo run --release
```

Per frame, writes to `out/`:

- `<id>_rgb.png` - the camera image, unmodified.
- `<id>_depth.png` - 16-bit grayscale depth map at the camera's resolution,
  restricted to the camera's FOV. KITTI depth-completion convention:
  `depth_metres = pixel_value / 256`, `0` = no LiDAR return at that pixel.
- `<id>_rgbd.bin` - the two combined as one flat array: a 12-byte header
  (`height: u32`, `width: u32`, `channels: u32`, little-endian) followed by
  `height * width * channels` `f32` values, row-major, channels
  `[R, G, B, depth_m]` with R/G/B in `0..1`.
- `<id>_depth_gray.png` - 8-bit grayscale visualization, normalized to the
  frame's own min/max depth. Closer is brighter; valid depths map to
  `40..255` so the farthest return stays visibly distinct from "no
  return" (`0`, pure black).
- `<id>_depth_heatmap.png` - color visualization: closer is red, farther
  is blue, running through yellow/green/cyan in between (a reversed "jet"
  gradient, normalized the same way). "No return" pixels are black.

The two visualizations dilate each point to a `2`-pixel-radius disc
(nearest value wins where discs overlap) purely so sparse single-pixel
LiDAR returns are visible at a glance. `<id>_depth.png` and
`<id>_rgbd.bin` are not dilated and stay pixel-exact.

Load the combined file in Python:

```python
import numpy as np
raw = open("out/000008_rgbd.bin", "rb").read()
h, w, c = np.frombuffer(raw[:12], dtype="<u4")
rgbd = np.frombuffer(raw[12:], dtype="<f4").reshape(h, w, c)
```

## Projection

For each point `p = (x, y, z)`, homogeneous-transform by the frame's
`lidar2img` matrix `M` and perspective-divide:

```
[u*d, v*d, d, 1]^T = M @ [x, y, z, 1]^T
```

A point is kept if `d > 0` (in front of the camera) and `(u, v)` lands
inside the image. When more than one point projects to the same pixel,
the nearer one wins.
