use crate::segmentation::{RangeGraph, NUM_COLS, NUM_RINGS};

/// Histogram-equalization lookup table: maps each value in `0..bins` to `1..=255`
/// by its rank among the nonzero `values` (0 is treated as "no data" and ignored).
fn equalize(values: impl Iterator<Item = usize>, bins: usize) -> Vec<u8> {
    let mut hist = vec![0usize; bins];
    for v in values.filter(|&v| v > 0) { hist[v] += 1; }
    let total = hist.iter().sum::<usize>().max(1);
    let mut acc = 0;
    hist.iter().map(|&count| {
        acc += count;
        (1 + 254 * acc / total) as u8
    }).collect()
}

// range iamges yay
pub fn write_images(range_graph: &RangeGraph, dir: &std::path::Path, frame: u64,
                    col_start: usize, col_end: usize) -> image::ImageResult<()> {
    use image::{GrayImage, ImageBuffer, Luma, imageops::{self, FilterType}};
    const PREVIEW_ROW_SCALE: u32 = 8;

    let width = (col_end + NUM_COLS - col_start) % NUM_COLS;
    let width = if width == 0 { NUM_COLS } else { width };
    let (w, h) = (width as u32, NUM_RINGS as u32);
    let mut range_img: ImageBuffer<Luma<u16>, Vec<u16>> = ImageBuffer::new(w, h);
    let mut inten_img = GrayImage::new(w, h);

    // top row = highest beam; columns flipped so left->right matches a camera
    for (row, ring) in (0..NUM_RINGS).rev().enumerate() {
        for c in 0..width {
            let n = &range_graph.nodes[ring * NUM_COLS + (col_start + width - 1 - c) % NUM_COLS];
            if n.valid {
                let r_mm = (n.range * 1000.0).clamp(1.0, 65535.0) as u16;
                range_img.put_pixel(c as u32, row as u32, Luma([r_mm]));
                inten_img.put_pixel(c as u32, row as u32, Luma([n.intensity.min(255.0) as u8]));
            }
        }
    }
    range_img.save(dir.join(format!("range.png")))?;
    inten_img.save(dir.join(format!("intensity.png")))?;

    // previews: histogram-equalized over valid pixels (both channels are heavily
    // skewed -- most intensities are < 10, most ranges are a few metres);
    // near = bright, no return = black
    let range_eq = equalize(range_img.pixels().map(|p| p[0] as usize), 1 << 16);
    // (intensity is shifted by 1 so a real 0-intensity return isn't dropped as "no return")
    let inten_eq = equalize(
        inten_img.pixels().zip(range_img.pixels())
            .filter(|(_, r)| r[0] > 0).map(|(i, _)| i[0] as usize + 1),
        257,
    );
    let range_view = GrayImage::from_fn(w, h, |x, y| {
        let r = range_img.get_pixel(x, y)[0] as usize;
        Luma([if r == 0 { 0 } else { 256 - range_eq[r] as u16 } as u8])
    });
    let inten_view = GrayImage::from_fn(w, h, |x, y| {
        // range is clamped >= 1 mm for valid cells, so it doubles as the validity mask
        let valid = range_img.get_pixel(x, y)[0] > 0;
        Luma([if valid { inten_eq[inten_img.get_pixel(x, y)[0] as usize + 1] } else { 0 }])
    });

    let preview_dir = dir.join("preview");
    std::fs::create_dir_all(&preview_dir)?;
    for (name, img) in [("range", range_view), ("intensity", inten_view)] {
        imageops::resize(&img, w, h * PREVIEW_ROW_SCALE, FilterType::Nearest)
            .save(preview_dir.join(format!("{name}.png")))?;
    }
    Ok(())
}