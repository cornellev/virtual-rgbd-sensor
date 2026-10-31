use std::sync::{Arc, Mutex};
use std::time::Instant;
use virtual_rgbd_sensor::rslidar_sdk_node::{DriverConfig, RsHeliosDecoder};
use virtual_rgbd_sensor::segmentation::SegParams;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let cfg = DriverConfig::parse(&std::fs::read_to_string(&args[1]).unwrap());
    let data = std::fs::read(&args[2]).unwrap();
    let mut dec = RsHeliosDecoder::new(&cfg, Arc::new(Mutex::new(SegParams::default())));
    let (mut frame_ms, mut pkt_ms) = (Vec::new(), 0.0f64);
    let mut clusters = Vec::new();
    let (mut same, mut far) = ([0u64; 2], [[0u64; 2]; 3]);
    let mut off = 24;
    while off + 16 <= data.len() {
        let incl = u32::from_le_bytes(data[off + 8..off + 12].try_into().unwrap()) as usize;
        let frame = &data[off + 16..off + 16 + incl];
        off += 16 + incl;
        if frame.len() < 42 {
            continue;
        }
        let udp = 14 + (frame[14] & 0x0F) as usize * 4;
        let port = u16::from_be_bytes([frame[udp + 2], frame[udp + 3]]);
        let payload = &frame[udp + 8..];
        match port {
            7788 => dec.decode_difop(payload),
            6699 => {
                let t = Instant::now();
                let out = dec.decode_msop(payload);
                let ms = t.elapsed().as_secs_f64() * 1e3;
                if out.is_empty() {
                    pkt_ms += ms;
                } else {
                    frame_ms.push(ms + pkt_ms);
                    pkt_ms = 0.0;
                    for (_, g) in &out {
                        use virtual_rgbd_sensor::segmentation::{NUM_COLS, NUM_RINGS};
                        for i in 0..NUM_RINGS - 1 {
                            for j in 0..NUM_COLS {
                                let (a, b) = (&g.nodes[i * NUM_COLS + j], &g.nodes[(i + 1) * NUM_COLS + j]);
                                if !(a.valid && b.valid && a.beta != -1 && b.beta != -1) {
                                    continue;
                                }
                                let d = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt();
                                let r = a.range.min(b.range);
                                if d < 0.15 {
                                    same[0] += 1;
                                    same[1] += (a.beta != b.beta) as u64;
                                } else if d > 0.5 {
                                    let k = if r < 4.0 { 0 } else if r < 8.0 { 1 } else { 2 };
                                    far[k][0] += 1;
                                    far[k][1] += (a.beta == b.beta) as u64;
                                }
                            }
                        }
                        let mut ids: Vec<i32> = g.nodes.iter().filter(|n| n.valid && n.beta != -1).map(|n| n.beta).collect();
                        ids.sort_unstable();
                        ids.dedup();
                        clusters.push(ids.len());
                    }
                }
            }
            _ => {}
        }
    }
    frame_ms.sort_by(|a, b| a.total_cmp(b));
    let n = frame_ms.len();
    let mean = frame_ms.iter().sum::<f64>() / n as f64;
    let mean_clusters = clusters.iter().sum::<usize>() as f64 / clusters.len() as f64;
    println!(
        "{n} frames | per-frame decode+seg ms: mean {mean:.2}  p50 {:.2}  p95 {:.2}  max {:.2} | clusters/frame mean {mean_clusters:.1}",
        frame_ms[n / 2], frame_ms[n * 95 / 100], frame_ms[n - 1]
    );
    let pct = |c: [u64; 2]| 100.0 * c[1] as f64 / c[0].max(1) as f64;
    println!(
        "stacked same-surface pairs split: {:.1}% (n={}) | >0.5m pairs merged: <4m {:.1}% (n={}), 4-8m {:.1}% (n={}), >8m {:.1}% (n={})",
        pct(same), same[0], pct(far[0]), far[0][0], pct(far[1]), far[1][0], pct(far[2]), far[2][0]
    );
}
