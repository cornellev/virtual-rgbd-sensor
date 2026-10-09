#![allow(dead_code)]

pub fn be_u16(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

pub fn be_u32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

pub fn be_u48(b: &[u8]) -> u64 {
    let mut v: u64 = 0;
    for &byte in &b[0..6] {
        v = (v << 8) | byte as u64;
    }
    v
}

pub fn cos_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().cos() as f32
}

pub fn sin_centideg(angle: i32) -> f32 {
    ((angle as f64) * 0.01).to_radians().sin() as f32
}

pub fn azimuth_round(v: i32) -> i32 {
    ((v % 36000) + 36000) % 36000
}