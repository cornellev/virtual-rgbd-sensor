// Rust types generated from cev-interfaces `cev/interfaces/v1/lidar.proto`
// (and the `common.proto` it imports) by build.rs. Published by the
// `zenoh_protobuf` binary, decoded by `zenoh_viz --proto`.

include!(concat!(env!("OUT_DIR"), "/cev.interfaces.v1.rs"));

pub use prost::Message;

/// Nanoseconds since the epoch from the driver's split (sec, nanosec) stamp.
/// Stamps before 1970 clamp to 0, since `Header.stamp_ns` is unsigned.
pub fn stamp_ns(stamp_sec: i32, stamp_nanosec: u32) -> u64 {
    (stamp_sec as i64 * 1_000_000_000 + stamp_nanosec as i64).max(0) as u64
}

impl Header {
    pub fn new(stamp_ns: u64, frame_id: impl Into<String>, seq: u64) -> Self {
        Self { stamp_ns, frame_id: frame_id.into(), seq }
    }
}
