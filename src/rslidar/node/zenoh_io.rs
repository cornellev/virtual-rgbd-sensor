//! Zenoh plumbing shared by the driver binaries. Subscriber callbacks run on
//! Zenoh's threads, so poses are forwarded over a channel to the thread that
//! owns the `Pipeline`.

use std::sync::{mpsc, Arc, Mutex};

use anyhow::{anyhow, Result};
use zenoh::{
    pubsub::{Publisher, Subscriber},
    qos::CongestionControl,
    Session, Wait,
};

use crate::segmentation::{SegParams, SEG_PARAMS_KEY};
use crate::transform::Transform;
use crate::wire::decode_pose;

pub const POSE_KEY: &str = "rslidar/pose_world";
pub const POINTS_RAW_KEY: &str = "rslidar/points/raw";
pub const POINTS_SEG_KEY: &str = "rslidar/points/segmented";
pub const RANGEGRAPH_KEY: &str = "rslidar/points/rangegraph";
pub const COSTMAP_KEY: &str = "rslidar/costmap";
pub const POSE_FRAME_KEY: &str = "rslidar/pose_frame";

/// Drops instead of blocking when a subscriber falls behind, so a slow
/// consumer can't add latency to the driver.
pub fn publisher<'a>(session: &'a Session, key: &'static str) -> Result<Publisher<'a>> {
    session
        .declare_publisher(key)
        .congestion_control(CongestionControl::Drop)
        .wait()
        .map_err(|e| anyhow!("declare {key} publisher: {e}"))
}

pub fn subscribe_poses(session: &Session, tx: mpsc::Sender<(i64, Transform)>) -> Result<Subscriber<()>> {
    session
        .declare_subscriber(POSE_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            match decode_pose(&payload) {
                Some(pose) => {
                    let _ = tx.send(pose);
                }
                None => eprintln!("rslidar: dropping malformed pose ({} bytes)", payload.len()),
            }
        })
        .wait()
        .map_err(|e| anyhow!("subscribe {POSE_KEY}: {e}"))
}

/// Live tuning from zenoh_viz; the decoder rereads `params` every revolution.
pub fn subscribe_seg_params(session: &Session, params: Arc<Mutex<SegParams>>) -> Result<Subscriber<()>> {
    session
        .declare_subscriber(SEG_PARAMS_KEY)
        .callback(move |sample| {
            let payload = sample.payload().to_bytes();
            match SegParams::from_bytes(&payload) {
                Some(p) => *params.lock().unwrap() = p,
                None => eprintln!("rslidar: dropping malformed seg params ({} bytes)", payload.len()),
            }
        })
        .wait()
        .map_err(|e| anyhow!("subscribe {SEG_PARAMS_KEY}: {e}"))
}
