// Compiles the cev-interfaces lidar protobuf messages (lidar.proto + the
// common.proto it imports) into Rust with prost, exposed as
// `virtual_rgbd_sensor::cev_proto`.
//
// Uses protox (a pure-Rust protobuf compiler) so no `protoc` is needed.
//
// The .proto files are read from a cev-interfaces checkout: set
// CEV_INTERFACES_PROTO to its `proto/` dir, or it defaults to the sibling
// checkout at ../cev-interfaces/proto.

use std::path::PathBuf;

const PROTO_FILES: &[&str] = &["cev/interfaces/v1/lidar.proto"];

fn main() {
    println!("cargo:rerun-if-env-changed=CEV_INTERFACES_PROTO");

    let proto_dir = std::env::var_os("CEV_INTERFACES_PROTO")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../cev-interfaces/proto")
        });
    if !proto_dir.join(PROTO_FILES[0]).is_file() {
        panic!(
            "can't find {} under {} -- clone cev-interfaces next to this repo or set \
             CEV_INTERFACES_PROTO to its proto/ dir",
            PROTO_FILES[0],
            proto_dir.display()
        );
    }

    let fds = protox::compile(PROTO_FILES, [&proto_dir]).unwrap_or_else(|e| panic!("protox: {e:?}"));
    // Rerun when lidar.proto or anything it imports changes.
    for file in &fds.file {
        if let Some(name) = &file.name {
            println!("cargo:rerun-if-changed={}", proto_dir.join(name).display());
        }
    }

    prost_build::Config::new()
        // Same option cev-interfaces' generate.sh uses: `bytes` fields are
        // `bytes::Bytes`, so the big point-cloud/costmap buffers move in
        // without a copy.
        .bytes(["."])
        .compile_fds(fds)
        .expect("prost-build");
}
