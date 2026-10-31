import json
import socket
import struct
import time
from pathlib import Path
from scipy.spatial.transform import Rotation
import pandas as pd
import math 

import zenoh

ROOT = Path(__file__).resolve().parent
TRAJECTORY = ROOT / "trajectory.jsonl"
PCAP = ROOT / "run" / "helios32.pcap"
HOST = "127.0.0.1"
PORTS = (6699, 7788)

# A "transform" here is (translation, rotation): translation a (x, y, z)
# tuple, rotation a (w, x, y, z) tuple -- same layout rslidar's Rust Transform
# uses, so compose()/quat_mul() below mirror Transform::then()/apply().


def stamp_ns(stamp):
    return int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q, v):
    """Rotates vector v=(x, y, z) by unit quaternion q=(w, x, y, z)."""
    w, x, y, z = q
    qvec = (x, y, z)
    t = tuple(2.0 * c for c in _cross(qvec, v))
    u = _cross(qvec, t)
    return tuple(v[i] + w * t[i] + u[i] for i in range(3))


def compose(outer, inner):
    """outer . inner: apply inner first, then outer (Transform::then)."""
    (ot, oq), (it, iq) = outer, inner
    rotated = quat_rotate(oq, it)
    translation = tuple(ot[i] + rotated[i] for i in range(3))
    rotation = quat_mul(oq, iq)
    return translation, rotation


def load_mount_and_poses():
    """Returns (mount, events). `mount` is the helios_frame -> base_link
    transform, or None if tf_static never named it. `events` are
    (sort_key_us, order, (stamp_ns, body_to_world)) tuples, order=0."""
    mount = None
    events = []
    for line in TRAJECTORY.read_text().splitlines():
        if not line:
            continue
        row = json.loads(line)
        if row["key"] == "orch/tf_static":
            for transform in row["body"]["transforms"]:
                if transform["child_frame_id"] != "helios_frame":
                    continue
                body = transform["transform"]
                t, r = body["translation"], body["rotation"]
                mount = ((t["x"], t["y"], t["z"]), (r["w"], r["x"], r["y"], r["z"]))
                # mount = ((0, 0, 0), (1, 0, 0, 0))
        elif row["key"] == "orch/tf":
            for transform in row["body"]["transforms"]:
                if transform["child_frame_id"] != "base_link":
                    continue
                body = transform["transform"]
                t, r = body["translation"], body["rotation"]
                ns = stamp_ns(transform["header"]["stamp"])
                pose = ((t["x"], t["y"], t["z"]), (r["w"], r["x"], r["y"], r["z"]))
                events.append((ns // 1000, 0, (ns, pose)))
    return mount, events


def load_packets(events):
    data = PCAP.read_bytes()
    if struct.unpack_from("<I", data, 0)[0] != 0xA1B2C3D4:
        raise SystemExit("helios32.pcap is not a little-endian PCAP")
    offset = 24
    while offset + 16 <= len(data):
        ts_sec, ts_usec, incl_len, _orig_len = struct.unpack_from("<IIII", data, offset)
        frame = data[offset + 16:offset + 16 + incl_len]
        offset += 16 + incl_len
        if len(frame) < 42:
            continue
        udp_at = 14 + (frame[14] & 0x0F) * 4
        if udp_at + 8 > len(frame):
            continue
        dest = struct.unpack_from("!H", frame, udp_at + 2)[0]
        if dest not in PORTS:
            continue
        payload = bytes(frame[udp_at + 8:])
        events.append((ts_sec * 1_000_000 + ts_usec, 1, (dest, payload)))


def encode_pose(stamp_ns_value, lidar_to_world):
    """Mirrors `encode_pose` in zenoh_wrapper.rs: a 12-byte header (stamp_sec:
    i32, stamp_nanosec: u32, reserved: u32) followed by translation (x, y, z:
    f32) and rotation (qx, qy, qz, qw: f32), all little-endian, 40 bytes."""
    translation, (w, x, y, z) = lidar_to_world
    stamp_sec = stamp_ns_value // 1_000_000_000
    stamp_nanosec = stamp_ns_value % 1_000_000_000
    tx, ty, tz = translation
    return struct.pack("<iII7f", stamp_sec, stamp_nanosec, 0, tx, ty, tz, x, y, z, w)

def invert(transform):
    """Inverse of a (translation, rotation) transform: if T maps A -> B,
    invert(T) maps B -> A."""
    t, (w, x, y, z) = transform
    q_inv = (w, -x, -y, -z)  # conjugate == inverse for a unit quaternion
    rt = quat_rotate(q_inv, t)
    return (-rt[0], -rt[1], -rt[2]), q_inv

def main():
    mount, events = load_mount_and_poses()
    load_packets(events)
    events.sort(key=lambda item: (item[0], item[1]))
    conf = zenoh.Config()
    conf.insert_json5("mode", json.dumps("peer"))
    conf.insert_json5("connect/endpoints", json.dumps(["tcp/127.0.0.1:7447"]))
    session = zenoh.open(conf)
    # Already lidar -> world; rslidar has no part in this anymore, so it's
    # published directly in the same binary format zenoh_wrapper used to
    # publish and zenoh_viz already decodes, not as orch/location JSON.
    publisher = session.declare_publisher("rslidar/pose_world")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    time.sleep(0.3)

    origin = events[0][0] if events else 0
    started = time.monotonic()
    poses = 0
    packets = 0
    for time_us, order, item in events:
        delay = (time_us - origin) / 1_000_000 - (time.monotonic() - started)
        if delay > 0:
            time.sleep(delay)
        if order == 1:
            dest, payload = item
            sock.sendto(payload, (HOST, dest))
            packets += 1
        else:
            stamp_ns_value, body_to_world = item
            lidar_to_world = compose(body_to_world, mount)
            # world_to_lidar = invert(lidar_to_world)
            publisher.put(encode_pose(stamp_ns_value, lidar_to_world))
            poses += 1
    time.sleep(0.3)
    session.close()
    sock.close()
    print(f"replayed {poses} poses and {packets} packets", flush=True)


if __name__ == "__main__":
    main()
