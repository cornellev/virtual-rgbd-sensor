# Virtual RGBD Sensor
## Tasks:
1) Rewrite LiDAR PCAP packet -> sensor_msgs::msg::PointCloud2: DONE, read
[this](src/rslidar/lidar.pdf) for impl deets
2) sensor_msgs::msg::PointCloud2 -> nav2::msg::Costmap: SEMI-DONE but still ROS2 dependent
3) Camera + sensor_msgs::msg::PointCloud2 -> RGBD

Some Docker instructions:
Our LiDAR is pinged via 192.168.1.102:
```bash
sudo ip link set enP8p1s0 up
sudo ip addr ad 192.168.1.102/24 dev enP8p1s0
```

Docker needs to be able to access IP addresses.

Run the Rust ```rslidar_sdk``` with
```bash
 RUSTFLAGS="-C link-arg=-fuse-ld=gold" cargo run --bin rslidar_viz
```

x11 host on Docker so I can GUI
on local:
```bash
ssh -X mini-dos@cev_jetson0.coecis.cornell.edu
```

in cev_jetson0, to test if GUI working
```bash
xclock

sudo docker run -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/root/.Xauthority \
    -v ~/.Xauthority:/root/.Xauthority:ro \
    --name dbimage-container \
    dbimage:lidar-dev \
    /bin/bash
```

in docker

```bash
rviz2
```
should actually display




