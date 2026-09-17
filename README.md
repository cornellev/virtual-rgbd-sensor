# Virtual RGBD Sensor
In Rust!

## Table of Contents 
1) [Features](#features)
2) [Recent Updates](#recent-updates)
3) [Instructions](#instructions)
4) [Debugging Notes](#debugging-notes)

## Features:
1) Converted 32-channel RoboSense LiDAR outputs in the form of .pcap or MSOP/DIFOP packets to PointCloud2: ```(x, y, z, intensity, cluster_id)```. Implementation explanation is [here](src/rslidar/lidar.pdf).
2) Implemented [Two-Layer-Graph Clustering](https://www.mdpi.com/2076-3417/10/23/8534) for 32-channel point cloud segmentation, per frame and without cross-frame matching and consistency. Implementation explanation is [here](src/rslidar/2-LAYER.pdf).
![segmentation_demo](/demos/segmentation.png)
<!-- <video width="320" height="240" controls>
  <source src="/demos/yay.mp4" type="video/mp4">
</video> -->

3) Ported cpp ```nav2_costmap_2d``` point cloud to costmap conversion to Rust.
![costmap_demo](/demos/occupancy_grid.png)

## Recent Updates:

|  Date     | Changelog / Update Notes |
|:----------|:-----------|
| 9/14/25  | - Integrated Two-Layer-Graph Clustering with the decoding of MSOP/DIFOP packets for optimized segmentation while point cloud are being processed. Specifically during the construction of the range and set graphs. |
| 9/12/26   | - Tested ```rslidar_sdk_node.rs``` on online LiDAR and offline .pcap files. Also integrated simple Bevy visualizer for point clouds. | 
| 9/8/26   | - Rewrote cpp ```rslidar_sdk``` with ```RSHeliosDecoder``` for RoboSense 32-channel LiDAR specifically. This is for converting offline .pcap or online MSOP/DIFOP packets to point clouds | 

## Instructions:
Run:
```bash
RUSTFLAGS="-C link-arg=-fuse-ld=gold" cargo run --bin rslidar_viz --release
```

### For Offline Demo:
To run the point cloud conversion and segmentation on the provided offline LiDAR packets, make sure that ```pcap_path``` in ```config.yaml``` is set to whatever file path points to ```test_cloud.pcap```. 

### For Online Demo:
If you want to run ```rslidar_sdk_node``` on online LiDAR via MSOP/DIFOP ports, make sure the ```pcap_path``` in ```config.yaml``` is empty.


## Debugging Notes:
Our LiDAR is pinged via 192.168.1.102:
```bash
sudo ip link set enP8p1s0 up
sudo ip addr ad 192.168.1.102/24 dev enP8p1s0
```

If working in a Docker container, it needs to be able to access IP addresses.

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
should actually display.
