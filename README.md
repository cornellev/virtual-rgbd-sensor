# Virtual RGBD Sensor
In Rust! Read the documentation [here](/documentation/report.pdf).

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

3) Adopted cpp ```nav2_costmap_2d``` point cloud to costmap conversion logic for outputting occupancy grids in Rust. Implementation details are contained [here](/src/rslidar/costmap).
![costmap_demo](/demos/occupancy_grid.png)

## Recent Updates:

|  Date     | Changelog / Update Notes |
|:----------|:-----------|
| 10/17/26 | - Tweaked ```costmap``` to output occupancy grids instead with ```NO_INFORMATION, LETHAL_OBSTACLE, FREE_SPACE``` elements. | 
| 9/26/26  | - Rewrote [ros2-planning costmap_2d](https://github.com/ros-planning/navigation/tree/noetic-devel/costmap_2d/src) implementation in Rust, also integrated costmap construction directly after `RangeGraph` initialization |
| 9/14/26  | - Integrated Two-Layer-Graph Clustering with the decoding of MSOP/DIFOP packets for optimized segmentation while point cloud are being processed. Specifically during the construction of the range and set graphs. |
| 9/12/26   | - Tested ```rslidar_sdk_node.rs``` on online LiDAR and offline .pcap files. Also integrated simple Bevy visualizer for point clouds. | 
| 9/8/26   | - Rewrote cpp ```rslidar_sdk``` with ```RSHeliosDecoder``` for RoboSense 32-channel LiDAR specifically. This is for converting offline .pcap or online MSOP/DIFOP packets to point clouds | 

## Instructions:
Have LiDAR perception results output via Zenoh publisher or shared memory, and listen to these
outputs by running the visualizer. 

To run Zenoh publisher:
```bash
RUSTFLAGS="-C link-arg=-fuse-ld=gold" cargo run --bin zenoh_wrapper --release
```

To run Zenoh shm:
```bash
RUSTFLAGS="-C link-arg=-fuse-ld=gold" cargo run --bin shm_wrapper --release
```

To run Zenoh subscriber (able to listen to publisher or shm):
```bash
RUSTFLAGS="-C link-arg=-fuse-ld=gold" cargo run --bin zenoh_viz --release
```

### For Offline Demo:
To run the point cloud conversion and segmentation on the provided offline LiDAR packets, make sure that ```pcap_path``` in ```config.yaml``` is set to whatever file path points to ```test_cloud.pcap```. 

### For Online Demo:
If you want to run ```rslidar_sdk_node``` on online LiDAR via MSOP/DIFOP ports, make sure the ```pcap_path``` in ```config.yaml``` is empty.


## Zenoh Mappings
A brief overview on the Zenoh pub/sub elements in ```virtual-rgbd-sensor```:
### Publishers:
1) ```rslidar/points/raw```: raw ```(x, y, z, intensity)``` point cloud from RoboSense LiDAR MSOP/DIFOP packets
2) ```rslidar/points/segmented```: segmented ```(x, y, z, cluster_id)``` point cloud from ```segmentation.rs```'s two-layer-graph clustering
3) ```rslidar/costmap```: occupancy grid containing free space, obstacle regions, and no information layers (from occlusion) from Rust implementation of ```ros-planning costmap_2d``` in ```/costmap```

### Subscribers:
In ```zenoh_test.rs```, there are example point cloud and costmap subscribers in ```_points_subscriber``` and ```_costmap_subscriber```, respectively.

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
