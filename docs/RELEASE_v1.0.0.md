# v1.0.0 — ROS Noetic bag-mapping baseline

Algorithm/source baseline: `f961419c5954ba425f03f1b0e43c4fb40fa4479e`.
Release preparation changes packaging/build discovery only, not calibration,
initialization thresholds, deskewing, registration, or optimization algorithms.

## Included

- Original standalone `test_vilo` and synchronized `test_vilo_bag`.
- Official Newer College image/Ouster IMU/LiDAR input and configurable bag window.
- SLAM feature map, body/LiDAR pose, odometry, trajectory, TF and simulated clock.
- Original left/right infrared images and an RViz configuration.
- Global display-map voxel fusion and map publication throttling.
- Memory-crash fixes, unconditional optimization-graph insertion and targeted
  trajectory/ROS/image regression checks from the baseline.
- Rebuildable bundled FAST/Sophus/g2o source, configurable external OpenCV/Pangolin
  discovery, and exclusion of local datasets, caches and build products.

## Requirements

Ubuntu 20.04 / ROS Noetic; OpenCV 3.4.20 and Pangolin 0.6 were used on the test
machine. Other dependencies and clone/build/launch commands are in README.md.
Install dependencies and compile after cloning; no Bag or portable binary is
bundled. The generated environment is `build/devel/setup.bash`.

## Release verification

A clean source clone (without the original build caches or generated libraries)
rebuilt FAST, Sophus, g2o, libvilo, `test_vilo`, `test_vilo_bag` and
`lidar_memory_test` on the stated environment. The build was completed with
one job after reducing concurrency to avoid desktop memory pressure.

Official bag, offset 19 seconds and duration 8 seconds, headless ROS launch:
238 processed images, 76 LiDAR states/trajectory poses, VIO/LIO/fused flags all
initialized. 25,109 keyframe surface features yielded 14,627 global display-map
points. Original left/right image data and timestamps matched the bag;
map/path/clock/TF output and final TF keep-alive checks passed.
Trajectory-format checks passed (76 finite monotonic poses, maximum adjacent
displacement 0.558 m). These are startup/output checks, not geometric accuracy
acceptance; the exact trajectory can vary with multithreaded execution.

## Known limits

- Long windows can exhaust RAM. Streaming input and retained-state lifetime
  work remain outstanding; lowering displayed map point counts alone is not a fix.
- The right image is displayed but not fused into the monocular frontend.
- Short tests are not a ground-truth accuracy or full 167-second acceptance.
- Deskew time coverage and historical local-map/optimized-pose consistency need
  follow-up validation and correction.
- No complete global loop closure is provided.

Keep tag `v1.0.0` unchanged. Develop subsequent fixes on separate branches.
