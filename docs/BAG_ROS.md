# Official Newer College bag: build, run, and verification

Project: `/home/bob/work/TCK-VILO`. ROS Noetic / Ubuntu 20.04.
The original `example/test_vilo.cpp` and `test_vilo` executable are retained.

## Build

```bash
cd /home/bob/work/TCK-VILO
source /opt/ros/noetic/setup.bash
bash build.sh
source build/devel/setup.bash
```

Incremental build after editing project sources:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
```

The executable is available both through `rosrun vilo test_vilo_bag` and
`bin/test_vilo_bag`. CMake keeps these copies synchronized.
Keep Eigen's 16-byte ABI for the project and bundled g2o; do not restore
`-march=native` while linking Ubuntu's prebuilt PCL.
The existing OpenCV 3.4 installation is retained. No cv_bridge/OpenCV 4 objects
cross the algorithm interface; ROS images are copied explicitly into cv::Mat.

## Launch with RViz

```bash
source /opt/ros/noetic/setup.bash
source /home/bob/work/TCK-VILO/build/devel/setup.bash
roslaunch vilo test_vilo_bag.launch \
  bag:=/home/bob/work/TCK-VILO/bag/rooster_2020-03-10-10-36-30_0.bag \
  calib:=/home/bob/work/TCK-VILO/example/newer_college.yaml \
  duration:=30
```

The node reads the bag directly. **Do not run a second rosbag player.**
It publishes `/clock`; the launch sets `/use_sim_time=true`.
`offset`, `duration` are seconds from the beginning of the bag's recording time.
The official bag starts almost stationary: the first 8 seconds did not provide
enough parallax for the original monocular initializer. The first successful
visual initialization in the tested prefix occurred around 19 seconds.
No initialization thresholds or calibration parameters were lowered/changed.

An 8-second window containing actual motion:

```bash
roslaunch vilo test_vilo_bag.launch offset:=19 duration:=8
```

Optional arguments: `rviz:=false`, `keep_alive:=false`,
`output:=/absolute/path/trajectory.tum`, `image_topic:=...`, `right_image_topic:=...`, `lidar_topic:=...`,
`imu_topic:=...`, `world_frame:=...`, `body_frame:=...`.
`map_voxel_size:=0.2` controls the displayed global map resolution in meters;
`map_publish_period:=1.0` limits map rebuilding/publication in wall-clock seconds.
Pose/TF updates are not throttled. The final map is always published.
Default output is `$HOME/.ros/tck_vilo_trajectory.tum`.
Do not run two copies simultaneously on the same ROS master.

By default the final publishers stay alive after processing so late RViz
subscribers can receive the latched map/path/pose. Ctrl-C closes the launch.
The last dynamic `/tf` message is also latched for late subscribers. Each sensor
timestamp is broadcast once; the final refresh and keep-alive stage do not
rebroadcast the same transform or advance the sensor clock.
To open RViz separately:

```bash
rviz -d /home/bob/work/TCK-VILO/rviz/slam_bag.rviz
```

## Output semantics

| Output | Meaning |
|---|---|
| `/vilo/map_cloud` | Registered keyframe surface features fused into global voxels in `vilo_world` |
| `/vilo/path` | Body/IMU poses estimated by SLAM, with original sensor timestamps |
| `/vilo/pose` | Latest estimated body/IMU pose |
| `/vilo/lidar_pose` | Latest estimated LiDAR pose |
| `/vilo/odom` | Body pose and estimated linear velocity in the child frame |
| `/tf` | `vilo_world -> vilo_body` |
| `/tf_static` | `vilo_body -> vilo_camera`, `vilo_body -> vilo_lidar`, using Tbc/Tbl |
| `/clock` | Bag image time, advancing as the offline estimator processes images |
| `/camera/infra1/image_rect_raw` | Original left infrared image used by SLAM; configurable via `image_topic` |
| `/camera/infra2/image_rect_raw` | Original right infrared image for display only; configurable via `right_image_topic` |

Both images are published while processing, including before initialization,
and latched so a late RViz can display the final received image. Their original
sensor timestamps, optical frame names, encoding and pixel bytes are preserved.
ROS assigns `header.seq` from each new publisher's sequence counter.
The right stream is read on demand, not preloaded as a second full
image sequence. The default RViz includes two Image panels and launch remaps
their subscriptions when input topic arguments change. These are 2D `Image`
displays, not a calibrated 3D `Camera` overlay; no right-camera TF or CameraInfo
is invented. Image panels do not require TF. Since the estimator runs offline,
wall-clock playback rate need not be 30 Hz and intermediate display frames can
be skipped. Images are input data, not an SLAM mapping result.
`/clock` now preserves the original image nanoseconds; ROS poses/path/TF aligned
to those images recover the same exact stamp from the core's double seconds.
In an existing RViz, open this repository's updated config or add an `Image`
display and select the corresponding image topic with `raw` transport. Do not
choose `Camera`, which also requires calibrated CameraInfo.

The map uses `Lidar::ptr_less_surfs_` after feature extraction/local mapping,
transformed using each retained keyframe's optimized LiDAR pose. It is a sparse
SLAM feature map, not a dense reconstruction and not the raw input cloud.
Global voxel fusion retains a centroid per occupied voxel, preventing repeated
correctly aligned observations from adding duplicate displayed points in that
voxel. `RESULT map_input_points` reports features before global fusion and
`map_points` reports the published result. This reduces output duplication, not
pose drift: separated wall layers may occupy different voxels and remain visible.
Map rebuilding still traverses retained keyframes; long-run incremental submap
management and geometric accuracy require separate validation and fixes.
The path is rebuilt from retained estimated states so initialization/optimization
corrections are reflected. No map is published before any SLAM scan is processed.

This is the original monocular visual frontend: it consumes the left infrared
image. The right stereo image is present in the bag but is not fused by this
algorithm. IMU input is Ouster `/os1_cloud_node/imu` (not RealSense IMU).
Ouster fields are float32 x/y/z, uint32 `t` in nanoseconds and uint8 `ring`.
The scan's end time is header time plus the maximum actual point offset.
There is no clock-domain rewrite or callback-arrival-time correction.

## Verify

After the result line (while the launch keeps the publishers alive):

```bash
cd /home/bob/work/TCK-VILO
source /opt/ros/noetic/setup.bash
source build/devel/setup.bash
python3 tests/check_ros_output.py
python3 tests/check_ros_output.py --check-keepalive
python3 tests/check_ros_images.py bag/rooster_2020-03-10-10-36-30_0.bag
python3 tests/check_trajectory.py "$HOME/.ros/tck_vilo_trajectory.tum"
rostopic echo -n 1 /vilo/map_cloud/width
rosrun tf tf_echo vilo_world vilo_body
```

Offline trajectory-only invocation, without a ROS master:

```bash
bin/test_vilo_bag \
  bag=/home/bob/work/TCK-VILO/bag/rooster_2020-03-10-10-36-30_0.bag \
  calib=/home/bob/work/TCK-VILO/example/newer_college.yaml \
  duration=30 output=/tmp/tck_trajectory.tum
python3 tests/check_trajectory.py /tmp/tck_trajectory.tum
```

`map_points` in the console summary counts **ROS-published** points. It is zero
in trajectory-only mode. Exit code 7 explicitly means the requested window never
produced an initialized trajectory; it is not a segmentation fault.
TUM rows contain `timestamp x y z qx qy qz qw`; timestamps are not frame indices.

Memory regression covering the original crashing function with three real scans:

```bash
valgrind --error-exitcode=99 --leak-check=no \
  bin/lidar_memory_test bag/rooster_2020-03-10-10-36-30_0.bag
```

## Root causes and changes

1. `-march=native` selected a 32-byte Eigen allocator in the project, while
   Ubuntu PCL allocated filter output using its 16-byte ABI. The project then
   read the word before the allocation and freed a bogus pointer. An independent
   VoxelGrid reproducer crashed with native/32-byte alignment and succeeded with
   system/16-byte alignment. Its corrected Valgrind run reported zero errors.
   Fixed in root CMake and bundled g2o CMake; both libraries were rebuilt.
2. 26 graph insertion calls were inside `assert(optimizer.addVertex/addEdge(...))`.
   NDEBUG removed the calls, not just the checks. GDB subsequently caught
   `visualLocalBundleAdjustment` dereferencing missing vertices. Graph insertion
   now executes unconditionally, with a runtime failure if insertion is rejected.
3. Corner and surface KD trees shared one instance. They now have distinct
   ownership. Empty scans and failed nearest-neighbor searches cannot index
   absent results.
4. The depth-filter history manually deleted raw pointers owned by shared_ptr.
   It now releases the container's shared ownership instead of double deleting.
5. Offline input used an IMU endpoint before the observation and rewound/cleared
   the IMU buffer. It now brackets each interval with samples on both sides,
   consumes each scan once and waits for mapping before advancing/reading results.
   Paired LiDAR is queued before the image wakes mapping. LiDAR input begins
   after visual bootstrap, preventing reset/initialization state from being used
   as a completed visual estimate.

Changed/added implementation files: `CMakeLists.txt`, `build.sh`,
`thirdparty/g2o/CMakeLists.txt`, `src/bundle_adjustment.cpp`, `src/lidar.cpp`,
`src/frame_handler_mono.cpp`, `src/depth_filter.cpp`, `include/vilo/depth_filter.h`,
`example/test_vilo_bag.cpp`, `example/ros_bag_output.h`, `package.xml`,
`launch/test_vilo_bag.launch`, `rviz/slam_bag.rviz`, and the three `tests/` files.
Other pre-existing user changes were retained.

## Recorded results

Logs and trajectories are under `run/verified8`, `run/verified30`, and
`run/verified_motion8`. Original failing GDB/Valgrind logs remain under
`run/bag_seq1`; the newly exposed pre-fix graph crash is recorded in
`run/verified30/gdb.log`.

* Real point-cloud regression: 3 scans, Valgrind **0 errors from 0 contexts**.
  This is a targeted invalid-access/free check, not a full application leak audit.
* First 8 seconds: 240 images read, 239 processed, normal shutdown with explicit
  no-initialization status; no trajectory/map falsely reported.
* First 30 seconds offline after graph fix: 898 images, 110 LiDAR states;
  VIO/LIO/fused initialization all completed. 110 finite, timestamp-monotonic
  TUM poses; maximum adjacent step 0.148 m in that run.
* First 30 seconds through Launch + RViz: 110 poses, 31,262 SLAM map points,
  valid bag timestamps and connected TF. RViz subscribed to the SLAM outputs
  and initialized OpenGL successfully; no raw input cloud is in its configuration.
* Motion window offset 19, duration 8 seconds offline: 238 images, 76 LiDAR
  states; VIO/LIO/fused all completed. 76 finite monotonic poses.
* The same 8-second motion window through Launch + RViz: 238 images, 76 poses,
  25,071 SLAM map points; all initialization flags true. ROS output checker
  passed point finiteness, timestamps, path quaternions and connected TF.
* TF keep-alive regression after removing duplicate broadcasts: the 8-second
  motion window produced 76 poses and 25,044 map points. A listener connected
  after processing could query the final TF, received one retained transform
  with no rebroadcast over 3 seconds, and observed an unchanged bag clock.
  RViz's launch log contained no `TF_REPEATED_DATA`. Evidence is under
  `run/verified_tf_keepalive`.

These checks establish functioning short-sequence estimation and ROS outputs.
No ground-truth accuracy evaluation or complete 167-second bag acceptance is
claimed. Mapping is multithreaded; exact trajectory metrics can vary between runs.

## Image and global display-map follow-up

Final regression evidence: `run/verified_image_fusion_exact8`.
Offset 19/duration 8: 238 images, 76 LiDAR states/poses, all three initialization
flags true, 25,124 keyframe features fused into 14,107 displayed map points.
Original left/right pixel bytes, encoding, optical frames and exact sensor
timestamps matched the bag. Live and late image checks passed; RViz established
both Image subscriptions. Map/path/TF and final TF keep-alive checks passed.
The finite trajectory had a maximum adjacent displacement of 0.667 m; this is
not an accuracy acceptance or evidence that geometric overlap is fixed.

Before-initialization regression: `run/verified_images_stationary8`.
The first 8 seconds processed 239 images and no initialized LiDAR states;
both image topics still passed original-data checks. No map was fabricated.

Full diagnosis, remaining geometric risks and ordered implementation plan:
`docs/MAPPING_REVIEW.md`. In particular, display voxel fusion does not repair
deskew time-coverage errors or stale internal local-map coordinates.
