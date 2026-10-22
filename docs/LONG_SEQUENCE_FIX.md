# Long-sequence memory and overlapping-map fixes

Working branch: `fix/long-sequence-map`, based on published `v1.0.0`.
Camera calibration and optimization/feature thresholds are unchanged. No loop closure
or dense-map substitute has been added. This branch has not been pushed.

## Confirmed implementation defects

1. The bag adapter preloaded all left images and decoded raw point clouds.
   Its trajectory also owned every complete `Lidar`, retaining point clouds,
   feature clouds and KD trees. Input is now streamed; history stores poses,
   sparse keyframe features and weak references to active estimator objects.
2. `Frame::m_last_frame` formed an unbounded strong ancestor chain. After the
   mapper is idle, the adapter keeps the immediate predecessor required by
   tracking/marginalization and cuts its older ancestor reference. Per-frame
   and per-scan preintegrations are released by their owners; key-interval
   accumulators have shared ownership instead of leaking raw allocations.
3. Deskew did not cover points before the previous image-aligned state
   (measured roughly 20 ms in this bag). It interpolated identity to a cumulative
   pose using a mismatched time origin. The new timeline brackets the entire
   scan, integrates backward/forward from the previous state, and interpolates
   adjacent poses. Preintegration clips both observation endpoints and cannot
   consume those extra early measurements as negative integration steps.
4. After deskew, `registerPoints` inferred firing rings from rotated elevations.
   Channel identity is fixed at acquisition, not in the reference scan frame.
   Bag input now preserves the actual hardware ring and relative point time.
   Legacy folder input remains available with its default convention.
5. Local world-grid points retained pre-BA positions while their keyframes were
   optimized. Active submaps now keep local coordinates and follow the latest
   keyframe pose; retired submaps are frozen only after leaving the local window.
   Weakly held child scans follow their anchor's pose correction too. Existing
   rolling-grid bounds and 0.8 m registration resolution are retained.
6. Three LiDAR/visual-LiDAR local BA paths skipped the oldest state on result
   application, but left its pose/velocity/bias vertices free. The solver could
   move a state that was never committed. Result application now commits every
   optimized state, including the oldest. This preserves the original graph's
   optimization freedom; active submaps/cache follow those updated poses.
   An earlier fixed-oldest-state attempt was not kept in the final implementation.
7. ROS global publication reconstructed all world points on every update. Its
   incremental voxel cache now subtracts a keyframe's old contribution before
   inserting its optimized replacement. Repeat publication does not append
   another copy. This is real SLAM feature geometry, not raw LiDAR output.
8. `newer_college.yaml` had the LiDAR/IMU Z lever arm reversed. Combining the
   recorded `os1_sensor -> os1_imu` and `os1_sensor -> os1_lidar` static transforms
   gives `Tbl.z=+0.028535 m`, not `-0.028535 m`. Both Tbl and inverse Tlb are
   corrected; the old translation discrepancy was 0.057070 m. This conclusion
   comes from the actual bag TF, not an assumed coordinate convention.
9. Valgrind exposed an uninitialized rotation-update counter in the LiDAR and
   fused `ImuCamPose` constructors (`ImuCamPose::Update`, not a PCL false positive).
   Those constructors now initialize their counter and accumulated rotation
   state. The rotation normalization function returns a matrix; its previously
   ignored return value is now applied in both pose-update methods.
10. Fused multi-frame BA computed `(l_err_num/v_err_num)` using integer
    division. A larger visual count made the LiDAR information weight zero;
    zero visual observations caused SIGFPE BEFORE its existing degraded-camera
    policy could execute. The captured instruction resolves to
    `bundle_adjustment.cpp`'s weight calculation. Empty first-window history
    also produced undefined reliability ratios. The ratio is now floating point,
    unavailable history has neutral relative reliability, and the original
    degraded-camera policy runs before division. Regression cases cover each.

## Evidence and verification

Evidence is retained in ignored local `run/long_map_fix/` directories.

- Original v1 30 s: completed, measured peak RSS about 2025 MB.
- Original v1 100 s selection: aborted by the 4500 MB test guard at about
  31 s of bag processing (4531 MB sampled RSS). This is a failed/partial run,
  not a successful 100 s baseline; its maps do not prove long-run accuracy.
- Intermediate fix (before hardware-ring/fixed-BA-anchor corrections): completed
  100 s, 810 poses, all initialization flags 1, peak RSS 1918 MB. ROS map/path,
  original stereo image bytes, exact sensor times and final TF checks passed.
  However, fixed-ROI wall spread increased. It was NOT accepted as a complete
  geometry fix, and its logs remain under `fixed100` for comparison.
- Before the final reliability correction (`final100_v2`): 100 s completed,
  810 poses, all initialization flags 1, peak RSS 1925 MB, 429086 output voxels
  from 570757 sparse feature points. ROS/TF/stereo image checks and actual RViz
  display passed. This is NOT geometric acceptance: an independent reconstruction
  from final keyframe poses still shows moving copies of the courtyard.
- Independent consecutive-keyframe ICP (`independent`), before the reliability
  correction: for high-overlap pairs from 65--100 s, median residual correction
  was 0.291 m per keyframe pair. This is a local consistency diagnostic, not
  ground truth; low-overlap comparisons against the starting scan were rejected.
  A repeat under GDB also stopped with SIGFPE at the integer count ratio above.
- After the reliability correction (`weights100`, final code): 100 s completed
  under GDB, then the keep-alive node was stopped cleanly (inferior exited
  normally). 2998 images, 810 LiDAR states/poses, all initialization flags 1;
  183789 map voxels from 570937 sparse feature points, peak RSS 1977.6 MB.
  Trajectory timestamps/quaternions pass, path length 58.173 m, max 0.237 m step.
  These trajectory checks do not measure ground-truth accuracy.
  ROS map/path/odom/TF, final late-listener TF/no-repeat behavior, and original
  stereo image headers/bytes pass on this final executable.
- Final executable through actual ROS Launch (`weights_launch8`), offset 19 s,
  duration 8 s: normal SLAM exit, 239 images, 76 LiDAR poses, all flags 1,
  14592 map points, peak RSS 1027.9 MB. The first stationary 8 s still cannot
  satisfy the original monocular motion initializer; thresholds are unchanged.
- Final CTest passes. Valgrind's synthetic consistency test reports zero errors
  (`--leak-check=no`, NOT a claim of zero leaks). The real three-scan LiDAR
  memory check also reports zero errors; its LiDAR implementation is unchanged
  by the final fusion-weight correction. `test_vilo` is rebuilt and its original
  source is unchanged; its folder-data runtime was not tested.

## Independent geometry result and limits

The decisive check uses actual sparse keyframe feature clouds in their local
coordinates, transformed by their final SLAM poses. GDB exports these read-only
at the end of the run. No raw-cloud replacement, global ICP alignment, leaf-size
change, or changed feature/initialization threshold is involved.

In the same two courtyard facade ROIs, inspect 82 keyframes from 59--100 s.
With `u=(x-y)/sqrt(2)`, `v=(x+y)/sqrt(2)`, and `0.5<z<5 m`, the first wall uses
`-15<u<-5` and fits `v=a*(u+10)+b`; the perpendicular wall uses `-25<v<-10`
and fits `u=a*(v+17.5)+b`. The dominant observed wall band is fitted separately
for each keyframe, without aligning those keyframes to each other.

| Wall-intercept range over time | Before weight correction | Final code |
| --- | ---: | ---: |
| First facade | 20.700 m | 0.106 m |
| Perpendicular facade | 5.236 m | 0.096 m |

This resolves the observed large moving courtyard copies in this tested prefix.
The ranges measure **temporal consistency in the selected physical facade
regions**, not absolute pose error, a guarantee for every surface, or a zero-drift
global map. Within-scan facade spread remains about 0.2--0.3 m in these sparse
features. Consecutive point-to-point ICP also improves (65--100 s median required
translation correction 0.291 -> 0.168 m), but is affected by sparse voxel feature
sampling and should not be interpreted as ground-truth trajectory error.

Plots, exported keyframes, scripts, and logs are retained locally under
`run/long_map_fix/weights100/`; `before_after.png` compares time-colored geometry
before/after the fusion-weight correction. The older `final100` fixed-anchor
experiment is NOT the final implementation. `final100_v2` and `independent`
pre-date the decisive weight correction.

Remaining boundaries: no loop-closure/global pose graph was added; the adapter
still starts LiDAR after visual bootstrap, and the original pre-LIO stage does
not perform the new gravity-based full deskew. The geometry of that initialization
interval has not been certified. Arbitrary-length/global loop accuracy and other
bags have not been accepted by these tests.

## Changed-file groups

- Streaming/history/output: `example/test_vilo_bag.cpp`, `slam_archive.h`,
  `incremental_voxel_map.h`, `ros_bag_output.h`.
- Estimation/local-map lifetime and state consistency:
  `src/{frame,frame_handler_mono,lidar,depth_filter,bundle_adjustment,ba_types}.cpp`,
  `include/vilo/{frame,frame_handler_mono,lidar,depth_filter}.h`,
  new `imu_timeline.h` and `sensor_reliability.h`.
- Calibration: `example/newer_college.yaml` (LiDAR/IMU Z translation only).
- Build/regressions/documentation: `CMakeLists.txt`, `tests/map_consistency_test.cpp`,
  `tests/{capture_map,check_bag_calibration,compare_map_geometry,run_bounded}.py`,
  `README.md`, `docs/{BAG_ROS,MAPPING_REVIEW,LONG_SEQUENCE_FIX}.md`.

All changes are local and uncommitted on `fix/long-sequence-map`; nothing was
pushed and published `v1.0.0` remains at `b0c969b75ccc30fedf92142df5489d0368e67f2b`.

The C++ consistency regression checks clipped IMU steps, points before the state
anchor, non-identity sensor extrinsics, hardware ring preservation, replacement
of old voxel contributions, weak historical ownership, anchored child poses,
and a synthetic BA window whose returned states must satisfy their own IMU
factor after optimization (no discarded oldest-state update), and fusion weights.
The early small-structure fixed-ROI wall-width check is only a warning: later
scans may see different parallel surfaces inside those ROIs. It is not a
standalone courtyard-geometry acceptance test. Time-separated actual keyframe
clouds and high-overlap consecutive registration provide the independent check.

## Build and run

```bash
cd /home/bob/work/TCK-VILO
source /opt/ros/noetic/setup.bash
cmake -S . -B build
cmake --build build --target test_vilo_bag map_consistency_test test_vilo -j1
(cd build && ctest --output-on-failure)
source build/devel/setup.bash
roslaunch vilo test_vilo_bag.launch duration:=100
```

This launch reads the official local bag and opens RViz itself. Do not also
run `rosbag play`. For headless processing add `rviz:=false`; for an 8 s motion
test use `offset:=19 duration:=8`. Final latched topics remain until Ctrl-C.

```bash
python3 tests/check_ros_output.py --check-keepalive
python3 tests/check_ros_images.py bag/rooster_2020-03-10-10-36-30_0.bag
python3 tests/check_trajectory.py "$HOME/.ros/tck_vilo_trajectory.tum"
```

Map point count may grow with genuinely newly observed geometry; occupied
voxels are not a fixed-size rolling display. The changes remove time-linear
retention of complete input sequences and repeated old pose contributions.
No finite global-memory bound or zero global drift is claimed for arbitrary
unbounded exploration without a global map/loop-closure policy.
