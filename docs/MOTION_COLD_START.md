# Newer College 运动冷启动修复（2026-10-22）

修改基线：`bag_newer_college`，HEAD `aea5b02`。本次没有提交或推送。
用户已暂存的 `launch/test_vilo_bag.launch` 改动原样保留；默认 Bag 是 `_9`。
`example/test_vilo.cpp`、标定文件、ASRE 权重和点云滤波分辨率没有修改。

## 根因及修改

1. **初始化对齐发生尺度坍缩，仍然提交为融合成功。** 原 `_9` 运行中，
   19 对同步位姿经单位变换起始的自由 Sim3 迭代，尺度变成
   `1.109878e-20`，位置 RMSE 为 `1.686 m`。随后仍缩放点逆深度、速度并设置
   `is_livo_initialized_`，最终 27 秒轨迹长约 942 m、相邻位姿速度约 235 m/s。
   这不是“移动开机必然漂移”的正常行为。
2. **世界坐标变换不一致。** 原逻辑分别拟合位置旋转与姿态旋转，并在姿态右侧相乘。
   两条轨迹描述的是相同 IMU 刚体，世界系对齐应对姿态左乘，且位置、姿态、速度
   应使用同一旋转。近直线运动时，单靠位置无法可靠约束绕运动方向的旋转。
3. **配对索引可能错位。** 早期图像可能早于首个可用雷达扫描，配对位姿向量并不
   与全部视觉关键帧一一同序。现保留实际图像索引，未配对帧保留其对齐后的位姿。
4. **缺少初始化验收。** 现在通过刚体姿态对应关系求旋转，再闭式求尺度和平移。
   至少需要 6 对位姿及有效平移基线；尺度须有限且为正，位置 RMSE 不大于
   0.5 m，姿态 RMSE 不大于 10°。不合格时不修改地图、不进入融合，保留独立
   VIS/LIS 处理，在后续测量到来后重新尝试对齐。插值 IMU 临时对象由 RAII 释放。

这不是轨迹限幅、隐藏异常或用原始点云代替 SLAM 地图。没有增加数据集文件名判断，
也没有增加“必须静止开始”的限制。未完成融合初始化的 Bag 入口返回 7，部分位姿
只作为诊断输出，不再以成功状态退出。

## 实测结果

所有下列有效窗口均正常退出，VIS/LIS/融合初始化标志均为 1。
轨迹长度是估计轨迹的累计路程，不是与真值比较的精度指标。

| 输入窗口 | 图像 / 位姿数 | 路程 | 最大相邻位移 | 峰值 RSS |
| --- | ---: | ---: | ---: | ---: |
| `_9`，0–27 s，离线 | 809 / 268 | 22.247 m | 0.536 m | 1607 MiB |
| `_9`，0–完整 Bag，ROS Launch | 809 / 268 | 22.350 m | 0.536 m | 1556 MiB |
| `_0`，0–167 s，离线 | 5006 / 1480 | 129.959 m | 0.148 m | 2196 MiB |
| `_0`，120–147 s，运动中启动 | 809 / 269 | 29.619 m | 0.261 m | 1460 MiB |
| `_9`，8–27 s，运动中启动 | 570 / 188 | 12.806 m | 0.524 m | 1430 MiB |

- `_9` 起点对齐：尺度约 7.346，位置 RMSE 0.268 m，姿态 RMSE 0.754°。
  最大速度出现在启动前 1 秒；第 8 秒后最高约 1.54 m/s，未再发生原来的数值飞行。
- `_0` 前 100 秒有 810 个位姿、路程 57.863 m；原稳定记录为 810 个位姿、
  58.173 m。59–100 秒两面院墙的逐关键帧截距范围约 0.161 / 0.088 m，
  没有出现数十米的逐步平移叠加。完整 167 秒的常驻完整扫描数维持约 100。
- `_9` 使用实际 SLAM 关键帧特征构图。ROS 最终地图 96,281 个体素点，
  输入特征 183,982 个，轨迹 268 个位姿；不是原始扫描话题。
  实际关键帧按前后时段着色检查；固定早期参考墙面后，第 6 秒至结束两面墙的
  逐帧法向中位数变化范围约 0.165 / 0.127 m。该检查是局部一致性诊断，不是全局真值精度。
- 地图、位姿、轨迹、里程计、模拟时钟、传感器外参 TF、末帧 TF 保持检查通过；
  左右 848×480 mono8 图像与原 Bag 的时间戳、格式、像素内容一致。
- 一致性单元测试通过，覆盖近直线、大角度世界系差异、尺度恢复、错误对齐拒绝、
  姿态变换方向、速度缩放，以及原有 IMU/地图/内存所有权回归。
  只有 1 秒的 `_9` 窗口因初始化未完成按预期返回 7。

日志、轨迹、实际关键帧导出、地图快照及图示保存在本机忽略目录
`run/cold_start_fix/`；`first9/keyframe_time_map.png` 与
`regression0/keyframe_time_map.png` 是实际 SLAM 几何的前后时段叠加图。
离线测试关闭 ROS 发布，其日志 `map_points=0` 不表示算法没有地图：关键帧特征另由
`tests/export_slam_keyframes.gdb` 导出。ROS 地图非空由单独的 Launch 测试确认。

## 编译与运行

```bash
cd /home/bob/work/TCK-VILO
source /opt/ros/noetic/setup.bash
cmake -S . -B build
cmake --build build --target test_vilo_bag map_consistency_test test_vilo -j1
(cd build && ctest --output-on-failure)
source build/devel/setup.bash

# _9：实际 Bag 长约 27 秒，duration=30 会处理到 Bag 结束，自动打开 RViz。
roslaunch vilo test_vilo_bag.launch \
  bag:=$PWD/bag/rooster_2020-03-10-11-01-34_9.bag duration:=30

# 结束上一个进程后，再运行 _0。
roslaunch vilo test_vilo_bag.launch \
  bag:=$PWD/bag/rooster_2020-03-10-10-36-30_0.bag duration:=167
```

程序自身读取 Bag 并发布 `/clock`，不要另外 `rosbag play`。无界面运行可加
`rviz:=false`；结束后保留最终结果，Ctrl-C 退出。工作空间环境入口是
`build/devel/setup.bash`，不是根目录的 `devel/setup.bash`。

```bash
# 正常运行的另一终端中检查输出（使用同一 ROS_MASTER_URI）。
source /opt/ros/noetic/setup.bash
python3 tests/check_ros_output.py
python3 tests/check_trajectory.py /home/bob/.ros/tck_vilo_trajectory.tum --max-speed 10
```

轨迹文件在处理结束后写完，速度上限是此步行数据的可选发散诊断，**不对算法输出限幅**。
验证命令只证明输出及有限数值检查，不能替代地图几何或真值精度验证。

## 边界

运动启动不等于瞬时完成初始化，仍需积累几秒有约束的数据。最初初始化暂态的误差
比融合完成后大，不能把所有启动阶段位姿都当作已收敛结果。本次没有加入回环，
也没有完整重写 Stage I 或所有退化场景的重初始化状态机；不承诺任意无纹理、
约束不足或传感器异常的片段都能冷启动，更不承诺零漂移或已达到真值精度。

## 修改文件

- `include/vilo/initialization_alignment.h`：有姿态约束的闭式对齐与验收。
- `src/depth_filter.cpp`：初始化对齐调用、索引对应、失败重试及临时对象释放。
- `src/map.cpp`：统一世界系变换的姿态乘法方向。
- `example/test_vilo_bag.cpp`：初始化未完成时明确失败。
- `tests/map_consistency_test.cpp`、`tests/check_trajectory.py`：回归与发散检查。
- `tests/export_slam_keyframes.gdb`：实际 SLAM 特征地图诊断导出。
- 本文档。用户已暂存的 Launch 不属于本轮修改。
