#!/usr/bin/env python3
"""Check the live/latching SLAM output, including bag time and TF frames."""
import math
import argparse
import time
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage

rospy.init_node("check_vilo_output", anonymous=True)
parser = argparse.ArgumentParser()
parser.add_argument("--check-keepalive", action="store_true",
                    help="after processing ends, check late TF lookup and no repeated broadcasts")
args = parser.parse_args(rospy.myargv()[1:])
cloud = rospy.wait_for_message("/vilo/map_cloud", PointCloud2, timeout=180)
path = rospy.wait_for_message("/vilo/path", Path, timeout=30)
while len(path.poses) < 2:
    path = rospy.wait_for_message("/vilo/path", Path, timeout=30)
pose = rospy.wait_for_message("/vilo/pose", PoseStamped, timeout=30)
odom = rospy.wait_for_message("/vilo/odom", Odometry, timeout=30)
clock = rospy.wait_for_message("/clock", Clock, timeout=30)
tf = rospy.wait_for_message("/tf", TFMessage, timeout=30)
static_tf = rospy.wait_for_message("/tf_static", TFMessage, timeout=30)
world = pose.header.frame_id
assert cloud.width * cloud.height > 0, "empty SLAM map"
assert len(path.poses) >= 2, "trajectory not initialized"
assert cloud.header.frame_id == path.header.frame_id == odom.header.frame_id == world
assert cloud.header.stamp.to_sec() > 1e9, "map lacks bag sensor timestamp"
assert clock.clock >= cloud.header.stamp, "map timestamp ahead of bag clock"
assert all(b.header.stamp > a.header.stamp for a,b in zip(path.poses,path.poses[1:])), "path timestamps not monotonic"
for p in path.poses:
    q=p.pose.orientation
    assert all(math.isfinite(x) for x in (p.pose.position.x,p.pose.position.y,p.pose.position.z,q.x,q.y,q.z,q.w))
    assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 1e-5
points = 0
for xyz in point_cloud2.read_points(cloud, field_names=("x","y","z")):
    assert all(math.isfinite(x) for x in xyz), "map contains NaN/Inf"
    points += 1
assert any(t.header.frame_id == world and t.child_frame_id == odom.child_frame_id for t in tf.transforms), "missing world/body TF"
assert {"vilo_camera","vilo_lidar"}.issubset({t.child_frame_id for t in static_tf.transforms}), "missing sensor extrinsics"
print("PASS SLAM map_points={} path_poses={} frame={} bag_time={:.6f} TF=connected".format(
    points,len(path.poses),world,cloud.header.stamp.to_sec()))

if args.check_keepalive:
    received = []
    def receive_tf(msg):
        received.extend(t for t in msg.transforms
                        if t.header.frame_id == world and t.child_frame_id == odom.child_frame_id)
    subscriber = rospy.Subscriber("/tf", TFMessage, receive_tf, queue_size=100)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    deadline = time.monotonic() + 5.0
    while not buffer.can_transform(world, odom.child_frame_id, pose.header.stamp):
        assert time.monotonic() < deadline, "late listener cannot look up final TF"
        time.sleep(0.05)
    time.sleep(3.0)
    assert len(received) == 1, "expected one latched final TF, got {} broadcasts".format(len(received))
    assert received[0].header.stamp == pose.header.stamp, "final TF sensor timestamp changed"
    final_clock = rospy.wait_for_message("/clock", Clock, timeout=5)
    assert final_clock.clock == clock.clock, "keepalive advanced the bag clock"
    subscriber.unregister()
    print("PASS keepalive: late TF lookup works, one retained TF, no rebroadcast, bag clock unchanged")
