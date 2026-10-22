#!/usr/bin/env python3
"""Capture actual SLAM map snapshots for offline geometric comparison.

This subscribes only to /vilo/map_cloud, never to the raw LiDAR topic.
"""
import argparse
import os
import threading
import time
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Path

parser=argparse.ArgumentParser()
parser.add_argument("output")
parser.add_argument("--start",type=float,required=True,help="bag recording start in seconds")
parser.add_argument("--at",type=float,nargs="+",default=[29,59,99])
parser.add_argument("--timeout",type=float,default=1800)
args=parser.parse_args(rospy.myargv()[1:])
os.makedirs(args.output,exist_ok=True)
rospy.init_node("capture_vilo_map",anonymous=True)
latest_path=None
remaining=sorted(args.at)
lock=threading.Lock()

def path_callback(msg):
    global latest_path
    latest_path=msg

def cloud_callback(msg):
    with lock:
        elapsed=msg.header.stamp.to_sec()-args.start
        if not remaining or elapsed<remaining[0]:return
        path=latest_path
        if not path or len(path.poses)<2:return
        points=np.array(list(point_cloud2.read_points(msg,field_names=("x","y","z"))),dtype=np.float32)
        poses=np.array([[p.header.stamp.to_sec(),p.pose.position.x,p.pose.position.y,p.pose.position.z]
                        for p in path.poses],dtype=np.float64)
        assert np.isfinite(points).all(),"Map contains non-finite points"
        while remaining and elapsed>=remaining[0]:
            target=remaining.pop(0)
            filename=os.path.join(args.output,"map_{:g}s.npz".format(target))
            np.savez_compressed(filename,points=points,poses=poses,
                stamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id)
            print("SNAPSHOT target={} actual={:.3f} points={} poses={} file={}".format(
                target,elapsed,len(points),len(poses),filename),flush=True)

rospy.Subscriber("/vilo/path",Path,path_callback,queue_size=1)
rospy.Subscriber("/vilo/map_cloud",PointCloud2,cloud_callback,queue_size=1,buff_size=64*1024*1024)
deadline=time.monotonic()+args.timeout
while not rospy.is_shutdown():
    with lock:
        if not remaining:break
    if time.monotonic()>deadline:raise RuntimeError("Missing map snapshots: {}".format(remaining))
    time.sleep(0.1)
