#!/usr/bin/env python3
"""Check official Ouster body/LiDAR calibration against recorded static TF."""
import argparse
import cv2
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix

parser=argparse.ArgumentParser()
parser.add_argument("bag")
parser.add_argument("calibration")
args=parser.parse_args()
transforms={}
with rosbag.Bag(args.bag) as bag:
    for _,message,_ in bag.read_messages(topics=["/tf_static"]):
        for tf in message.transforms:
            child=tf.child_frame_id.lstrip("/")
            if child not in ("os1_imu","os1_lidar"):continue
            p=tf.transform.translation;q=tf.transform.rotation
            matrix=quaternion_matrix([q.x,q.y,q.z,q.w])
            matrix[:3,3]=[p.x,p.y,p.z]
            transforms[child]=(tf.header.frame_id.lstrip("/"),matrix)
        if len(transforms)==2:break
assert len(transforms)==2,"Missing recorded Ouster static transforms"
assert transforms["os1_imu"][0]==transforms["os1_lidar"][0],"Different TF parents"
expected=np.linalg.inv(transforms["os1_imu"][1])@transforms["os1_lidar"][1]
source=cv2.FileStorage(args.calibration,cv2.FILE_STORAGE_READ)
assert source.isOpened(),"Calibration cannot be read"
Tbl=source.getNode("Tbl").mat()
Tlb=source.getNode("Tlb").mat()
source.release()
assert np.max(np.abs(Tbl-expected))<1e-6,"Tbl disagrees with physical Bag TF"
assert np.max(np.abs(Tbl@Tlb-np.eye(4)))<1e-6,"Tbl/Tlb are not inverse transforms"
print("PASS Tbl/Tlb agree with official Bag TF; body_from_lidar translation_m={}".format(Tbl[:3,3]))
