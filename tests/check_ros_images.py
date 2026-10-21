#!/usr/bin/env python3
"""Verify displayed stereo images against original bag bytes and headers."""
import argparse
import hashlib

import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


def signature(image):
    # roscpp assigns header.seq from the new publisher's own sequence counter.
    # Compare exposure time/frame, format and actual pixels, not transport seq.
    return (image.header.stamp.to_nsec(), image.header.frame_id,
            image.width, image.height, image.encoding, image.is_bigendian,
            image.step, hashlib.sha256(bytes(image.data)).hexdigest())


def main():
    rospy.init_node("check_vilo_images", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--left-topic", default="/camera/infra1/image_rect_raw")
    parser.add_argument("--right-topic", default="/camera/infra2/image_rect_raw")
    args = parser.parse_args(rospy.myargv()[1:])
    with rosbag.Bag(args.bag) as bag:
        for topic in (args.left_topic, args.right_topic):
            image = rospy.wait_for_message(topic, Image, timeout=180)
            clock = rospy.wait_for_message("/clock", Clock, timeout=10)
            assert image.width > 0 and image.height > 0, "empty image"
            assert image.step >= image.width, "invalid image stride"
            assert len(image.data) == image.step * image.height, "invalid image payload"
            assert clock.clock >= image.header.stamp, "image ahead of simulated clock"
            stamp = image.header.stamp
            original = None
            # Official bag recording time and sensor image time are aligned.
            for _, candidate, _ in bag.read_messages(
                    topics=[topic], start_time=stamp-rospy.Duration(1),
                    end_time=stamp+rospy.Duration(1)):
                if candidate.header.stamp == stamp:
                    original = candidate
                    break
            assert original is not None, "published image timestamp not found in bag"
            assert signature(image) == signature(original), "image differs from original bag"
            print("PASS {} {}x{} {} frame={} stamp={:.9f} original_bytes=identical".format(
                topic, image.width, image.height, image.encoding,
                image.header.frame_id, stamp.to_sec()))


if __name__ == "__main__":
    main()
