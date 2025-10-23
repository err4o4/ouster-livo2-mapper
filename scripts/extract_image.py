#!/usr/bin/env python3
import argparse
import rosbag
import cv2
from cv_bridge import CvBridge

# python3 extract_image.py --bag 33.bag --output 33.png

def main():
    parser = argparse.ArgumentParser(description="Extract a single image from a ROS bag.")
    parser.add_argument("--bag", "-b", required=True, help="Path to ROS bag file (e.g. 33.bag)")
    parser.add_argument("--topic", "-t", default="/usb_cam/image_raw", help="Image topic name")
    parser.add_argument("--output", "-o", default="frame.png", help="Output image filename (e.g. 33.png)")
    args = parser.parse_args()

    bag = rosbag.Bag(args.bag)
    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(args.output, cv_img)
        print(f"Saved {args.output} at timestamp {t}")
        break

    bag.close()

if __name__ == "__main__":
    main()
