#!/usr/bin/env python

import rosbag
import cv2
from cv_bridge import CvBridge

bag = rosbag.Bag('33.bag')
bridge = CvBridge()

for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw']):
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imwrite("33.png", cv_img)
    print("Saved at timestamp", t)
    break

bag.close()