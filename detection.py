#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import roslib
roslib.load_manifest('detector')
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray

sys.path.insert(0, "Mask_RCNN/")

from Mask_RCNN import apple_detection_mask_rcnn
class InferenceConfig(apple_detection_mask_rcnn.BalloonConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    DETECTION_MIN_CONFIDENCE = 0.7

config = InferenceConfig()
config.display()

from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
import os
import sys
import random
import math
import numpy as np
import skimage.io
import time
import glob
import logging

import matplotlib
import matplotlib.pyplot as plt

# Local path to trained weights file
MODEL_DIR = 'weights/'
COCO_MODEL_PATH = MODEL_DIR +'mask_rcnn_apple_0008.h5'
# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
# Load weights trained on MS-COCO
model.load_weights(COCO_MODEL_PATH, by_name=True, exclude=["mrcnn_bbox"])
# for online service (clear the last session)
model.keras_model._make_predict_function()
class_names = ['bg', ' ']


def detect(img):
    start = time.time()
    results = model.detect([img], verbose=0) # 1: display log information
    r = results[0]    
    print('\nResult: ----------------')
    duration = time.time() - start;
    print('The detection time is: %.2fs' % (duration))
    # print('The count of apples is ', len(r['class_ids']))
    return r['rois']


pub = rospy.Publisher('Detection', Int16MultiArray, queue_size=1)
bridge = CvBridge()

def color_callback(image_msg):
  image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
  bboxes = detect(image)
  bbox_msg = Int16MultiArray()
  bbox_msg.data = bboxes.ravel()
  # print(bbox_msg.data)
  pub.publish(bbox_msg)

def depth_callback(depth_msg):
  image = bridge.imgmsg_to_cv2(depth_msg, "mono16")


def subscribe():
  rospy.Subscriber('color_image', Image, color_callback, queue_size = 1)
  # rospy.Subscriber('depth_image', Image, depth_callback, queue_size = 1)
  rospy.spin()


if __name__ == '__main__':
    logging.basicConfig(filename="log.log", level=logging.INFO)
    try:
        rospy.init_node('Detector', anonymous=True)
        subscribe()
    except rospy.ROSInterruptException:
        pass


