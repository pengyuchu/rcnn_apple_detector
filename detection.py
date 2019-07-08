#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import roslib

roslib.load_manifest('rcnn_apple_detector')

import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray

sys.path.insert(0, "Mask_RCNN/")

from Mask_RCNN import apple_detection_mask_rcnn

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


class InferenceConfig(apple_detection_mask_rcnn.BalloonConfig):

	# Set batch size to 1 since we'll be running inference on
	# one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
	GPU_COUNT = 1
	IMAGES_PER_GPU = 1
	DETECTION_MIN_CONFIDENCE = 0.8

config = InferenceConfig()
config.display()

# Local path to trained weights file
MODEL_DIR = 'weights/'
MODEL_NAME = 'model.h5'
MODEL_PATH = MODEL_DIR + MODEL_NAME
# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
# Load weights trained on MS-COCO
# model.load_weights(COCO_MODEL_PATH, by_name=True, exclude=["mrcnn_bbox"])
model.load_weights(MODEL_PATH, by_name=True, exclude=["mrcnn_bbox"])
# for online service (clear the last session)
model.keras_model._make_predict_function()
class_names = ['bg', ' ']

# for the server without display
plt.switch_backend('agg')

pub = rospy.Publisher('Detection', Int16MultiArray, queue_size=1)
# img_pub = rospy.Publisher('detection_img', Image, queue_size=1)
bridge = CvBridge()


def detect(img):

	start = time.time()
	results = model.detect([img], verbose=0) # 1: display log information
	print('results length: ', len(results))
	r = results[0]    
	print('\nResult: ----------------')
	duration = time.time() - start;
	print('The detection time is: %.2fs' % (duration))
	print('The count of apples is ', len(r['class_ids']))
	# masked_image = visualize.save_instances(img, r['rois'], r['masks'], r['class_ids'], 
	#                           class_names, scores = r['scores'], filename = 'test.jpg', colors= [(0.2, 0.2, 0.95)]*100, figsize=(12, 12), show_mask=True, show_bbox=True)
	# origial image
	# img = bridge.cv2_to_imgmsg(masked_image, "rgb8")
	# img_pub.publish(img)
	return r['rois']


def color_callback(image_msg):
	
	image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
	bboxes = detect(image)
	bbox_msg = Int16MultiArray()
	bbox_msg.data = bboxes.ravel()
	print(bbox_msg.data)
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
