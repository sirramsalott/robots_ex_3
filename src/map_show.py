#! /usr/bin/env python

import numpy as np
import cv2

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robots_exercise_3.msg import TrackFace, StudentFaceLocked, NewFaceLocked
from face_detection_lib import FaceDetectionModel

class FaceHandler:
    
    def __init__(self):

        self.framesReceived = 0

        self.imageSub = rospy.Subscriber("/heat_map", Image, self.imageCallback, queue_size=1)

        self.bridge = CvBridge()

    def imageCallback(self, img):
        self.framesReceived += 1
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")

        if self.framesReceived % self.FRAME_RATE == 0:
            if self.mode == self.MODE_SCANNING:
                img = self.scanningMode(img)

            elif self.mode == self.MODE_TRACKING:
                img = self.trackingMode(img)

            elif self.mode == self.MODE_LOCKED:
                img = self.lockedMode(img)

            if self.visualise:
                cv2.imshow("Camera Stream", img)
                cv2.waitKey(1)
