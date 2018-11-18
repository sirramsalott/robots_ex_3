#! /usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robots_exercise_3.msg import TrackFace, FaceLocked
import face_detection_lib as fdl

class FaceHandler:
    MODE_SCANNING = 0
    MODE_TRACKING = 1
    MODE_LOCKED = 2
    
    def __init__(self):
        self.mode = self.MODE_SCANNING
        self.trackingFace = None

        self.trackFacePub = rospy.Publisher("track_face", TrackFace)
        self.faceLockedPub = rospy.Publisher("face_locked", FaceLocked)
        self.faceLostPub = rospy.Publisher("face_lost", Empty)

        self.imageSub = rospy.Subscriber("/usb_cam/image_raw",
                                         Image,
                                         self.imageCallback,
                                         queue=1)
        self.interactionCompleteSub = rospy.Subscriber("interaction_complete",
                                                       Empty,
                                                       self.completeCallback<
                                                       queue=1)

        self.bridge = CvBridge()

    def completeCallback(self, msg):
        self.mode = self.MODE_SCANNING

    def imageCallback(self, img):
        img_cv = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img_np = np.asarray(img_cv)

        if self.mode == self.MODE_SCANNING:
            self.scanningMode(img_np)

        elif self.mode == self.MODE_TRACKING:
            self.trackingMode(img_np)

        elif self.mode == self.MODE_LOCKED:
            self.lockedMode(img_np)

    def scanningMode(self, img_np):
        faces = fdl.getAllFaces(img_np)
        if len(faces) > 0:
            self.trackingFace = fdl.selectFaceToTrack(faces)
            self.mode = self.MODE_TRACKING

    def trackingMode(self, img_np):
        faceBoundingBox = fdl.findFaceInImage(imp_np, self.trackingFace)

        if faceBoundingBox is None:
            self.publishFaceLost()
            self.mode = self.MODE_SCANNING
            self.trackingFace = None

        elif fdl.faceIsCentred(faceBoundingBox, img_np):
            self.mode = self.MODE_LOCKED
            faceType, faceID = fdl.faceTypeAndID(self.trackingFace)
            self.publishFaceLocked(faceType, faceID)

        else:
            x, y = fdl.boundingBoxCentre(faceBoundingBox)
            self.publishTrackFace(x, y)

    def lockedMode(self, img_np):
        faceBoundingBox = fdl.findFaceInImage(img_np, self.trackingFace)

        if faceBoundingBox is None:
            self.mode = self.MODE_SCANNING
            self.publishFaceLost()

    def publishTrackFace(self, x, y):
        trackFaceMsg = TrackFace()
        trackFaceMsg.imgX = x
        trackFaceMsg.imgY = y
        self.trackFacePub.publish(trackFaceMsg)

    def publishFaceLocked(self, faceType, faceID):
        faceLockedMsg = FaceLocked()
        faceLockedMsg.type = faceType
        if faceID is not None:
            faceLockedMsg.id = faceID
        self.faceLockedPub.publish(faceLockedMsg)

    def publishFaceLost(self):
        self.faceLostPub.Publish(Empty())

if __name__ == "__main__":
    rospy.init_node(name="face_detect")
    fh = FaceHandler()
    rospy.spin()
