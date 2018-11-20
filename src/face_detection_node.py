#! /usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robots_exercise_3.msg import TrackFace, StudentFaceLocked, NewFaceLocked
from face_detection_lib import FaceDetectionModel

class FaceHandler:
    MODE_SCANNING = 0
    MODE_TRACKING = 1
    MODE_LOCKED = 2
    FRAME_RATE = 4
    
    def __init__(self):
        self.mode = self.MODE_SCANNING
        self.trackingFace = None
        self.framesReceived = 0

        self.fdm = FaceDetectionModel()

        self.trackFacePub = rospy.Publisher("track_face", TrackFace)
        self.studentFaceLockedPub = rospy.Publisher("student_face_locked", StudentFaceLocked)
        self.newFaceLockedPub = rospy.Publisher("new_face_locked", NewFaceLocked)
        self.faceLostPub = rospy.Publisher("face_lost", Empty)

        self.imageSub = rospy.Subscriber("/usb_cam/image_raw",
                                         Image,
                                         self.imageCallback,
                                         queue_size=1)
        self.interactionCompleteSub = rospy.Subscriber("interaction_complete",
                                                       Empty,
                                                       self.completeCallback<
                                                       queue_size=1)

        self.bridge = CvBridge()

    def completeCallback(self, msg):
        self.mode = self.MODE_SCANNING

    def imageCallback(self, img):
        framesReceived += 1
        if framesReceived % FRAME_RATE == 0:
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")

            if self.mode == self.MODE_SCANNING:
                self.scanningMode(img)

            elif self.mode == self.MODE_TRACKING:
                self.trackingMode(img)

            elif self.mode == self.MODE_LOCKED:
                self.lockedMode(img)

    def scanningMode(self, img):
        boundingBoxes = self.fdm.getBoundingBoxes(img)
        if len(boundingBoxes) > 0:
            self.trackingFace = fdl.selectFaceToTrack(boundingBoxes, img)
            self.mode = self.MODE_TRACKING

    def faceLost(self):
        self.publishFaceLost()
        self.mode = self.MODE_SCANNING
        self.trackingFace = None

    def trackingMode(self, img):
        boundingBoxes = self.fdm.getBoundingBoxes(imp)

        if len(boundingBoxes) == 0:
            self.faceLost()
        else:
            matches = []
            for box in boundingBoxes:
                match, eigenface = self.fdm.facesMatch(box, img, self.trackingFace)
                if match:
                    matches.append((box, eigenface))

            if len(matches) == 0:
                self.faceLost()

            elif self.fdm.faceIsCentred(matches[0][0], img):
                faceID = self.fdm.getFaceID(matches[0][1])
                if faceID is None:
                    self.publishNewFaceLocked(matches[0][1])
                else:
                    self.publishStudentFaceLocked(faceID)

            else:
                self.publishTrackFace(matches[0][0])

    def lockedMode(self, img_np):
        faceBoundingBox = fdl.findFaceInImage(img_np, self.trackingFace)

        if faceBoundingBox is None:
            self.mode = self.MODE_SCANNING
            self.publishFaceLost()

    def publishTrackFace(self, boundingBox):
        trackFaceMsg = TrackFace()
        trackFaceMsg.top = boundingBox[0]
        trackFaceMsg.right = boundingBox[1]
        trackFaceMsg.bottom = boundingBox[2]
        trackFaceMsg.left = boundingBox[3]
        self.trackFacePub.publish(trackFaceMsg)

    def publishStudentFaceLocked(self, studentID):
        faceLockedMsg = StudentFaceLocked()
        faceLockedMsg.studentID = studentID
        self.studentFaceLockedPub.publish(faceLockedMsg)

    def publishNewFaceLocked(self, eigenface):
        newFaceMsg = NewFaceLocked()
        newFaceMsg.eigenface = eigenface
        self.newFaceLockedPub.publish(newFaceMsg)

    def publishFaceLost(self):
        self.faceLostPub.Publish(Empty())

if __name__ == "__main__":
    rospy.init_node(name="face_detect")
    fh = FaceHandler()
    rospy.spin()
