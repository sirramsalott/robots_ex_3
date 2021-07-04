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
    MODE_SCANNING = 0
    MODE_TRACKING = 1
    MODE_LOCKED = 2
    MODE_WAITING = 3
    FRAME_RATE = 10
    FACE_LOST_THRESHOLD = 10
    WAIT_THRESHOLD = 20
    
    def __init__(self):
        self.framesSinceLastFace = self.FACE_LOST_THRESHOLD + 1
        self.mode = self.MODE_SCANNING
        self.trackingFace = None
        self.framesReceived = 0

        self.visualise = rospy.get_param("/visualise")

        self.fdm = FaceDetectionModel()

        self.trackFacePub = rospy.Publisher("track_face", TrackFace, queue_size=10)
        self.studentFaceLockedPub = rospy.Publisher("student_face_locked", StudentFaceLocked, queue_size=1)
        self.newFaceLockedPub = rospy.Publisher("new_face_locked", NewFaceLocked, queue_size=1)
        self.faceLostPub = rospy.Publisher("face_lost", Empty, queue_size=1)
        self.facePendPub = rospy.Publisher("face_pend", Empty, queue_size=1)
        self.startTrackingPub = rospy.Publisher("start_tracking_face", Empty, queue_size=1)

        self.imageSub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCallback, queue_size=1)
        self.interactionCompleteSub = rospy.Subscriber("/interaction_complete", Empty, self.completeCallback, queue_size=1)
        self.newFaceAddedSub = rospy.Subscriber("new_face_added", Empty, self.newFaceCallback, queue_size=1)

        self.bridge = CvBridge()
        self.framesInWaitingMode = 0

    def completeCallback(self, msg):
        self.mode = self.MODE_WAITING
        self.trackingFace = None

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

            elif self.mode == self.MODE_WAITING:
                img = self.waitingMode(img)

            if self.visualise:
                cv2.imshow("Camera Stream", img)
                cv2.waitKey(1)

    def waitingMode(self, img):
        self.framesInWaitingMode += 1
        if self.framesInWaitingMode >= self.WAIT_THRESHOLD:
            self.mode = self.MODE_SCANNING
            self.framesInWaitingMode = 0

        return img

    def newFaceCallback(self, msg):
        self.fdm.updateFaceDBCache()

    def scanningMode(self, img):
        boundingBoxes = self.fdm.getBoundingBoxes(img)
        print(boundingBoxes)
        if len(boundingBoxes) > 0:
            self.startTrackingPub.publish(Empty())
            self.trackingFace = self.fdm.selectFaceToTrack(boundingBoxes, img)
            self.mode = self.MODE_TRACKING

        for box in boundingBoxes:
            img = self.drawBoundingBox(img, box, colour=(0, 0, 255))

        return img

    def trackingMode(self, img):
        boundingBoxes = self.fdm.getBoundingBoxes(img)

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
            else:
                self.framesSinceLastFace = 0
                box = matches[0][0]
                face = matches[0][1]
                img = self.drawBoundingBox(img, box,
                                           colour=(0, 255, 0))

                if self.fdm.faceIsCentred(box, img):
                    self.mode = self.MODE_LOCKED
                    faceID = self.fdm.getFaceID(face)
                    if faceID is None:
                        self.publishNewFaceLocked(face)
                    else:
                        self.publishStudentFaceLocked(faceID)
                else:
                    self.publishTrackFace(box)

        return img

    def lockedMode(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    def faceLost(self):
        self.framesSinceLastFace += 1
        if self.framesSinceLastFace <= self.FACE_LOST_THRESHOLD:
            self.publishFacePend()
        else:
            self.publishFaceLost()
            self.mode = self.MODE_SCANNING
            self.trackingFace = None
            self.framesSinceLastFace = 0

    def publishTrackFace(self, boundingBox):
        trackFaceMsg = TrackFace()
        trackFaceMsg.top = boundingBox[0]
        trackFaceMsg.right = boundingBox[1]
        trackFaceMsg.bottom = boundingBox[2]
        trackFaceMsg.left = boundingBox[3]
        self.trackFacePub.publish(trackFaceMsg)

    def publishStudentFaceLocked(self, studentID):
        faceLockedMsg = StudentFaceLocked()
        faceLockedMsg.studentID = str(studentID)
        self.studentFaceLockedPub.publish(faceLockedMsg)

    def publishNewFaceLocked(self, eigenface):
        newFaceMsg = NewFaceLocked()
        newFaceMsg.eigenface = eigenface
        self.newFaceLockedPub.publish(newFaceMsg)

    def publishFaceLost(self):
        self.faceLostPub.publish(Empty())

    def publishFacePend(self):
        self.facePendPub.publish(Empty())

    def drawBoundingBox(self, img, box, colour):
        top, right, bottom, left = box
        return cv2.rectangle(img,
                             pt1=(left, top),
                             pt2=(right, bottom),
                             color=colour,
                             thickness=3)

if __name__ == "__main__":
    rospy.init_node(name="face_detect")
    fh = FaceHandler()
    rospy.spin()
