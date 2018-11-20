#! /usr/bin/env python

import rospy

from std_msgs.msg import Empty
from robots_exercise_3.msg import FaceLocked

import ui_lib as ui

class UI:
    def __init__(self):
        self.studentFaceLockedSub = rospy.Subscriber("student_face_locked",
                                              StudentFaceLocked,
                                              self.studentFaceLockedCallback,
                                              queue_size=1)
        self.newFaceLockedSub = rospy.Subscriber("new_face_locked",
                                                 NewFaceLocked,
                                                 self.newFaceLockedCallback,
                                                 queue_size=1)
        self.faceLostSub = rospy.Subscriber("face_lost",
                                            Empty,
                                            self.faceLostCallback,
                                            queue_size=1)

        self.interactionCompletePub = rospy.Publisher("interaction_complete",
                                                      Empty)
        self.newFaceAddedPub = rospy.Publisher("new_face_added",
                                               Empty)

    def studentFaceLockedCallback(msg):
        ui.nagStudent(msg.studentID)
        self.publishInteractionComplete()

    def newFaceLockedCallback(self, msg):
        # TODO: work out FSM for this state

    def faceLostCallback(self, msg):
        # need some way to kill interaction if the face is lost
        pass

    def publishInteractionComplete(self):
        self.interactionCompletePub.publish(Empty())


if __name__ == "__main__":
    rospy.init_node("ui")
    ui = UI()
    rospy.spin()
