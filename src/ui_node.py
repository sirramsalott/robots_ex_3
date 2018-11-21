#! /usr/bin/env python

import rospy

from std_msgs.msg import Empty
from robots_exercise_3.msg import StudentFaceLocked, NewFaceLocked

from ui_presenter import UIPresenter
from text_ui import TextView


class UIHandler:
    def __init__(self, view):
        self.UIPresenter = UIPresenter(view)

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
                                                      Empty,
                                                      queue_size=1)
        self.newFaceAddedPub = rospy.Publisher("new_face_added",
                                               Empty,
                                               queue_size=1)

    def studentFaceLockedCallback(self, msg):
        self.UIPresenter.nagStudent(msg.studentID)
        self.publishInteractionComplete()

    def newFaceLockedCallback(self, msg):
        self.UIPresenter.newUser(msg.eigenface)
        self.newFaceAddedPub.publish(Empty())

    def faceLostCallback(self, msg):
        self.UIPresenter.killInteraction()

    def publishInteractionComplete(self):
        self.interactionCompletePub.publish(Empty())


if __name__ == "__main__":
    view = TextView()
    uiHandler = UIHandler(view)
    rospy.init_node("ui")
    rospy.spin()
