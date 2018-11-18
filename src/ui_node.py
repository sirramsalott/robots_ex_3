#! /usr/bin/env python

import rospy

from std_msgs.msg import Empty
from robots_exercise_3.msg import FaceLocked

import ui_lib as ui

class UI:
    def __init__(self):
        self.faceLockedSub = rospy.Subscriber("face_locked",
                                              FaceLocked,
                                              self.faceLockedCallback,
                                              queue=1)
        self.faceLostSub = rospy.Subscriber("face_lost",
                                            Empty,
                                            self.faceLostCallback,
                                            queue=1)

        self.interactionCompletePub = rospy.Publisher("interaction_complete",
                                                      Empty)

    def faceLockedCallback(self, msg):
        if msg.type == msg.TYPE_STUDENT:
            ui.nagStudent(msg.id)
        elif msg.type == msg.TYPE_STAFF:
            ui.reportAbsentees(msg.id)
        elif msg.type == msg.TYPE_UNKNOWN:
            # need to know face representation and add it to the message
            # before we can do this one
            pass

    def faceLostCallback(self, msg):
        # need some way to kill interaction if the face is lost
        pass

    def publishInteractionComplete(self):
        self.interactionCompletePub.publish(Empty())


if __name__ == "__main__":
    rospy.init_node("ui")
    ui = UI()
    rospy.spin()
