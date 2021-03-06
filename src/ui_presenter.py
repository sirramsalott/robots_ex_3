import db_interface as db
import rospy
from std_msgs.msg import Empty

class UIPresenter:
    def __init__(self, view):
        self.cachedEigenface = None
        # TODO: change TextViewTk to speech interface
        # (it create SpeechView to match the interface implemented by TextViewTk)
        self.view = view
        self.view.setPresenter(self)
        self.dbHandle = db.DB_Interface()
        self.interactionCompletePub = rospy.Publisher("/interaction_complete",
                                                      Empty,
                                                      queue_size=1)

    def nagStudent(self, studentID):
        lectureID = self.dbHandle.getStudentCurrentLecture(studentID)
        print("Student recognised: {}!".format(studentID))
        if lectureID is not None:
            print("student id of type {}: {}".format(type(studentID), studentID))
            print("lecture id of type {}: {}".format(type(lectureID), lectureID))
            self.dbHandle.storeAbsence(studentID, lectureID)
            lectureName, location = self.dbHandle.getLectureNameAndLocation(lectureID)
            self.view.deliverNag(lectureName, location)

    def newUser(self, eigenface):
        print("EIGENFACE: {}".format(eigenface))
        self.cachedEigenface = eigenface
        self.view.promptForID()

    def killInteraction(self):
        #self.cachedEigenface = None
        self.view.killInteraction()
        self.interactionCompletePub.publish(Empty())

    def notifyIDSubmitted(self, studentID):
        # THIS IS THE ONLY METHOD ON PRESENTER THAT VIEW SHOULD CALL
        rospy.loginfo("ID Submitted {}".format(studentID))
        if self.cachedEigenface is None:
            self.view.warn("StudentID submitted, but no face cached on presenter")
        else:
            self.dbHandle.storeNewStudent(studentID, self.cachedEigenface)
            self.cachedEigenface = None
            self.interactionCompletePub.publish(Empty())
