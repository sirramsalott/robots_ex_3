import db_interface as db


class UIPresenter:
    def __init__(self, view):
        self.cachedEigenface = None
        # TODO: change TextViewTk to speech interface
        # (it create SpeechView to match the interface implemented by TextViewTk)
        self.view = view
        self.view.setPresenter(self)

    def nagStudent(self, studentID):
        lectureID = db.getStudentCurrentLecture(studentID)

        if lectureID is not None:
            db.storeAbsence(studentID, lectureID)
            lectureName, location = db.getLectureNameAndLocation(lectureID)
            self.view.deliverNag(lectureName, location)

    def newUser(self, eigenface):
        self.cachedEigenface = eigenface
        self.view.promptForID()

    def killInteraction(self):
        self.cachedEigenface = None
        self.view.killInteraction()

    def notifyIDSubmitted(self, studentID):
        # THIS IS THE ONLY METHOD ON PRESENTER THAT VIEW SHOULD CALL
        if self.cachedEigenface is None:
            self.view.warn("StudentID submitted, but no face cached on presenter")
        else:
            db.storeNewStudent(studentID, self.cachedEigenface)
            self.cachedEigenface = None
