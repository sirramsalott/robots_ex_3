import rospy
import os

class SpeechView:
    def __init__(self):

        self.presenter = None

    def greeting(self, name):
        self.os.system("say \"hello {}\"".format(name))
        
    def setPresenter(self, presenter):
        self.presenter = presenter

    def deliverNag(self, lectureName, location):
        msg = "You should be in {} at {}".format(lectureName, location)
        print(msg)
        os.system("say \"{}\"".format(msg))

    def promptForID(self):
        os.system("say \"please enter your i d\"")
        idInput = raw_input("Please enter your id")
        self.presenter.notifyIDSubmitted(idInput)

    def killInteraction(self):
        pass

    def warn(self, warning):
        os.system("say \"{}\"".format(warning))
        print("Warning in UI: {}".format(warning))
