import rospy
import roslib
roslib.load_manifest("sound_play")
from sound_play.libsoundplay import SoundClient

class SpeechView:
    def __init__(self):
        rospy.init_node('say', anonymous=True)
        self.presenter = None
        self.soundHandle = SoundClient()

    def greeting(self, name):
        self.soundHandle.say("hello in {}".format(name))
        
    def setPresenter(self, presenter):
        self.presenter = presenter

    def deliverNag(self, lectureName, location):
        self.soundHandle.say("You should be in {} at {}".format(lectureName, location))
        print("You should be in {} at {}".format(lectureName, location))

    def promptForID(self):
        self.soundHandle.say("please enter your i d")
        print("Please enter your id")
        self.presenter.notifyIDSubmitted(idInput)

    def killInteraction(self):
        pass

    def warn(self, warning):
        print("Warning in UI: {}".format(warning))
