import rospy
import os
from std_msgs.msg import String

class SpeechView:

    def __init__(self):
        self.presenter = None
        self.listening_for_id = True
        self.checking = False
        self.got_id = False
        self.counter = 0
        self.word = ""
        self.number_subscriber = rospy.Subscriber("/recognizer/output",
                                                String,
                                                self.number_subscriber_callback,
                                                queue_size=1)

    def greeting(self, name):
        os.system("say \"hello {}\"".format(name))
        
    def setPresenter(self, presenter):
        self.presenter = presenter

    def deliverNag(self, lectureName, location):
        msg = "You should be in {} at {}".format(lectureName, location)
        print(msg)
        os.system("say \"{}\"".format(msg))

    def promptForID(self):
        os.system("say \"please enter your i d\"")
        rospy.loginfo("please enter your id")
        self.listening_for_id = True
        #idInput = raw_input("Please enter your id")

    def number_subscriber_callback(self, msg):
        if self.listening_for_id and msg.data == "clear":
            self.word = ""
            self.counter = 0
            rospy.loginfo(self.word)
        elif self.listening_for_id and msg.data == "delete" and self.counter > 0:
            self.word = self.word[:-1]
            self.counter -= 1
            rospy.loginfo(self.word)
        elif self.listening_for_id and self.counter < 7 and len(msg.data) == 1:
            self.counter += 1
            self.word += msg.data
            rospy.loginfo(self.word)
        if self.counter == 7 and self.checking == False:
            os.system("say \"Is this your i d {} \"".format(self.word))
            rospy.loginfo("Is this your id?")
            self.checking = True
            self.listening_for_id = False
        if self.checking:
            if msg.data == "yes":
                self.got_id = True
                self.checking = False
                self.counter = 0
                os.system("say \"thank you\"")
            if msg.data == "no":
                os.system("say \"please type in your student i d\"")
                self.word = raw_input("please enter your student id ")
                self.got_id = True
                self.checking = False
                self.counter = 0
                os.system("say \"oh i see\"")
        if self.got_id:
            self.presenter.notifyIDSubmitted(self.word)
            self.word = ""
            self.listening_for_id = False
            self.counter = 0
            self.got_id = False

    def killInteraction(self):
        pass

    def warn(self, warning):
        os.system("say \"{}\"".format(warning))
        print("Warning in UI: {}".format(warning))
