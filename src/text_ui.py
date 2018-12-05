import os

class TextView:
    def __init__(self):
        self.presenter = None

    def setPresenter(self, presenter):
        self.presenter = presenter

    def deliverNag(self, lectureName, location):
        msg = "You should be in {} at {}".format(lectureName, location)
        print(msg)
        self.say(msg)

    def promptForID(self):
        self.say("please enter your i d on the laptop")
        idInput = raw_input("Please enter your ID")
        self.presenter.notifyIDSubmitted(idInput)

    def killInteraction(self):
        pass

    def warn(self, warning):
        msg = "Warning in U I: {}".format(warning)
        self.say(msg)
        print(msg)

    def say(self, msg):
        os.system("say \"{}\"".format(msg))
