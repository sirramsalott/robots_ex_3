class TextView:
    def __init__(self):
        self.presenter = None

    def setPresenter(self, presenter):
        self.presenter = presenter

    def deliverNag(self, lectureName, location):
        print("You should be in {} at {}".format(lectureName, location))

    def promptForID(self):
        idInput = raw_input("Please enter your ID")
        self.presenter.notifyIDSubmitted(idInput)

    def killInteraction(self):
        pass

    def warn(self, warning):
        print("Warning in UI: {}".format(warning))
