from db_interface import DB_Interface
import smtplib
from email.message import EmailMessage

gmail_user = "stuart.the.snitch@gmail.com"
# Email account password redacted for git
# You must first change the below to the password for anything to send!!
gmail_pass = None 

class Comm_Sender:
    def __init__(self, dbif : DB_Interface):
        self.dbif = dbif

    def emailLecturer(self, lecturerID, time):
    # Email lecturer %ID% about all students missing lecture in the last %TIME%
    # Email format will be:
    # Hi %Lecturer%,
    # These students have been caught skipping your lectures in the last %TIME%
    # %MODULE_NAME%:
    #   Lecture %STARTTIME%-%ENDTIME%, %LOCATION%:
    #       %STUDENTNAME% - %STUDENTID% seen at %TIMEOFINCIDENT%
    # Keep up the good work!
    # - Stuart the Snitch
        absences = self.dbif.getAbsences(lecturerID)
        lecturer = self.dbif.getLecturer(lecturerID)
        content = "Hi %s\nThese students have been caught skipping your lectures in the last %s\n" % (lecturer[0], time)
        lastmoduleid = None
        lastlectureid = None
        for a in absences:
            if a[3] != lastmoduleid:
                lastmoduleid = a[3]
                content = content + "\tModule: %s\n" % a[2]
            if a[4] != lastlectureid:
                lastlectureid = a[4]
                content = content + "\t\tLecture %s-%s, %s:\n" % (a[5], a[6], a[7])
            content = content + "\t\t\t%s - %s seen at %s\n" % (a[1], a[0], a[8])
        
        msg = EmailMessage()
        msg['Subject'] = 'Snitch Report'
        msg['From'] = 'Stuart the Snitch'
        msg['To'] = lecturer[1]
        msg.set_content(content)
        try:  
            server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
            server.ehlo()
            server.login(gmail_user, gmail_pass)
            server.send_message(msg)
            server.close()
            print('Email sent!')
        except:  
            print('Something went wrong...')


if __name__=="__main__":
    db = DB_Interface()
    sender = Comm_Sender(db)
    sender.emailLecturer(1, '-3 Hour')
