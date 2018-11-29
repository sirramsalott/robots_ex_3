from db_interface import DB_Interface
import smtplib
from email.message import EmailMessage
from twilio.rest import Client

# Email authentication info
gmail_user = "stuart.the.snitch@gmail.com"
# Email account password redacted for git
# You must first change the below to the password for anything to send!!
gmail_pass = None

# Twilio auth info
# Your Account SID from twilio.com/console
account_sid = "AC0c22ab95bd91d211f05f3700ccd5d4a7"
# Your Auth Token from twilio.com/console
# You must first change the below to the authentication token for SMS to send
auth_token  = None



class Comm_Sender:
    def __init__(self, dbif : DB_Interface):
        self.dbif = dbif
        self.client = Client(account_sid, auth_token)

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
        content = content + "Keep up the good work!\n- Stuart the Snitch\n"
        
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

    def smsLecturer(self, lecturerID, lectureID):
        lecturer = self.dbif.getLecturer(lecturerID)
        lecture = self.dbif.getLectureNameAndLocation(lectureID)
        absences = self.dbif.getLectureAbsences(lectureID)
        content = "Hi %s! This is Stuart the Snitch reporting on your last %s lecture, these students were caught skipping:\n" % (lecturer[0], lecture[1])
        for a in absences:
            content = content + "\t%s - %s, %s\n" % (a[1], a[0], a[2])
        message = self.client.messages.create(
            to="+447538530766",
            from_="+441384686183",
            body=content
        )
        print(message.sid)



if __name__=="__main__":
    db = DB_Interface()
    sender = Comm_Sender(db)
    # sender.emailLecturer(1, '-3 Hour')
    sender.smsLecturer(1, 1)