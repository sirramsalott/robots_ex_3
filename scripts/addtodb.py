import sqlite3
import sys

conn = sqlite3.connect('../database.db', isolation_level=None)
c = conn.cursor()

days = {
    'Sun':0,
    'Mon':1,
    'Tue':2,
    'Wed':3,
    'Thu':4,
    'Fri':5,
    'Sat':6
    }

def addLecturer(name, pref, email=None, sms=None):
    if pref != 'email' and pref != 'sms':
        return None
    if pref == 'email' and email != None:    
        c.execute("INSERT INTO Lecturer(name, email, preferredcontact) VALUES (?, ?, ?);", (name, email, pref))
    elif pref == 'sms' and sms != None:
        c.execute("INSERT INTO Lecturer(name, sms, preferredcontact) VALUES (?, ?, ?);", (name, sms, pref))
    else:
        return None
    return c.lastrowid

def addModule(name, lecturerid):
    c.execute("INSERT INTO Module(name, lecturerid) VALUES (?, ?);", (name, lecturerid))
    return c.lastrowid

def addStudent(name, studentid):
    c.execute("INSERT INTO Student(id, name) VALUES (?, ?);", (studentid, name))
    return c.lastrowid

def enrollStudent(studentid, moduleid):
    c.execute("INSERT INTO Enrollment(studentid, moduleid) VALUE (?, ?);", (studentid, moduleid))

def addLecture(starttime, day, endtime, location, moduleid):
    c.execute("INSERT INTO Lecture(starttime, day, endtime, location, moduleid) VALUES (?, ?, ?, ?, ?);", (starttime, days[day], endtime, location, moduleid))
    return c.lastrowid

def printUsage():
    print("Usage: " + sys.argv[0] + "\n\t" + 
            "lecturer <name> <preferredcontact = 'email'/'sms'> <contactvalue>\n\t" +
            "module <name> <lecturerid>\n\t" +
            "student <name> <studentid>\n\t" +
            "enrollment <studentid> <moduleid>\n\t" +
            "lecture <starttime = 'HH:MM'> <day = 'Sun'/'Mon'/'Tue'/'Wed'/'Thu'/'Fri'/'Sat'> <endtime = 'HH:MM'> <location> <moduleid>\n")

if len(sys.argv) < 2:
    printUsage()
else:
    mode = sys.argv[1]

    if mode == 'lecturer':
        if len(sys.argv) != 5:
            printUsage()
        elif sys.argv[3] == 'email':
            id = addLecturer(sys.argv[2], sys.argv[3], email=sys.argv[4])
            print("Added new lecturer with id: " + str(id))
        elif sys.argv[3] == 'sms':
            id = addLecturer(sys.argv[2], sys.argv[3], sms=sys.argv[4])
            print("Added new lecturer with id: " + str(id))
        else:
            printUsage()
    elif mode == 'module':
        if len(sys.argv) != 4:
            printUsage()
        else:
            id = addModule(sys.argv[2], int(sys.argv[3]))
            print("Added new module with id: " + str(id))
    elif mode == 'student':
        if len(sys.argv) != 4:
            printUsage()
        else:
            id = addStudent(sys.argv[2], int(sys.argv[3]))
            print("Added new student with id: " + str(id))
    elif mode == 'enrollment':
        if len(sys.argv) != 4:
            printUsage()
        else:
            addEnrollment(sys.argv[2], int(sys.argv[3]))
            print("Added new enrollment")
    elif mode == 'lecture':
        if len(sys.argv) != 7:
            printUsage()
        else:
            id = addLecture(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], int(sys.argv[6]))
            print("Added new lecture with id: " + str(id))
    else:
        printUsage()



