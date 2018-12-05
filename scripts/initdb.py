# Script that will reset the database with testing data
# Warning: this will delete everything in the database
import sqlite3

conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db')
c = conn.cursor()

STUDENT_TABLE_NAME = "Student"
MODULE_TABLE_NAME = "Module"
ENROLLMENT_TABLE_NAME = "Enrollment"
LECTURE_TABLE_NAME = "Lecture"
LECTURER_TABLE_NAME = "Lecturer"
MISSED_LEC_TABLE_NAME = "Missed"

# Drop all tables
c.execute("DROP TABLE IF EXISTS " + STUDENT_TABLE_NAME)
c.execute("DROP TABLE IF EXISTS " + LECTURE_TABLE_NAME)
c.execute("DROP TABLE IF EXISTS " + LECTURER_TABLE_NAME)
c.execute("DROP TABLE IF EXISTS " + MISSED_LEC_TABLE_NAME)
c.execute("DROP TABLE IF EXISTS " + MODULE_TABLE_NAME)
c.execute("DROP TABLE IF EXISTS " + ENROLLMENT_TABLE_NAME)

# Enables foreign keys in SQLite
c.execute("PRAGMA foreign_keys = ON;")

# Re-create the tables
create_student_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY, 
                            name TEXT,
                            eigenface VARCHAR
)
""".format(STUDENT_TABLE_NAME)

create_lecturer_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            name TEXT,
                            email TEXT NOT NULL,
                            sms TEXT,
                            preferredcontact TEXT NOT NULL DEFAULT 'email'
)
""".format(LECTURER_TABLE_NAME)

create_module_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            name TEXT,
                            lecturerid INTEGER NOT NULL,
                            FOREIGN KEY(lecturerid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE
)
""".format(MODULE_TABLE_NAME, LECTURER_TABLE_NAME)

create_enrollment_query = """CREATE TABLE {}(
                                id INTEGER PRIMARY KEY,
                                studentid INTEGER NOT NULL,
                                moduleid INTEGER NOT NULL,
                                FOREIGN KEY(studentid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE,
                                FOREIGN KEY(moduleid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE
)
""".format(ENROLLMENT_TABLE_NAME, STUDENT_TABLE_NAME, MODULE_TABLE_NAME)


create_lecture_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            starttime DATETIME NOT NULL,
                            day DATETIME NOT NULL,
                            endtime DATETIME NOT NULL,
                            location TEXT NOT NULL,
                            moduleid INTEGER NOT NULL,
                            FOREIGN KEY(moduleid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE
)
""".format(LECTURE_TABLE_NAME, MODULE_TABLE_NAME)

create_missed_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            lectureid INTEGER NOT NULL,
                            studentid INTEGER NOT NULL,
                            datetime DATETIME DEFAULT current_timestamp,
                            FOREIGN KEY(lectureid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE,
                            FOREIGN KEY(studentid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE
)
""".format(MISSED_LEC_TABLE_NAME, LECTURE_TABLE_NAME, STUDENT_TABLE_NAME)

c.execute(create_student_query)
c.execute(create_lecturer_query)
c.execute(create_module_query)
c.execute(create_enrollment_query)
c.execute(create_lecture_query)
c.execute(create_missed_query)

# Some test data
c.execute("INSERT INTO %s(id, name) VALUES (1549223, 'Matt');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1522968, 'George');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1556067, 'Adriana');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1558595, 'Mike');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1559226, 'Joe');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(name, email, sms, preferredcontact) VALUES ('Mohan', 'mattcallaway1406@gmail.com', '+447538530766', 'sms');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, email, sms, preferredcontact) VALUES ('Bob', 'bob@bham.com', '+447867766564', 'sms');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, email) VALUES ('Alice', 'alice@bham.com');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Intelligent Robotics', 1);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('PPL', 2);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Maths', 1);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Security', 3);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1549223, 1);" % ENROLLMENT_TABLE_NAME) # Matt
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1549223, 2);" % ENROLLMENT_TABLE_NAME) # Matt
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1522968, 1);" % ENROLLMENT_TABLE_NAME) # George
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1522968, 2);" % ENROLLMENT_TABLE_NAME) # George
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1558595, 1);" % ENROLLMENT_TABLE_NAME) # Mike
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1558595, 3);" % ENROLLMENT_TABLE_NAME) # Mike
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1556067, 1);" % ENROLLMENT_TABLE_NAME) # Adriana
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('01:30', '3', '02:30', 'Room 101',1);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('01:00', '3', '02:30', 'Room 203',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('09:00', '2', '10:00', 'Room 304',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('13:00', '2', '14:00', 'Room 105',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('10:00', '3', '11:00', 'Room 205',1);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('10:00', '3', '11:00', 'Room 94',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('14:00', '3', '15:00', 'Room 87',1);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('15:00', '4', '16:00', 'Room 103',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('10:00', '4', '12:00', 'Room 120',3);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('09:00', '5', '10:00', 'Room 304',4);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('11:00', '5', '13:00', 'Room 305',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (1, 1549223);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (1, 1522968);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (1, 1558595);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (1, 1559226);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (2, 1558595);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (3, 1549223);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (4, 1558595);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (1, 1556067);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (2, 1556067);" % MISSED_LEC_TABLE_NAME)
c.execute("INSERT INTO %s(lectureid, studentid) VALUES (3, 1556067);" % MISSED_LEC_TABLE_NAME)




conn.commit()
conn.close()



