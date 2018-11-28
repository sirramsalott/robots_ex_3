# Script that will reset the database with testing data
# Warning: this will delete everything in the database
import sqlite3

conn = sqlite3.connect('../database.db')

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
                            eigenface BLOB
)
""".format(STUDENT_TABLE_NAME)

create_lecturer_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            name TEXT,
                            email TEXT
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
c.execute("INSERT INTO %s(id, name) VALUES (1549224, 'George');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1549225, 'Adriana');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1549226, 'Mike');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(id, name) VALUES (1549227, 'Joe');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(name, email) VALUES ('Mohan', 'mattcallaway1406@gmail.com');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, email) VALUES ('Bob', 'bob@bham.com');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, email) VALUES ('Alice', 'alice@bham.com');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Intelligent Robotics', 1);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('PPL', 1);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Maths', 2);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1549223, 1);" % ENROLLMENT_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('02:00', '2', '03:00', 'Room 101',1);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('00:00', '2', '01:00', 'Room 101',1);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('09:00', '3', '10:00', 'Room 101',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('13:00', '3', '14:00', 'Room 101',2);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('07:00', '3', '10:00', 'Room 101',3);" % LECTURE_TABLE_NAME)
c.execute("INSERT INTO %s(starttime, day, endtime, location, moduleid) VALUES ('11:00', '3', '14:00', 'Room 101',3);" % LECTURE_TABLE_NAME)

conn.commit()
conn.close()



