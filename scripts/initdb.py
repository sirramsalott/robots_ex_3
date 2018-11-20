# Script that will reset the database with testing data
# Warning: this will delete everything in the database
import sqlite3

conn = sqlite3.connect('database.db')

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
                            name TEXT
)
""".format(STUDENT_TABLE_NAME)

create_lecturer_query = """CREATE TABLE {}(
                            id INTEGER PRIMARY KEY,
                            name TEXT
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
                            time TEXT NOT NULL,
                            day TEXT NOT NULL,
                            duration INTEGER NOT NULL DEFAULT 60,
                            moduleid INTEGER NOT NULL,
                            FOREIGN KEY(moduleid) REFERENCES {}(id) ON DELETE CASCADE ON UPDATE CASCADE
)
""".format(LECTURE_TABLE_NAME, MODULE_TABLE_NAME)

c.execute(create_student_query)
c.execute(create_lecturer_query)
c.execute(create_module_query)
c.execute(create_enrollment_query)
c.execute(create_lecture_query)

# Some test data
c.execute("INSERT INTO %s(id, name) VALUES (1549223, 'Matt');" % STUDENT_TABLE_NAME)
c.execute("INSERT INTO %s(name) VALUES ('Mohan');" % LECTURER_TABLE_NAME)
c.execute("INSERT INTO %s(name, lecturerid) VALUES ('Intelligent Robotics', 1);" % MODULE_TABLE_NAME)
c.execute("INSERT INTO %s(studentid, moduleid) VALUES (1549223, 1);" % ENROLLMENT_TABLE_NAME)
c.execute("INSERT INTO %s(time, day, moduleid) VALUES ('14:00', 'Mon', 1);" % LECTURE_TABLE_NAME)

conn.commit()
conn.close()



