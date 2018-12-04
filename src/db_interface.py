import sqlite3

class DB_Interface:
    def __init__(self):
        self.conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        self.c = self.conn.cursor()

    def __del__(self):
        self.conn.close()

    def tuple_to_string(self, t):
        if t is None:
            return None
        return ','.join(str(p) for p in t)

    def string_to_tuple(self, s):
        if s is None:
            return None
        return tuple(float(p) for p in s.split(','))

    def getStudentCurrentLecture(self, studentID):
        # ID of lecture that student should be in, if any
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                SELECT id
                FROM Lecture
                WHERE moduleid IN (
                    SELECT moduleid
                    FROM enrollment
                    WHERE studentid = ?
                ) AND strftime('%w', 'now') = day
                AND strftime('%H:%M','now') BETWEEN starttime AND endtime
                """, (studentID,))
        result = c.fetchone()
        conn.close()
        return result

    def storeAbsence(self, studentID, lectureID):
        # Make a note in the database that the student was not at this lecture
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("INSERT INTO Missed(lectureid, studentid) VALUES (?,?)", (lectureID, studentID))
        conn.close()
        pass

    def getLectureNameAndLocation(self, lectureID):
        # Name and location of lecture from DB
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT location, name FROM Lecture   
                    JOIN Module
                    ON Lecture.moduleid = Module.id
                    WHERE Lecture.id=?
                    """, (lectureID,))
        result = c.fetchone()
        conn.close()   
        return result

    # def getLectureInfo(self, lectureID):
    #     conn = sqlite3.connect('../database.db', isolation_level=None)
    #     c = conn.cursor()
    #     c.execute("""
    #                 SELECT Module.name, 
    #     """)
    #     conn.close()

    # def getLectureInfo(self, lectureID):
    #     conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
    #     c = conn.cursor()
    #     c.execute("""
    #                 SELECT Module.name, 
    #     """)
    #     conn.close()

    def storeNewStudent(self, studentID, eigenface, name=None):
        # Store a new student, with their face, in the DB
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("INSERT OR IGNORE INTO Student(id, eigenface) VALUES (?,?)", (studentID, self.tuple_to_string(eigenface)))
        c.execute("UPDATE Student SET eigenface = ? WHERE id = ?", (self.tuple_to_string(eigenface), studentID))
        if name != None:
            c.execute("UPDATE Student SET name = ? WHERE id = ?", (name, studentID))
        conn.close()
        pass

    def getNewFaces(self, existingStudentIDs):
        # out of the list of existing student IDs needs to select student.id student.eigenface from student where student.id not in existing student ids
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT id, eigenface FROM Student
                    WHERE id NOT IN (%s)
        """ % ','.join('?' * len(existingStudentIDs)), existingStudentIDs)
        result = c.fetchall()
        conn.close()
        return [(i, self.string_to_tuple(f)) for i, f in result]

    def getStudentName(self, studentID):
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("SELECT name FROM Student WHERE id = ?", (studentID,))
        result = c.fetchone()
        conn.close()
        return result

    def getLecturer(self, lecturerID):
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT name, email FROM Lecturer
                    WHERE id = ?
        """, (lecturerID,))
        result = c.fetchone()
        conn.close()
        return result

    def getAbsences(self, lecturerID, time='-3 Hour'):
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT Missed.studentid, Student.name, Module.name, Module.id, Lecture.id, Lecture.starttime, Lecture.endtime, Lecture.location, Missed.datetime FROM Missed
                    JOIN Lecture ON Missed.lectureid = Lecture.id
                    JOIN Student ON Missed.studentid = Student.id
                    JOIN Module ON Lecture.moduleid = Module.id
                    WHERE Module.lecturerid = ?
                    AND Missed.datetime >= datetime('now', ?)
                    ORDER BY Lecture.id
        """, (lecturerID,time))
        result = c.fetchall()
        conn.close()
        return result

    def getLectureAbsences(self, lectureID):
        # Return the students recorded as absent from a specified lecture within the last week.
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT Missed.studentid, Student.name, Missed.datetime FROM Missed
                    JOIN Student ON Missed.studentid = Student.id
                    WHERE Missed.lectureid = ?
                    AND Missed.datetime >= datetime('now', '-7 Day')
        """, (lectureID,))
        result = c.fetchall()
        return result

    def getLectureAbsences(self, lectureID):
        # Return the students recorded as absent from a specified lecture within the last week.
        conn = sqlite3.connect('/home/robot/catkin_ws/src/robots_ex_3/database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT Missed.studentid, Student.name, Missed.datetime FROM Missed
                    JOIN Student ON Missed.studentid = Student.id
                    WHERE Missed.lectureid = ?
                    AND Missed.datetime >= datetime('now', '-7 Day')
        """, (lectureID,))
        return self.c.fetchall()

if __name__ == "__main__":
    # Code to test the database interface 
    db = DB_Interface()
    #db.storeNewStudent(1549228, db.tuple_to_string((1.0,2.0,3.0)), "Bob")
    print(db.getStudentName(1549223))
    print(db.getStudentName(1549228))
    print(db.getNewFaces([]))
    db.storeAbsence(1549223, 1)
    db.storeAbsence(1549224, 1)
    db.storeAbsence(1549225, 1)
    db.storeAbsence(1549226, 1)
    db.storeAbsence(1549224, 2)
    db.storeAbsence(1549223, 3)
    db.storeAbsence(1549225, 1)
    db.storeAbsence(1549223, 4)
    db.storeAbsence(1549223, 5)
    print('added absences')
    print(db.getAbsences(1))
