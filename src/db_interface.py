import sqlite3

class DB_Interface:
    def __init__(self):
        self.conn = sqlite3.connect('../database.db', isolation_level=None)
        self.c = self.conn.cursor()

    def __del__(self):
        self.conn.close()

    def getStudentCurrentLecture(self, studentID):
        # ID of lecture that student should be in, if any
        conn = sqlite3.connect('../database.db', isolation_level=None)
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
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("INSERT INTO Missed(lectureid, studentid) VALUES (?,?)", (lectureID, studentID))
        conn.close()
        pass

    def getLectureNameAndLocation(self, lectureID):
        # Name and location of lecture from DB
        conn = sqlite3.connect('../database.db', isolation_level=None)
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

    def storeNewStudent(self, studentID, eigenface, name=None):
        # Store a new student, with their face, in the DB
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("INSERT OR IGNORE INTO Student(id, eigenface) VALUES (?,?)", (studentID, eigenface))
        c.execute("UPDATE Student SET eigenface = ? WHERE id = ?", (eigenface, studentID))
        if name != None:
            c.execute("UPDATE Student SET name = ? WHERE id = ?", (name, studentID))
        conn.close()
        pass

    def getNewFaces(self, existingStudentIDs):
        # out of the list of existing student IDs needs to select student.id student.eigenface from student where student.id not in existing student ids
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT id, eigenface FROM Student
                    WHERE id NOT IN (%s)
        """ % ','.join('?' * len(existingStudentIDs)), existingStudentIDs)
        result = c.fetchall()
        conn.close()
        return result

    def getStudentName(self, studentID):
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("SELECT name FROM Student WHERE id = ?", (studentID,))
        result = c.fetchone()
        conn.close()
        return result

    def getLecturer(self, lecturerID):
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        c.execute("""
                    SELECT name, email, sms FROM Lecturer
                    WHERE id = ?
        """, (lecturerID,))
        result = c.fetchone()
        conn.close()
        return result

    def getAbsences(self, lecturerID=None, time='-3 Hour'):
        conn = sqlite3.connect('../database.db', isolation_level=None)
        c = conn.cursor()
        if lecturerID != None:
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
        else:
            c.execute("""
                        SELECT DISTINCT Module.lecturerid, Lecturer.preferredcontact  FROM Missed
                        JOIN Lecture ON Missed.lectureid = Lecture.id
                        JOIN Module ON Lecture.moduleid = Module.id
                        JOIN Lecturer ON Module.lecturerid = Lecturer.id
                        WHERE Missed.datetime >= datetime('now', ?)
            """, (time,))
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


if __name__ == "__main__":
    # Code to test the database interface 
    db = DB_Interface()
    print(db.getAbsences())
