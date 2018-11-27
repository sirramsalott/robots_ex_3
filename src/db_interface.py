import sqlite3

class DB_Interface:
    def __init__(self):
        self.conn = sqlite3.connect('../database.db', isolation_level=None)
        self.c = self.conn.cursor()

    def __del__(self):
        self.conn.close()

    def getStudentCurrentLecture(self, studentID):
        # ID of lecture that student should be in, if any
        self.c.execute("""
                SELECT id
                FROM Lecture
                WHERE moduleid IN (
                    SELECT moduleid
                    FROM enrollment
                    WHERE studentid = ?
                ) AND strftime('%w', 'now') = day
                AND strftime('%H:%M','now') BETWEEN starttime AND endtime
                """, (studentID,))
        return self.c.fetchone()

    def storeAbsence(self, studentID, lectureID):
        # Make a note in the database that the student was not at this lecture
        self.c.execute("INSERT INTO Missed(lectureid, studentid) VALUES (?,?)", (lectureID, studentID))
        pass

    def getLectureNameAndLocation(self, lectureID):
        # Name and location of lecture from DB
        self.c.execute("""
                    SELECT location, name FROM Lecture   
                    JOIN Module
                    ON Lecture.moduleid = Module.id
                    WHERE Lecture.id=?
                    """, (lectureID,))   
        return self.c.fetchone()

    def storeNewStudent(self, studentID, eigenface, name=None):
        # Store a new student, with their face, in the DB
        self.c.execute("INSERT OR IGNORE INTO Student(id, eigenface) VALUES (?,?)", (studentID, eigenface))
        self.c.execute("UPDATE Student SET eigenface = ? WHERE id = ?", (eigenface, studentID))
        if name != None:
            self.c.execute("UPDATE Student SET name = ? WHERE id = ?", (name, studentID))
        pass

    def getNewFaces(self, existingStudentIDs):
        # out of the list of existing student IDs needs to select student.id student.eigenface from student where student.id not in existing student ids
        self.c.execute("""
                    SELECT id, eigenface FROM Student
                    WHERE id NOT IN (%s)
        """ % ','.join('?' * len(existingStudentIDs)), existingStudentIDs)
        return self.c.fetchall()

    def getStudentName(self, studentID):
        self.c.execute("SELECT name FROM Student WHERE id = ?", (studentID,))
        return self.c.fetchone()

    def getLecturer(self, lecturerID):
        self.c.execute("""
                    SELECT name, email FROM Lecturer
                    WHERE id = ?
        """, (lecturerID,))
        return self.c.fetchone()

    def getAbsences(self, lecturerID, time='-3 Hour'):
        self.c.execute("""
                    SELECT Missed.studentid, Student.name, Module.name, Module.id, Lecture.id, Lecture.starttime, Lecture.endtime, Lecture.location, Missed.datetime FROM Missed
                    JOIN Lecture ON Missed.lectureid = Lecture.id
                    JOIN Student ON Missed.studentid = Student.id
                    JOIN Module ON Lecture.moduleid = Module.id
                    WHERE Module.lecturerid = ?
                    AND Missed.datetime >= datetime('now', ?)
                    ORDER BY Lecture.id
        """, (lecturerID,time))
        return self.c.fetchall()

if __name__ == "__main__":
    # Code to test the database interface 
    db = DB_Interface()
    db.storeNewStudent(1549228, 0, "Bob")
    db.storeNewStudent(1549223, 0)
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
