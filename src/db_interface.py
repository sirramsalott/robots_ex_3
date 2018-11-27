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

if __name__ == "__main__":
    # Code to test the database interface 
    db = DB_Interface()
    print(db.getLectureNameAndLocation(1))
    print(db.getStudentCurrentLecture(1549223))
    db.storeNewStudent(1549228, 0, "Bob")
    db.storeNewStudent(1549223, 0)
    print(db.getStudentName(1549223))
    print(db.getStudentName(1549228))
    print(db.getNewFaces([]))
