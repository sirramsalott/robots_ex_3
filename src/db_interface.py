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

    def storeNewStudent(self, studentID, eigenface):
        # Store a new student, with their face, in the DB
        
        pass

    def getNewFaces(self, existingStudentIDs):
        return # (studentID, eigenface) pairs from database that are not already in the cache


if __name__ == "__main__":
    db = DB_Interface()
    print(db.getLectureNameAndLocation(1))
    print(db.getStudentCurrentLecture(1549223))