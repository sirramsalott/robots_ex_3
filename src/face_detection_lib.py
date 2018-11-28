import face_recognition as fr
import cv2

import db_interface as db


class Face:
    def __init__(self, eigenface, studentID=None):
        self.studentID = studentID
        self.eigenface = eigenface


class FaceDetectionModel:
    def __init__(self):
        self.dbHandle = db.DB_Interface()

        self.faceDBCache = []
        self.faceDBCache = self.updateFaceDBCache()

    def getBoundingBoxes(self, img):
        IMG_SCALE = 4.

        imgSmall = cv2.resize(img, (0, 0),
                              fx=1/IMG_SCALE,
                              fy=1/IMG_SCALE)

        boundingBoxes = fr.face_locations(imgSmall,
                                          number_of_times_to_upsample=2)

        return list((int(top * IMG_SCALE),
                     int(right * IMG_SCALE),
                     int(bottom * IMG_SCALE),
                     int(left * IMG_SCALE))
                    for (top, right, bottom, left) in boundingBoxes)

    def selectFaceToTrack(self, boundingBoxes, img):
        # TODO: we just go to the closest face, but is this best?
        biggestBoundingBox = max(boundingBoxes,
                                 key=lambda box: box[1] - box[3])
        return fr.face_encodings(face_image=img,
                                 known_face_locations=[biggestBoundingBox])[0]

    def facesMatch(self, boundingBox, img, faceToMatch):
        eigenface = fr.face_encodings(face_image=img,
                                      known_face_locations=[boundingBox])[0]

        return any(fr.compare_faces([faceToMatch], eigenface)), eigenface

    def getFaceID(self, faceToMatch):
        for studentID, eigenface in self.faceDBCache:
            if any(fr.compare_faces([faceToMatch], eigenface):
                   return studentID

        return None

    def faceIsCentred(self, boundingBox, img):
        # expect: bounding box containing a face, image
        return # whether the camera is sufficiently centred on the face to begin interacting with it

    def updateFaceDBCache(self):
        self.faceDBCache += self.dbHandle.getNewFaces(existingStudentIDs=self.faceDBCache)
