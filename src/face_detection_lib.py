import face_recognition as fr
import cv2
import numpy as np
import db_interface as db

image_width = 640
image_height = 480

face_lock_threshold = 0.4

class Face:
    def __init__(self, eigenface, studentID=None):
        self.studentID = studentID
        self.eigenface = eigenface


class FaceDetectionModel:
    def __init__(self):
        self.dbHandle = db.DB_Interface()

        self.faceDBCache = []
        self.updateFaceDBCache()

    def getBoundingBoxes(self, img):
        IMG_SCALE = 4.

        imgSmall = cv2.resize(img, (0, 0),
                              fx=1 / IMG_SCALE,
                              fy=1 / IMG_SCALE)

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
        print("FACE CACHE {}".format(self.faceDBCache))
        for studentID, eigenface in self.faceDBCache:
            print("EIG {}".format(np.array(eigenface)))
            if any(fr.compare_faces([faceToMatch], np.array(eigenface))):
                return studentID

        return None

    def faceIsCentred(self, boundingBox, img):
        height = boundingBox[2] - boundingBox[0]
        return (float(height) / image_height) >= face_lock_threshold

    def updateFaceDBCache(self):
        self.faceDBCache += [(i, f) for (i, f) in self.dbHandle.getNewFaces(existingStudentIDs=self.faceDBCache) if f is not None]

