TYPE_STUDENT = 1
TYPE_STAFF = 2
TYPE_UNKNOWN = 3

class Face:
    pass

def getAllFaces(img_np):
    # expect: nparray representing the image
    return # list of Face objects found in image

def selectFaceToTrack(faces):
    # expect: list of Face objects
    return # the Face object we'd like to track next

def findFaceInImage(img_np, face):
    # expect: np array of image, Face object to find
    return # bounding box containing that face

def faceIsCentred(boundingBox, img_np):
    # expect: bounding box containing a face, np array of image
    return # whether the camera is sufficiently centred on the face to begin interacting with it

def boundingBoxCentre(boundingBox):
    # expect: a bounding box
    return # centre of the bounding box in image coordinates

def faceTypeAndID(face):
    # expect: Face object
    return # the type (student/staff/unrecognised) and database ID (if known) of the face
