import os
import pytesseract
# import cv2
import re
from PIL import Image, ImageEnhance, ImageFilter
from collections import Counter

idre = re.compile(r'\d\d\d\d\d\d\d')
# cam = cv2.VideoCapture(1)
keepreading = True
studentid_readings = Counter()
path = os.path.dirname(__file__)

img = Image.open(path+"/v24.png")
img = img.convert('L')
sharpen = ImageEnhance.Sharpness(img)
img = sharpen.enhance(0.5)
text = pytesseract.image_to_string(img)
print(text)
img.save(path+"/v25.png")



# while keepreading:
#     ret_val, img = cam.read()
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)[1]
#     img = cv2.medianBlur(img, 3)
#     cv2.imshow('FaceTime HD', img)
#     if cv2.waitKey(1) == 27: 
#         break  # esc to quit
#     ocrtext = pytesseract.image_to_string(img)
#     m = idre.search(ocrtext)
#     if m:
#         group = m.group()
#         print("Found ID: " + group)
#         studentid_readings[group] = studentid_readings[group] + 1
#         if sum(studentid_readings.values()) >= 5:
#             max_studentid = max(studentid_readings, key=studentid_readings.get)
#             print("Final ID guess: " + max_studentid)
#             keepreading = False
# cv2.destroyAllWindows()