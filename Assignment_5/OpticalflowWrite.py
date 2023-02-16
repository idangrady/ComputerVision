import cv2
import numpy as np


cap = cv2.VideoCapture(0) # This could be changed


while True: 
    _, frame = cap.read()

    cv2.imshow("Frame", frame)

    key = waitKey(1)
    if(key==27):
        break

cap.realease()
cv2.destroyAllWindows()
