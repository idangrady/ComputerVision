import cv2 as cv
import numpy as np


feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

cap = cv.VideoCapture(r'D:/github_/ComputerVision/Assignment_5/__temp__.mp4') # here we should add the video
_, oldFrame = cap.read()
old_gray = cv.cvtColor(oldFrame, cv.COLOR_BGR2GRAY)

# get the values to track and find their corners 
oldPoints = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
mask = np.zeros_like(old_gray)

color = np.random.randint(0, 255, (100, 3))

while(cap.isOpened()):
    ret, frame_new = cap.read()
    if ret == True:
        greyFrame_new = cv.cvtColor(frame_new, cv.COLOR_BGR2GRAY)
        newPoints, status, error = cv.calcOpticalFlowPyrLK(old_gray, greyFrame_new, oldPoints, None, **lk_params) # where p0 are the points to track

        if newPoints is not None:
            good_new = newPoints[status==1]
            good_old = oldPoints[status==1]
        
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                frame = cv.circle(old_gray, (int(a), int(b)), 5, color[i].tolist(), -1)


        blank = np.ones((frame.shape[0], frame.shape[1]), dtype= mask.dtype) # This is  a black frame. 
        img = cv.add(blank, mask)

        old_gray = greyFrame_new.copy()
        oldPoints = good_new.reshape(-1, 1, 2)

    else:
        break

cv.imshow('Frame',img)
cv.waitKey(0)

    