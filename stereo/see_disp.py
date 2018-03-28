import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle

# define video capture object

camR = cv2.VideoCapture(0);
camL = cv2.VideoCapture(2);

# define display window names

windowNameL = "LEFT Camera Input"; # window name
windowNameR = "RIGHT Camera Input"; # window name

print("-> calc. disparity image")

max_disparity = 128;
stereoProcessor = cv2.StereoSGBM_create(0, max_disparity, 21);

keep_processing = True;

windowNameD = "SGBM Stereo Disparity - Output"; # window name
#count = 0
f0 = open('mapL1.pckl','rb')
mapL1 = pickle.load(f0)
f0.close()

f1 = open('mapL2.pckl','rb')
mapL2 = pickle.load(f1)
f1.close()

f2 = open('mapR1.pckl','rb')
mapR1 = pickle.load(f2)
f2.close()

f3 = open('mapR2.pckl','rb')
mapR2 = pickle.load(f3)
f3.close()

f4 = open('q.pckl','rb')
Q = pickle.load(f4)
f4.close()

for i in range(20):
    camL.grab();
    camR.grab();

    # then retrieve the images in slow(er) time
    # (do not be tempted to use read() !)

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();
    
while (keep_processing):
    # ===========================0
    # grab frames from camera (to ensure best time sync.)
    camL.grab();
    camR.grab();

    # then retrieve the images in slow(er) time
    # (do not be tempted to use read() !)

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();
        
    # undistort and rectify based on the mappings (could improve interpolation and image border settings here)
    # N.B. mapping works independant of number of image channels

    undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
    undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);
    
    # remember to convert to grayscale (as the disparity matching works on grayscale)
    grayL0 = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
    grayR0 = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);

    disparity_scaled = bm.disp_bm(grayL0, grayR0)


    cv2.imshow(windowNameL,undistorted_rectifiedL);
    cv2.imshow(windowNameR,undistorted_rectifiedR);

    cv2.imshow(windowNameD, disparity_scaled);

    # start the event loop - essential

    key = cv2.waitKey(30) & 0xFF; # wait 40ms (i.e. 1000ms / 25 fps = 40 ms)
    if (key == ord('c')):
        keep_processing = False;

#####################################################################

# close all windows and cams.

cv2.destroyAllWindows()

#####################################################################

