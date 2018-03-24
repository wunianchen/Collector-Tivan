import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle

# define video capture object

camL = cv2.VideoCapture(0);
camR = cv2.VideoCapture(1);

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
    while 1:
        camL.grab();
        camR.grab();

        # then retrieve the images in slow(er) time
        # (do not be tempted to use read() !)

        ret, frameL = camL.retrieve();
        ret, frameR = camR.retrieve();
        
        cv2.imshow(windowNameL,frameL);
        cv2.imshow(windowNameR,frameR);
        key = cv2.waitKey(100) & 0xFF
        if (key == ord('c')):
            break
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
    
    # ===========================1
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
    grayL1 = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
    grayR1 = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);
    
    # ===========================2
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
    grayL2 = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
    grayR2 = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);

    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c,r,w,h      = track_window
    
    #disparity_scaled = bm.disp_bm(im0, im1)
    #disparity_scaled = (disparity / 16.).astype(np.uint8) + abs(disparity.min())
    #disparity_scaled = sgbm.disp_sgbm(undistorted_rectifiedL, undistorted_rectifiedR)
    # display image
    #count = count + 1

    disparity_scaled = bm.disp_bm(grayL0, grayR0)
    points = cv2.reprojectImageTo3D(disparity_scaled, Q)
    points = points[r:r+h, c:c+w, :]
    coor = np.mean(np.mean(points, axis=0), axis=0)
    angle0 = np.degrees(np.arctan2(coor[2], coor[0]))
    distance0 = np.linalg.norm([coor[0], coor[2]])
    print('=======================')
    print(angle0)
    print(distance0)
    
    cv2.imshow(windowNameD, disparity_scaled);
    key = cv2.waitKey(0)
    
    disparity_scaled = bm.disp_bm(grayL1, grayR1)
    points = cv2.reprojectImageTo3D(disparity_scaled, Q)
    points = points[r:r+h, c:c+w, :]
    coor = np.mean(np.mean(points, axis=0), axis=0)
    angle1 = np.degrees(np.arctan2(coor[2], coor[0]))
    distance1 = np.linalg.norm([coor[0], coor[2]])
    print('=======================')
    print(angle1)
    print(distance1)
    
    cv2.imshow(windowNameD, disparity_scaled);
    key = cv2.waitKey(0)

    disparity_scaled = bm.disp_bm(grayL2, grayR2)
    points = cv2.reprojectImageTo3D(disparity_scaled, Q)
    points = points[r:r+h, c:c+w, :]
    coor = np.mean(np.mean(points, axis=0), axis=0)
    angle2 = np.degrees(np.arctan2(coor[2], coor[0]))
    distance2 = np.linalg.norm([coor[0], coor[2]])
    print('=======================')
    print(angle2)
    print(distance2)
    
    cv2.imshow(windowNameD, disparity_scaled);
    key = cv2.waitKey(0)


    #cv2.imshow(windowNameL,frameL);
    #cv2.imshow(windowNameR,frameR);

    #display disparity

    #cv2.imshow(windowNameD, disparity_scaled);

    # start the event loop - essential

    #key = cv2.waitKey(0)# wait 40ms (i.e. 1000ms / 25 fps = 40 ms)

    # It can also be set to detect specific key strokes by recording which key is pressed

    # e.g. if user presses "x" then exit
    
    
    if (key == ord('x')):
        camL.release()
        camR.release()
        print('release')
        keep_processing = False;

#####################################################################

# close all windows and cams.

cv2.destroyAllWindows()

#####################################################################

