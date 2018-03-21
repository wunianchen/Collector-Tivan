import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle

# define video capture object

camL = cv2.VideoCapture(1);
camR = cv2.VideoCapture(2);

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

while (keep_processing):

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

    grayL = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
    grayR = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);
    '''
    track_window = cv2.selectROI(grayL, False)
    c,r,w,h      = track_window
    print(track_window)
    
    # set up the ROI for tracking
    im0 = grayL[r:r+h, c:c+w]
    im1 = grayR[r:r+h, c:c+w]
    '''
    #vis2 = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    # compute disparity image from undistorted and rectified versions
    # (which for reasons best known to the OpenCV developers is returned scaled by 16)
    '''
    disparity = stereoProcessor.compute(undistorted_rectifiedL,undistorted_rectifiedR);
    cv2.filterSpeckles(disparity, 0, 4000, 128);

    # scale the disparity to 8-bit for viewing

    disparity_scaled = (disparity / 16.).astype(np.uint8) + abs(disparity.min())
    '''
    #disparity_scaled = bm.disp_bm(im0, im1)
    disparity_scaled = bm.disp_bm(grayL, grayR)
    #disparity_scaled = sgbm.disp_sgbm(undistorted_rectifiedL, undistorted_rectifiedR)
    # display image
    #count = count + 1
    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c,r,w,h      = track_window

    points = cv2.reprojectImageTo3D(disparity_scaled[r:r+h, c:c+w], Q)
    
    # set up the ROI for tracking
    #im0 = grayL[r:r+h, :]
    #im1 = grayR[r:r+h, :]
    
    coor = np.mean(np.mean(points, axis=0), axis=0)
    print(np.degrees(np.arctan2(coor[1], coor[0])))
    print(np.linalg.norm([coor[0], coor[1]]))


    cv2.imshow(windowNameL,undistorted_rectifiedL);
    cv2.imshow(windowNameR,undistorted_rectifiedR);

    #display disparity

    cv2.imshow(windowNameD, disparity_scaled);

    # start the event loop - essential

    key = cv2.waitKey(30) & 0xFF; # wait 40ms (i.e. 1000ms / 25 fps = 40 ms)

    # It can also be set to detect specific key strokes by recording which key is pressed

    # e.g. if user presses "x" then exit

    if (key == ord('c')):
        keep_processing = False;

#####################################################################

# close all windows and cams.

cv2.destroyAllWindows()

#####################################################################

