import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle

def to_cm(x):
    return (0.1729*x - 18.271)

# define video capture object

camL = cv2.VideoCapture(0);
camR = cv2.VideoCapture(1);

# define display window names

windowNameL = "LEFT Camera Input"; # window name
windowNameR = "RIGHT Camera Input"; # window name

print("-> calc. disparity image")

keep_processing = True;

windowNameD = "SGBM Stereo Disparity - Output"; # window name

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

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();
    
while (keep_processing):
    while 1:
        camL.grab();
        camR.grab();

        ret, frameL = camL.retrieve();
        ret, frameR = camR.retrieve();
        
        cv2.imshow(windowNameL,frameL);
        cv2.imshow(windowNameR,frameR);
        
        key = cv2.waitKey(100) & 0xFF
        if (key == ord('c')):
            break
        elif (key == ord('x')):
            camL.release()
            camR.release()
            print('release')
            keep_processing = False;
            break
    
    camL.grab();
    camR.grab();

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();

    undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
    undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);

    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c,r,w,h      = track_window
    
    rst = 0
    for i in range(10):
        camL.grab();
        camR.grab();

        ret, frameL = camL.retrieve();
        ret, frameR = camR.retrieve();
            
        # undistort and rectify based on the mappings (could improve interpolation and image border settings here)
        # N.B. mapping works independant of number of image channels

        undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
        undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);
        
        # remember to convert to grayscale (as the disparity matching works on grayscale)
        grayL = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
        grayR = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);
        
        disparity_scaled = bm.disp_bm(grayL, grayR)
        points = cv2.reprojectImageTo3D(disparity_scaled, Q)
        points = points[r:r+h, c:c+w, :]
        coor = np.mean(np.mean(points, axis=0), axis=0)
        angle = np.degrees(np.arctan2(coor[2], coor[0]))
        distance = np.linalg.norm([coor[0], coor[2]])
        print('angle: ', angle)
        print('distance', distance)
        rst = rst + distance
    
    final = to_cm(rst/10)
    print('final distance: ', final)
    
    if (key == ord('x')):
        camL.release()
        camR.release()
        print('release')
        keep_processing = False;

cv2.destroyAllWindows()
