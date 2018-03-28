import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle


def to_cm(x):
    return (0.1884*x - 40.327)


def reject_outliers(data, m):
    data = data.reshape((1, data.shape[0]*data.shape[1]))
    return data[abs(data-np.mean(data)) < m * np.std(data)]


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
        
        undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
        undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);
        
        cv2.imshow(windowNameL,undistorted_rectifiedL);
        cv2.imshow(windowNameR,undistorted_rectifiedR);
        
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
    
    camL.grab();
    camR.grab();

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();

    undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
    undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);

    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c1,r1,w1,h1      = track_window
    
    number = 10
    rst = np.zeros((1,number))
    rst1 = np.zeros((1,number))
    for i in range(number):
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
        
        grayL = cv2.normalize(src=grayL, dst=grayL, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        grayL = np.uint8(grayL)
        grayR = cv2.normalize(src=grayR, dst=grayR, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        grayR = np.uint8(grayR)
        
        disparity_scaled = sgbm.disp_sgbm(grayL, grayR)
        
        points1 = cv2.reprojectImageTo3D(disparity_scaled, Q)
        points = points1[r:r+h, c:c+w, :]
        coor = np.mean(np.mean(points, axis=0), axis=0)
        angle = np.degrees(np.arctan2(coor[2], coor[0]))
        distance = np.linalg.norm([coor[0], coor[2]])
        print('======',i)
        print(coor)
        print('angle: ', angle)
        print('distance', distance)
        rst[0, i] = distance
        
        ###
        points = points1[r1:r1+h1, c1:c1+w1, :]
        coor = np.mean(np.mean(points, axis=0), axis=0)
        angle = np.degrees(np.arctan2(coor[2], coor[0]))
        distance = np.linalg.norm([coor[0], coor[2]])
        print('======',i)
        print('angle: ', angle)
        print('distance', distance)
        rst1[0, i] = distance
        ###
        cv2.imshow(windowNameD, disparity_scaled);
        key = cv2.waitKey(40) & 0xFF;
    
    final = to_cm(np.mean(reject_outliers(rst, 1.5)))
    print('final distance: ', final)
    final = to_cm(np.mean(reject_outliers(rst1, 1.5)))
    print('final distance: ', final)
    
    if (key == ord('x')):
        camL.release()
        camR.release()
        print('release')
        keep_processing = False;

cv2.destroyAllWindows()
