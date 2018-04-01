import cv2
import sys
import numpy as np
import bm
import sgbm
import pickle

import smbus
import time

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)
# This is the address we setup in the Arduino Program
address = 0x04


def send2uno(angle):
    data_list = list(str(data))
    print(data_list)
    for i in data_list:
    	#Sends to the Slaves 
        bus.write_byte(address,int(ord(i)))
        time.sleep(.1)
    return 1


def to_cm(x):
    return (0.1313*x - 3.8408)


def reject_outliers(data, m):
    data = data.reshape((1, data.shape[0]*data.shape[1]))
    return data[abs(data-np.mean(data)) < m * np.std(data)]


def normalize(im):
    #image_mean = np.mean(X, axis=(0,1,2))
    #image_std = np.std(X, axis=(0,1,2))
    #im = (X - image_mean) / image_std
    im = cv2.normalize(src=im, dst=im, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    im = np.uint8(im)
    return im


# define video capture object

camR = cv2.VideoCapture(0);
camL = cv2.VideoCapture(1);

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
        if (key == ord('a')):
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
    
    cv2.imwrite('L.png', undistorted_rectifiedL)
    cv2.imwrite('R.png', undistorted_rectifiedR)
    
    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c,r,w,h      = track_window
    
    number = 10
    rst = np.zeros((1,number))
    angle_rst = np.zeros((1,number))
    for i in range(number):
        camL.grab();
        camR.grab();

        ret, frameL = camL.retrieve();
        ret, frameR = camR.retrieve();
            
        # undistort and rectify based on the mappings (could improve interpolation and image border settings here)
        # N.B. mapping works independant of number of image channels

        undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
        undistorted_rectifiedR = cv2.remap(frameR, mapR1, mapR2, cv2.INTER_LINEAR);

        undistorted_rectifiedL = normalize(undistorted_rectifiedL)
        undistorted_rectifiedR = normalize(undistorted_rectifiedR)
        
        
        # remember to convert to grayscale (as the disparity matching works on grayscale)
        grayL = cv2.cvtColor(undistorted_rectifiedL,cv2.COLOR_BGR2GRAY);
        grayR = cv2.cvtColor(undistorted_rectifiedR,cv2.COLOR_BGR2GRAY);

        disparity_scaled = sgbm.disp_sgbm(grayL, grayR)
        cv2.imwrite(str(i)+'.png', disparity_scaled)
        
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
        angle_rst[0, i] = angle
        
        cv2.imshow(windowNameD, disparity_scaled);
        key = cv2.waitKey(40) & 0xFF;
    
    final = to_cm(np.mean(reject_outliers(rst, 1.5)))
    print('final distance: ', final)
    final_angle = int(abs(90 - np.mean(angle_rst)))
    print('final angle: ', final_angle)
    #send2uno(final_angle)
    
    
    if (key == ord('x')):
        camL.release()
        camR.release()
        print('release')
        keep_processing = False;

cv2.destroyAllWindows()
