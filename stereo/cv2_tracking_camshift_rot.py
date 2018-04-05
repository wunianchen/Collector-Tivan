from __future__ import print_function
import numpy as np
import cv2

import smbus
import time

import sys
import sgbm
import pickle

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)
# This is the address we setup in the Arduino Program
address = 0x04

def send2uno(direction,num):
    data = direction+str(num)
    data_list = list(data)
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

camL = cv2.VideoCapture(0);
camR = cv2.VideoCapture(1);

# define display window names

windowNameL = "LEFT Camera Input"; # window name
windowNameR = "RIGHT Camera Input"; # window name
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

rows = frameL.shape[0]
cols = frameL.shape[1]

while 1:
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
        elif (key == ord('s')):
            # swap the cameras if specified
            tmp = camL;
            camL = camR;
            camR = tmp;

    camL.grab();
    camR.grab();

    ret, frameL = camL.retrieve();
    ret, frameR = camR.retrieve();
    undistorted_rectifiedL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);

    # setup initial location of window
    track_window = cv2.selectROI(undistorted_rectifiedL, False)
    c,r,w,h      = track_window
    track_window = tuple([c,r,w,h])
    track_loop   = 1
    print(track_window)

    # set up the ROI for tracking
    roi = frameL[r:r+h, c:c+w]
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
    roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)
    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
    rot_num   = 1
    send_msg_ok = 1
    msg_count = 30
    while(1):
        pts_vec = np.zeros([4,2])
        for i in range(track_loop):
            camL.grab();
            camR.grab();
            ret, frameL = camL.retrieve();
            ret, frameR = camR.retrieve();
            frameL = cv2.remap(frameL, mapL1, mapL2, cv2.INTER_LINEAR);
            if ret == True:
                hsv = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
                dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
                # apply meanshift to get the new location
                ret, track_window = cv2.CamShift(dst, track_window, term_crit)
                pts = cv2.boxPoints(ret)
                pts_vec = pts_vec+pts
            else:
                break
        # Draw it on imagep
        pts = pts_vec/track_loop
        pts = np.int0(pts)
        # img2 = cv2.polylines(frame,[pts],True, 255,2)
        # Draw central points
        cpts = (int(pts[:,0].mean()),int(pts[:,1].mean()))
        img2 = cv2.circle(frameL,cpts,10,255,-1)
        if (cpts[0] < cols/2-20):
            cv2.putText(img2, 'left',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                send2uno('l',rot_num)
                msg_count = 0
                if (rot_num == 9):
                    rot_num = 4
                else:
                    rot_num = rot_num+1
        elif (cpts[0] > cols/2+20):
            cv2.putText(img2, 'right',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                send2uno('r',rot_num)
                msg_count = 0
                if (rot_num == 9):
                    rot_num = 4
                else:
                    rot_num = rot_num+1
        else:
            cv2.putText(img2, 'center',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                send2uno('c',0)
                send_msg_ok = 0
                break
        msg_count = msg_count + 1
        cv2.imshow(windowNameL, img2)
        k = cv2.waitKey(10) & 0xff
        if k == 27:
            break
        else:
            cv2.imwrite(chr(k)+".jpg",img2)

    print("-> calc. disparity image")
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
        
        h1 = int(h/2)
        w1 = int(w/2)
        left_top = [(cpts[1]-h1), (cpts[0]-w1)]
        right_bottom = [(cpts[1]+h1), (cpts[0]+w1)]
        disparity_scaled = cv2.rectangle(disparity_scaled, (left_top[1], left_top[0]),
                                      (right_bottom[1], right_bottom[0]), (0, 0, 100), 1)
        
        points1 = cv2.reprojectImageTo3D(disparity_scaled, Q)
        points = points1[(cpts[1]-h1):(cpts[1]+h1), (cpts[0]-w1):(cpts[0]+w1), :]
        
        num_points = points.shape[0]*points.shape[1]
        points = points.reshape(num_points, points.shape[2])
        points2 = []
        for n in range(num_points):
            if np.all(np.isfinite(points[n,:])):
                points2.append(points[n,:])
        points = np.array(points2)
        
        coor = np.mean(points, axis=0)
        angle = np.degrees(np.arctan2(coor[2], coor[0]))
        distance = np.linalg.norm([coor[0], coor[2]])
        print('======',i)
        print(coor)
        print('angle: ', angle)
        print('distance', distance)
        rst[0, i] = distance
        angle_rst[0, i] = angle
        
        cv2.imshow(windowNameL, undistorted_rectifiedL);
        cv2.imshow(windowNameR, undistorted_rectifiedR);
        cv2.imshow(windowNameD, disparity_scaled[(cpts[1]-h1):(cpts[1]+h1), (cpts[0]-w1):(cpts[0]+w1)]);
        key = cv2.waitKey(40) & 0xFF;

    final = int(to_cm(np.mean(reject_outliers(rst, 1.5))))
    print('final distance: ', final)
    final_angle = int(abs(90 - np.mean(angle_rst)))
    #final_angle = 5;
    print('final angle: ', final_angle)
    send2uno('a', final_angle%10)
    send2uno('d', int(final/100))
    send2uno('d', int((final%100)/10))
    send2uno('d', final%10)
    
cv2.destroyAllWindows()
capL.release()
camR.release()
