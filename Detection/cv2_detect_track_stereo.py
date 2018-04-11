from __future__ import print_function
import numpy as np
import cv2

import smbus
import time

import sys
import sgbm
import pickle

import os
import fnmatch

def find_file(pattern,path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result
    
def plot_image_box_label(img,out_dict):
    for key in out_dict:
        for vec in out_dict[key]:
            y      = vec[0] ; x     = vec[1]
            bottom = vec[2] ; right = vec[3]
            score  = vec[4]
            cv2.putText(img, key+' '+str(score)+'%',(int(x), int(y)),cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            cv2.rectangle(img, (x, y), (right, bottom), (125, 255, 51), thickness=2)
    cv2.imshow('Object Detection', img)
    cv2.imwrite('detected.png', img)
    return

def ExtractROI (objectId,out_dict):
    #vec = sorted(out_dict[objectId],key = lambda x: int(x[4])) 
    score_pre = 0
    for vec in out_dict[objectId]:
        score = vec[4]
        if (score < score_pre):
            break
        left   = vec[1] ; top    = vec[0]
        right  = vec[3] ; bottom = vec[2]
        c = left ; w = right - left
        r = top  ; h = bottom - top
        score_pre = score
        break
    track_window = (c,r,w,h)
    return track_window

# Detection path
PATH_TO_SHR = 'Share/'

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
    new_data = data[abs(data-np.mean(data)) < m * np.std(data)]
    if new_data.size == 0:
        new_data = data
    return np.mean(new_data)


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

f5 = open('invmapL1.pckl','rb')
invMapL1 = pickle.load(f5).astype(int)
f5.close()

f6 = open('invmapL2.pckl','rb')
invMapL2 = pickle.load(f6).astype(int)
f6.close()

    
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
    
    while 1:
        input_name = input('What is the function (Manual-ROI,Auto-Detect,Self-Select)?')
        if (input_name == 'Auto-Detect'):
            func_code = 1
            obj_name = input('What is the type of the target?')
            cap_det = False
            break
        elif (input_name == 'Self-Select'):
            func_code = 0
            cap_det = False
            break
        elif (input_name == 'Manual-ROI'):
            func_code = 2
            cap_det = True
            break

    ############ Detection ###################
    try_time = 0
    while (not cap_det):
        # Capture image
        print('Capture image ......')
        if (try_time > 0):
            send2uno('n',try_time)
            time.sleep(3)
        for i in range(5):
            ret, frameL = camL.read()
        cv2.imshow('Object Detection',frameL)
        key = cv2.waitKey(500) & 0xFF
        ret1 = cv2.imwrite(PATH_TO_SHR+'img0.jpg',frameL[120:480,0:480,:])
        ret2 = cv2.imwrite(PATH_TO_SHR+'img1.jpg',frameL[120:480,160:640,:])
        ret0 = cv2.imwrite(PATH_TO_SHR+'img2.jpg',frameL)
        print(ret0 and ret1)
        # Detection
        print('Wait for detection ......')
        while(1):
            pkl_name = find_file('*.pkl',PATH_TO_SHR)
            try:
                if (pkl_name[0] == 'Share/target.pkl'):
                    print(pkl_name)
                    pkl_file = open(pkl_name[0],'rb')
                    out_dict = pickle.load(pkl_file)
                    break
            except:
                continue
            time.sleep(0.5)
            cv2.destroyAllWindows()
        print(out_dict)
        pkl_file.close()
        os.remove(pkl_name[0])
        print('Finish Detection ......')
        if (func_code == 0):
            plot_image_box_label(frameL,out_dict)
            key = cv2.waitKey(0) & 0xFF
        while True:
            if func_code == 0:
                obj_name = input('What is the type of the target (or Retry)?')
            if (obj_name in out_dict and len(out_dict[obj_name])>0):
                print('Detection Success ......')
                cv2.destroyAllWindows()
                cap_det = True
                break
            else:
                cv2.destroyAllWindows()
                try_time = try_time + 1
                print('Failed')
                break

    ######################### Tracking ################################
    # setup initial location of window
    if (func_code != 2):
        track_window = ExtractROI(obj_name,out_dict)
    else:
        track_window = cv2.selectROI(frameL, False)
    c,r,w,h      = [int(track_window[v]) for v in range(4)]
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
    msg_count = 25
    while(1):
        pts_vec = np.zeros([4,2])
        for i in range(track_loop):
            ret, frameL = camL.read()
            if ret == True:
                hsv = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
                dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
                # apply meanshift to get the new location
                ret, track_window = cv2.CamShift(dst, track_window, term_crit)
                pts = cv2.boxPoints(ret)
                pts_vec = pts_vec+pts
                #print('pts',pts)
            else:
                break
        # Draw it on imagep
        pts = pts_vec/track_loop
        pts = np.int0(pts)
        # img2 = cv2.polylines(frame,[pts],True, 255,2)
        # Draw central points
        cpts = (int(pts[:,0].mean()),int(pts[:,1].mean()))
        #print(cpts)
        #exit()
        img2 = cv2.circle(frameL,cpts,10,255,-1)
        if (cpts[0] < cols/2-20):
            cv2.putText(img2, 'left',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                print('cpts[0]',cpts[0])
                print(cols/2)
                send2uno('l',rot_num)
                msg_count = 0
                if (rot_num == 9):
                    rot_num = 4
                else:
                    rot_num = rot_num+1
        elif (cpts[0] > cols/2+20):
            cv2.putText(img2, 'right',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                print('cpts[0]',cpts[0])
                print(cols/2)
                send2uno('r',rot_num)
                msg_count = 0
                if (rot_num == 9):
                    rot_num = 4
                else:
                    rot_num = rot_num+1
        else:
            cv2.putText(img2, 'center',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
            if (msg_count == 30 and send_msg_ok==1):
                #cv2.imwrite('tracking.png', img2)
                send2uno('c',0)
                send_msg_ok = 0
                break
        msg_count = msg_count + 1
        cv2.imshow(windowNameL, img2)
        k = cv2.waitKey(10) & 0xff
        if k == 27:
            break

    ###########################Stereo ###############################################
    
    for i in range(50):
        camL.grab();
        camR.grab();

        ret, frameL = camL.retrieve();
        ret, frameR = camR.retrieve();
    
    print("-> calc. disparity image")
    h1 = int(h/4)
    w1 = int(w/4)
    
    cleft = int(cpts[0] - w1)
    cright = int(cpts[0] + w1)
    rtop = int(cpts[1] - h1)
    rbottom = int(cpts[1] + h1)

    left_top = [invMapL2[rtop, cleft], invMapL1[rtop, cleft]]
    right_bottom = [invMapL2[rbottom, cright], invMapL1[rbottom, cright]]
    
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
        disparity_scaled = cv2.rectangle(disparity_scaled, (left_top[1], left_top[0]),
                                      (right_bottom[1], right_bottom[0]), (0, 0, 100), 1)
        
        points1 = cv2.reprojectImageTo3D(disparity_scaled, Q)
        points = points1[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1], :]
        
        num_points = points.shape[0]*points.shape[1]
        points = points.reshape(num_points, points.shape[2])
        points2 = []
        #dlist = []
        for n in range(num_points):
            if np.all(np.isfinite(points[n,:])):
                points2.append(points[n,:])
                #dlist.append(points[n,2])
        points = np.array(points2)
        #dlist = np.array(dlist)
        
        coor = np.mean(points, axis=0)
        angle = np.degrees(np.arctan2(coor[2], coor[0]))
        distance = np.linalg.norm([coor[0],coor[2]])
        print('======',i)
        print(coor)
        print('angle: ', angle)
        print('distance', distance)
        rst[0, i] = distance
        angle_rst[0, i] = angle
        
        cv2.imshow(windowNameL, undistorted_rectifiedL);
        cv2.imshow(windowNameR, undistorted_rectifiedR);
        cv2.imshow(windowNameD, disparity_scaled);
        key = cv2.waitKey(40) & 0xFF;

    final = int(to_cm(reject_outliers(rst, 1.5)))
    print('final distance: ', final)
    final_angle = int(abs(90 - np.mean(angle_rst)))
    #final_angle = 5;
    print('final angle: ', final_angle)
    send2uno('a', final_angle%10)
    send2uno('d', int(final/100))
    send2uno('d', int((final%100)/10))
    send2uno('d', final%10)
    key = cv2.waitKey() & 0xFF
    if (key == 13):
        cv2.destroyAllWindows()

    ############### GO-BACK ###################
    go_back = False
    while True:
        inst = input('Go back (Yes or No)?')
        if (inst == 'Yes'):
            go_back = True
            break
        elif (inst == 'No'):
            break
    
    if go_back == True:
        obj_name = 'Sign'
        try_time = 0
        cap_det  = False
        while (not cap_det):
            # Capture image
            print('Capture image ......')
            if (try_time > 0):
                send2uno('m',try_time)
                time.sleep(3)
            for i in range(5):
                ret, frameL = camL.read()
            cv2.imshow('Object Detection',frameL)
            key = cv2.waitKey(500) & 0xFF
            ret1 = cv2.imwrite(PATH_TO_SHR+'img0.jpg',frameL[120:480,0:480,:])
            ret2 = cv2.imwrite(PATH_TO_SHR+'img1.jpg',frameL[120:480,160:640,:])
            ret0 = cv2.imwrite(PATH_TO_SHR+'img2.jpg',frameL)
            print(ret0 and ret1 and ret2)
            cv2.destroyAllWindows()
            # Detection
            print('Wait for detection ......')
            while(1):
                pkl_name = find_file('*.pkl',PATH_TO_SHR)
                try:
                    if (pkl_name[0] == 'Share/target.pkl'):
                        print(pkl_name)
                        pkl_file = open(pkl_name[0],'rb')
                        ini_dict = pickle.load(pkl_file)
                        break
                except:
                    continue
                time.sleep(0.5)
            
            out_dict = ini_dict
            print(out_dict)
            pkl_file.close()
            os.remove(pkl_name[0])
            print('Finish Detection ......')
            plot_image_box_label(frameL,out_dict)
            key = cv2.waitKey(0) & 0xFF
            while True:
                if (obj_name in out_dict and len(out_dict[obj_name])>0):
                    print('Detection Success ......')
                    cv2.destroyAllWindows()
                    cap_det = True
                    break
                else:
                    cv2.destroyAllWindows()
                    try_time = try_time + 1
                    print('Failed')
                    break

        #Tracking
        # setup initial location of window
        track_window = ExtractROI(obj_name,out_dict)
        c,r,w,h      = [int(track_window[v]) for v in range(4)]
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
        msg_count = 25
        while(1):
            pts_vec = np.zeros([4,2])
            for i in range(track_loop):
                ret, frameL = camL.read()
                if ret == True:
                    hsv = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
                    dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
                    # apply meanshift to get the new location
                    ret, track_window = cv2.CamShift(dst, track_window, term_crit)
                    pts = cv2.boxPoints(ret)
                    pts_vec = pts_vec+pts
                    #print('pts',pts)
                else:
                    break
            # Draw it on imagep
            pts = pts_vec/track_loop
            pts = np.int0(pts)
            # img2 = cv2.polylines(frame,[pts],True, 255,2)
            # Draw central points
            cpts = (int(pts[:,0].mean()),int(pts[:,1].mean()))
            #print(cpts)
            #exit()
            img2 = cv2.circle(frameL,cpts,10,255,-1)
            if (cpts[0] < cols/2-20):
                cv2.putText(img2, 'left',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
                if (msg_count == 30 and send_msg_ok==1):
                    print('cpts[0]',cpts[0])
                    print(cols/2)
                    send2uno('l',rot_num)
                    msg_count = 0
                    if (rot_num == 9):
                        rot_num = 4
                    else:
                        rot_num = rot_num+1
            elif (cpts[0] > cols/2+20):
                cv2.putText(img2, 'right',cpts,cv2.FONT_HERSHEY_SIMPLEX,1,(125, 255, 51), 2, cv2.LINE_AA)
                if (msg_count == 30 and send_msg_ok==1):
                    print('cpts[0]',cpts[0])
                    print(cols/2)
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
                    send2uno('b',0)
                    send_msg_ok = 0
                    break
            msg_count = msg_count + 1
            cv2.imshow(windowNameL, img2)
            k = cv2.waitKey(10) & 0xff
            if k == 27:
                break
    
cv2.destroyAllWindows()
capL.release()
camR.release()
