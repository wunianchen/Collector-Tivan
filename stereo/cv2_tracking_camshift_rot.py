import numpy as np
import cv2

import smbus
import time

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

    
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
for i in range(100):
    ret, frame = cap.read()
rows = frame.shape[0]
cols = frame.shape[1]

# setup initial location of window
track_window = cv2.selectROI(frame, False)
c,r,w,h      = [int(track_window[v]) for v in range(4)]
track_window = tuple([c,r,w,h])
track_loop   = 1
print(track_window)

# set up the ROI for tracking
roi = frame[r:r+h, c:c+w]
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
        ret,frame = cap.read()
        if ret == True:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
    img2 = cv2.circle(frame,cpts,10,255,-1)
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
    msg_count = msg_count + 1
    cv2.imshow('img2',img2)
    k = cv2.waitKey(10) & 0xff
    if k == 27:
        break
    else:
        cv2.imwrite(chr(k)+".jpg",img2)
    
cv2.destroyAllWindows()
cap.release()
