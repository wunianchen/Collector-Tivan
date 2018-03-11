from __future__ import print_function

import numpy as np
import cv2 as cv
import re
import sys
from matplotlib import pyplot as plt

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

def load_properties(sep='=', comment_char='#'):
    props = {}
    with open('calib.txt', 'rt') as f:
        for line in f:
            l = line.strip()
            if l and not l.startswith(comment_char):
                key_value = l.split(sep)
                key = key_value[0].strip()
                value = sep.join(key_value[1:]).strip().strip('"') 
                props[key] = value 
    return props


if __name__ == '__main__':
    print('loading images...')
    #imgL = cv.pyrDown( cv.imread('im0.png') )  # downscale images for faster processing
    #imgR = cv.pyrDown( cv.imread('im1.png') )
    imgL = cv.imread('im0.png')
    imgR = cv.imread('im1.png')

    # disparity range is tuned for 'aloe' image pair
    calib = load_properties()
    min_disp = int(calib['vmin'])
    num_disp = int(calib['vmax'])-min_disp
    window_size = 3
    stereo = cv.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 7,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    plt.imshow(disp,'gray')
    plt.show()

    print('generating 3d point cloud...',)
    h, w = imgL.shape[:2]
    f = 0.8*w                          # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    points = cv.reprojectImageTo3D(disp, Q)
    colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]
    out_fn = 'out.ply'
    write_ply('out.ply', out_points, out_colors)
    print('%s saved' % 'out.ply')

    cv.imshow('left', imgL)
    cv.imshow('disparity', (disp-min_disp)/num_disp)
    cv.waitKey()
    cv.destroyAllWindows()
