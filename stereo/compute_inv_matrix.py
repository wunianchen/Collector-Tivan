from __future__ import print_function
import numpy as np
import cv2
import time
import sys
import pickle

f0 = open('mapL1.pckl','rb')
mapL1 = pickle.load(f0)
f0.close()

f1 = open('mapL2.pckl','rb')
mapL2 = pickle.load(f1)
f1.close()

print(mapL1)
print(mapL2)

invMapL1 = np.zeros(mapL1.shape)
invMapL2 = np.zeros(mapL1.shape)

for i in range(mapL1.shape[0]):
	for j in range(mapL1.shape[1]):
		invMapL1[int(mapL2[i, j]), int(mapL1[i, j])] = j
		invMapL2[int(mapL2[i, j]), int(mapL1[i, j])] = i

f0 = open('invmapL1.pckl','wb')
pickle.dump(invMapL1,f0)
f0.close()

f1 = open('invmapL2.pckl','wb')
pickle.dump(invMapL2,f1)
f1.close()
