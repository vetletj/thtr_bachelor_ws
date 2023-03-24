import numpy
import cv2
import cv2.aruco as aruco
from matplotlib import pyplot as plt

im = cv2.imread('2-03.png')
# im = cv2.imread('1-07.jpg')

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)

plt.figure(figsize = (20,20))
imgplot = plt.imshow(im, interpolation='nearest')
plt.show() 

im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
corners, ids, rejectedImgPoints = aruco.detectMarkers(im, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

if ids is not None:
    print('detected: {}'.format(len(ids)))
    # for i, corner in zip(ids, corners):
            # print('ID: {}; Corners: {}'.format(i, corner))

    im = aruco.drawDetectedMarkers(im, corners, borderColor=(255, 0, 0))
else:
    print("NONE")
plt.figure(figsize = (20,20))
imgplot = plt.imshow(im, interpolation='nearest')
plt.show() 