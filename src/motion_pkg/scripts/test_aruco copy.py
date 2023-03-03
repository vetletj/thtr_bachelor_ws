import numpy as np
import cv2
from cv2 import aruco

# Set up the aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

# Create the aruco board
board = aruco.CharucoBoard_create(5, 7, 100, 20, aruco_dict)

# Draw the board
img = board.draw((600, 600), 1)

# Save the image

cv2.imwrite("aruco_board.png", img)
