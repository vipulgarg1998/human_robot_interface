import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from cv2 import aruco
def detectMarkers(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
	parameters = aruco.DetectorParameters_create()
	print("parameters", parameters)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	print("corners",corners)
	print("ids", ids)
	print("rejectedImgPoints",rejectedImgPoints)
	frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
	return frame_markers
img = cv2.imread('img1.png')

# while True:
imageWithMarkers = detectMarkers(img)
cv2.imshow('FOLLOWER',imageWithMarkers)
k = cv2.waitKey(2000)
# cv2.imshow('image',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()