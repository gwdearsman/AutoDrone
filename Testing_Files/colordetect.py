import cv2
import time
import socket
import argparse
import math
import numpy as np

cap = cv2.VideoCapture(0)

def empty(a):
	pass

cv2.namedWindow("HSV")
cv2.createTrackbar("HUE MIN", "HSV", 0, 179, empty)
cv2.createTrackbar("SAT MIN", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE MIN", "HSV", 0, 255, empty)
cv2.createTrackbar("HUE MAX", "HSV", 0, 179, empty)
cv2.createTrackbar("SAT MAX", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE MAX", "HSV", 0, 255, empty)

White = [0,0,0,179,255,255]
def findColor(img,White):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower = np.array(White[0:3])

while True:
	success,img = cap.read()
	imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	h_min = cv2.getTrackbarPos("HUE MIN", "HSV")
	s_min = cv2.getTrackbarPos("SAT MIN", "HSV")
	v_min = cv2.getTrackbarPos("VALUE MIN", "HSV")
	h_max = cv2.getTrackbarPos("HUE MAX", "HSV")
	s_max = cv2.getTrackbarPos("SAT MAX", "HSV")
	v_max = cv2.getTrackbarPos("VALUE MAX", "HSV")
	lower = np.array([h_min,s_min,v_min])
	upper = np.array([h_max,s_max,v_max])
	mask = cv2.inRange(imgHSV,lower,upper)
	imgMasked = cv2.bitwise_and(img,img,mask=mask)


	cv2.imshow("Image",img)
	cv2.imshow("Mask",imgMasked)

	if cv2.waitKey(1) >= 0:
		break
