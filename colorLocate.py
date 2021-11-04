import cv2
import time
import socket
import argparse
import math
import numpy as np

cap = cv2.VideoCapture(0)

White = [57,0,180,102,65,255]
def findColor(img,White):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower = np.array(White[0],White[1],White[2])
  upper = np.array(White[3],White[4],White[5])
  mask = cv2.inRange(imgHSV,lower,upper)
  

while True:
	success,img = cap.read()
	imgMasked = cv2.bitwise_and(img,img,mask=mask)


	cv2.imshow("Image",img)
	cv2.imshow("Mask",imgMasked)

	if cv2.waitKey(1) >= 0:
		break
