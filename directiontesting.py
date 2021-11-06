import cv2
import time
import socket
import argparse
import math
import numpy as np


#####################################################################################################
########################################Start of OpenCV################################################
#####################################################################################################

cap = cv2.VideoCapture(0)
success,img = cap.read()
imgResult = img.copy()

White = [0,0,190,43,43,255]
def findColor(img,White):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower = np.array([White[0],White[1],White[2]])
	upper = np.array([White[3],White[4],White[5]])
	mask = cv2.inRange(imgHSV,lower,upper)
	x,y,img=getContours(img,mask)
	cv2.circle(img,(x,y),10,(255,0,0),cv2.FILLED)
	return x,y,img

def getContours(img,mask):
	x,y,w,h = 320,220,0,0
	contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area>300:
			cv2.drawContours(img, cnt, -1, (255,0,0), 3)
			peri = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02*peri,True)
			x, y, w, h = cv2.boundingRect(approx)
	return x+w//2,y+h//2,img

def trackLand():
	fwTopSpeed = 0.5 #m/s
	swTopSpeed = 0.5 #m/s
	vertSpeed = 0.3 #m/s
	refreshRate = 10 #Hz

	time.sleep(1)

	print("starting object tracking")
	while True:
		success,img = cap.read()
		imgResult = img.copy()
		x,y,imgResult = findColor(img, White)
		cv2.imshow("Mask",imgResult)
		#converts 0,0 position from top left corner to center of camera
		x_rel = x-320
		y_rel = 220-y
		print("x location: " + str(x_rel) + "  y location: " + str(y_rel))
		#converts pixel offset to velocity settings
		sw_velocity = x_rel/(320/fwTopSpeed)
		fw_velocity = y_rel/(220/swTopSpeed)
		print("x velocity: " + str(fw_velocity) + "  y velocity: " + str(sw_velocity))
		if cv2.waitKey(int(1000/refreshRate)) >= 0:
			break

"""
def trackLand():
	print("starting object tracking")
	time.sleep(1)
	while True:
		success,img = cap.read()
		imgResult = img.copy()
		x,y,imgResult = findColor(img, White)
		cv2.imshow("Mask",imgResult)
		x_rel = x-320
		y_rel = 220-y
		print("x location: " + str(x_rel) + "  y location: " + str(y_rel))
		sw_velocity = x_rel/640
		fw_velocity = y_rel/440
		print("x velocity: " + str(fw_velocity) + "  y velocity: " + str(sw_velocity))
		if cv2.waitKey(100) >= 0:
			break
"""

#####################################################################################################
########################################Start of Code################################################
#####################################################################################################



trackLand()
#land()



print("end of script")