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
White = [57,0,180,102,65,255]
def findColor(img,White):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower = np.array([White[0],White[1],White[2]])
	upper = np.array([White[3],White[4],White[5]])
	mask = cv2.inRange(imgHSV,lower,upper)
	x,y=getContours(mask)
	cv2.circle(imgResult,(x,y),10,(255,0,0),cv2.FILLED)
	return x,y

def getContours(img):
	x,y,w,h = 0,0,0,0
	contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area>500:
			cv2.drawContours(imgResult, cnt, -1, (255,0,0), 3)
			peri = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02*peri,True)
			x, y, w, h = cv2.boundingRect(approx)
	return x+w//2,y+h//2

def trackLand():
	condition_yaw(0)
	time.sleep(1)
	while True:
		time.sleep(1)
		success,img = cap.read()
		imgResult = img.copy()
		x,y = findColor(img, White)
		if x==0 and y==0:
			print("down")
		elif x<300:
			if y>240:
				print("SW")
			if y<200:
				print("NW")
			else:
				print("West")
		elif x>340:
			if y>240:
				print("SE")
			if y<200:
				print("NE")
			else:
				print("E")
		else:
			if y>240:
				print("South")
			if y<200:
				print("North")
			else:
				print("down")


#####################################################################################################
########################################Start of Code################################################
#####################################################################################################



trackland()
#land()



print("end of script")