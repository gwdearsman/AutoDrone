import cv2
import time
import socket
import argparse
import math

cap = cv2.VideoCapture(0)

while True:
	success,img = cap.read()
	cv2.imshow("Image",img)
	if cv2.waitKey(1) >= 0:
		break
