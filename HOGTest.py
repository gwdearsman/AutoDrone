from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import cv2
import time
import socket
import argparse
import math
import numpy as np
from pymavlink import mavutil


#####################################################################################################
########################################Start of OpenCV################################################
#####################################################################################################

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

cap = cv2.VideoCapture(0)

#write output video
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 15., (640,480))

def findPerson():
    while(True):
        ret, img = cap.read()

        #resizing for smaller file
        img = cv2.resize(img, (640, 480))
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        boxes, weights = hog.detectMultiScale(img, winStride=(8,8))
        boxes = np.array([[x,y,x+w,y+h] for (x,y,w,h) in boxes])

        for (xA, yA, xB, yB) in boxes:
            #display te detected boxes in img
            cv2.rectangle(img, (xA, yA), (xB, yB), (255,0,0), 2)
        
        out.write(img.astype('uint8'))
        cv2.imshow('img', img)
        if cv2.waitKey(1) > 0:
            break


#####################################################################################################
########################################Start of Code################################################
#####################################################################################################

findPerson()

print("end of script")