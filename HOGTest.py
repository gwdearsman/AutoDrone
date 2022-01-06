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

cap = cv2.VideoCapture(0)
success,img = cap.read()
imgResult = img.copy()
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


#write output video

def findPerson(img):
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 5., (320,240))
    x,y,w,h = 160,120,0,0
    #resizing for smaller file
    img = cv2.resize(img, (320, 240))
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    boxes, weights = hog.detectMultiScale(img, winStride=(8,8))
    boxes = np.array([[x,y,x+w,y+h] for (x,y,w,h) in boxes])

    for (xA, yA, xB, yB) in boxes:
        #display te detected boxes in img
        cv2.rectangle(img, (xA, yA), (xB, yB), (255,0,0), 2)
        x,y,w,h = xA,yA,xB,yB
        
    out.write(img.astype('uint8'))
    cv2.imshow('img', img)
    return x,y,w,h


def track():
    fwTopSpeed = 0.3 #m/s
    swTopSpeed = 0.3 #m/s
    vertSpeed = 0.4 #m/s
    refreshRate = 10 #Hz

    print("yaw set")
    time.sleep(1)

    print("starting object tracking")
    while True:
        success,img = cap.read()
        imgResult = img.copy()
        xA, yA, xB, yB = findPerson(imgResult)
        #converts 0,0 position from top left corner to center of camera
        area = (xB-xA)*(yB-yA)
        x_rel = xA-160
        y_rel = 120-yA
        angle = x_rel/4
        if abs(angle) < 4:
            angle = 0
        print("yaw rotating by " + str(angle) + " degrees")
        if area<8000:
            if area<50:
                speed = 0
            else:
                speed = fwTopSpeed = 0.3 #m/s
        up_velocity = y_rel/(120/vertSpeed)
        
        print("area = " + str(area))
        print("moving at " + str(speed) + " m/s forward")
        print("moving at " + str(up_velocity) + "m/s vertically")

#####################################################################################################
########################################Start of Code################################################
#####################################################################################################

track()

print("end of script")