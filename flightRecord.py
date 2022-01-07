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
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 20., (640,480))
while(True):
    success,img = cap.read()
    img = cv2.resize(img, (640, 480))
    out.write(img.astype('uint8'))
    cv2.imshow("Video", img)
    if cv2.waitKey(1) > 0:
        break



