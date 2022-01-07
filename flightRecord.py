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
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1280,720))
while(cap.isOpened()):
    ret, frame = cap.read()
    if(ret==True):
        out.write(frame)
        cv2.imshow('output',frame)
        if cv2.waitKey(1) > 0:
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()



