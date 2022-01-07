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
while(true):
    success,img = cap.read()
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 5., (320,240))
    img = cv2.resize(img, (320, 240))
    out.write(img.astype('uint8'))


