from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import cv2
import time
import socket
import argparse
import math
import numpy as np
from pymavlink import mavutil


#####################################################################################################
########################################  Functions  ################################################
#####################################################################################################

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 921600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    #print("Mode: %s" % vehicle.mode.name
    #print  "Armable?: %s" % vehicle.is_armable
    return vehicle

def arm():
    while vehicle.channels['6'] < 1500:
    	#print("Waiting for vehicle to become armable...")
    	time.sleep(1)
##    	print("Vehicle is armable")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    while vehicle.armed==False:
        print("Waiting for drone to arm")
        vehicle.armed=True
        time.sleep(1)
        if vehicle.armed==True:
            break
        time.sleep(5)
    print("Vehicle is now armed.")
    print("Props are now spinning")
    return None

def takeoff(targetAltitude):
    #print "Taking off!"
    #print  "Mode: %s" % vehicle.mode.name
    thrust = 0.7
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        #print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAltitude*0.95:
            #print "Reached target altitude"
            break
        time.sleep(1)

def arm_and_takeoff(targetAltitude):
    arm()
    takeoff(targetAltitude)


def move_to_pos(point):
    print("moving to next location")
    vehicle.simple_goto(point)
    time.sleep(30)

def landHome():
    print("returning to launch")
    vehicle.mode = VehicleMode("RTL")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    time.sleep(duration)
# Set up velocity mappings
# velocity_x > 0 => fly North
# velocity_x < 0 => fly South
# velocity_y > 0 => fly East
# velocity_y < 0 => fly West
# velocity_z < 0 => ascend
# velocity_z > 0 => descend

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

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
	x,y=getContours(mask)
	cv2.circle(imgResult,(x,y),10,(255,0,0),cv2.FILLED)
	return x,y

def getContours(img):
	x,y,w,h = 320,220,0,0
	contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area>400:
			cv2.drawContours(imgResult, cnt, -1, (255,0,0), 3)
			peri = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02*peri,True)
			x, y, w, h = cv2.boundingRect(approx)
	return x+w//2,y+h//2

def trackLand():
    print("setting yaw")
    condition_yaw(0,False)
    print("yaw set")
    time.sleep(1)
    print("starting object tracking")
    while True:
        success,img = cap.read()
        imgResult = img.copy()
        x,y = findColor(img, White)
        x_rel = x-320
        y_rel = 220-y
        print("x location: " + str(x_rel) + "  y location: " + str(y_rel))
        sw_velocity = x_rel/640
        fw_velocity = y_rel/440
        send_ned_velocity(fw_velocity,sw_velocity,0.3,0.1)
        if vehicle.location.global_relative_frame.alt<=2:
            print("close to ground, preparing to land")
            vehicle.mode = VehicleMode("Land")
            break

"""
def trackLand():
	print("setting yaw")
	condition_yaw(0,False)
	print("yaw set")
	time.sleep(1)
	while True:
		print("changing velocity")
		success,img = cap.read()
		imgResult = img.copy()
		x,y = findColor(img, White)
		if x==0 and y==0:
			send_ned_velocity(0,0,0.5,1)
		elif x<300:
			if y>240:
				send_ned_velocity(-0.5,-0.5,0.5,1)
			if y<200:
				send_ned_velocity(-0.5,0.5,0.5,1)
			else:
				send_ned_velocity(-0.5,0,0.5,1)
		elif x>340:
			if y>240:
				send_ned_velocity(0.5,-0.5,0.5,1)
			if y<200:
				send_ned_velocity(0.5,0.5,0.5,1)
			else:
				send_ned_velocity(0.5,0,0.5,1)
		else:
			if y>240:
				send_ned_velocity(0,-0.5,0.5,1)
			if y<200:
				send_ned_velocity(0,0.5,0.5,1)
			else:
				send_ned_velocity(0,0,0.5,1)
		if vehicle.location.global_relative_frame.alt<=2:
			vehicle.mode = VehicleMode("Land")
			break
"""

#####################################################################################################
########################################Start of Code################################################
#####################################################################################################


vehicle = connectMyCopter()
point1 = LocationGlobalRelative(28.6107822,-81.2098002,10)
vehicle.airspeed = 5

arm()
takeoff(10)
move_to_pos(point1)
trackLand()
#landHome()



print("end of script")