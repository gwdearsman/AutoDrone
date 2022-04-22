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
    return vehicle

def arm():
    while vehicle.channels['6'] < 1500:
    	#print("Waiting for vehicle to become armable...")
    	time.sleep(1)
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
    print("Taking off!")
    thrust = 0.7
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        #print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAltitude*0.95:
            print("Reached target altitude")
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

#MAV_FRAME_BODY_OFFSET_NED sets NORTH=Forward and EAST=Right
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
    vehicle.send_mavlink(msg)
    time.sleep(duration)


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

def condition_servo(servo_num, pwm_value):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        servo_num,    # param 1, SERVO number
        pwm_value,          # param 2, PWM 1000-2000
        0,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
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
    #converts image to HSV color spectrum, less sensitive to lighting as compared to RGB
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower = np.array([White[0],White[1],White[2]])
	upper = np.array([White[3],White[4],White[5]])
	mask = cv2.inRange(imgHSV,lower,upper)
	x,y,img=getContours(img,mask)
	cv2.circle(img,(x,y),10,(255,0,0),cv2.FILLED)
    #slaps a beautiful blue circle onto the detected object
	return x,y,img

def getContours(img,mask):
    #inputs a masked image from findColor and maps its contours
	x,y,w,h = 320,220,0,0
	contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area>100:
            #removes noise by excluding small areas
			print("Area of Target: " + str())
			cv2.drawContours(img, cnt, -1, (255,0,0), 3)
			peri = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02*peri,True)
			x, y, w, h = cv2.boundingRect(approx)
        #returns the bounding box centerpoint for the detected color
	return x+w//2,y+h//2,img

def trackLand():
    fwTopSpeed = 0.3 #m/s
    swTopSpeed = 0.3 #m/s
    vertSpeed = 0.4 #m/s
    refreshRate = 10 #Hz

    print("setting yaw")
    #Orients drone North. The need for this has been removed after switching
    # to body coordinates from NED. It can remain for debugging purposes
    condition_yaw(0,False)
    print("yaw set")
    time.sleep(1)

    print("starting object tracking")
    while True:
        success,img = cap.read()
        imgResult = img.copy()
        x,y,imgResult = findColor(img, White)
        #converts 0,0 position from top left corner to center of camera
        x_rel = x-320
        y_rel = 220-y
        print("x location: " + str(x_rel) + "  y location: " + str(y_rel))
        print("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        #converts pixel offset to velocity settings
        sw_velocity = x_rel/(320/swTopSpeed)
        fw_velocity = y_rel/(220/fwTopSpeed)
        send_ned_velocity(fw_velocity,sw_velocity,vertSpeed,1/refreshRate)
        if vehicle.location.global_relative_frame.alt<=2:
            #safe landing when approaching ground
            print("close to ground, preparing to drop off")
            send_ned_velocity(0,0,0,1/refreshRate)
            condition_servo(9,2000)
            time.sleep(2)
            vehicle.mode = VehicleMode("RTL")
            break


#####################################################################################################
########################################Start of Code################################################
#####################################################################################################


vehicle = connectMyCopter()
vehicle.airspeed = 5

#points to travel to
point1 = LocationGlobalRelative(28.6105938,-81.2099399,7)

time.sleep(1)
while True:
	while vehicle.channels['6'] < 1500:
		condition_servo(9,400)
		time.sleep(1)
	while vehicle.channels['6'] > 1500:
		condition_servo(9,1000)
		time.sleep(1)
print("end of script")
