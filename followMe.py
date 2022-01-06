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


def condition_yaw(heading, relative=True):
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
        
    out.write(img.astype('uint8'))
    cv2.imshow('img', img)
    return boxes


def track():
    fwTopSpeed = 0.3 #m/s
    swTopSpeed = 0.3 #m/s
    vertSpeed = 0.4 #m/s
    refreshRate = 10 #Hz

    print("setting yaw")
    #Orients drone North. The need for this has been removed after switching
    # to body coordinates from NED. It can remain for debugging purposes
    condition_yaw(0,True)
    print("yaw set")
    time.sleep(1)

    print("starting object tracking")
    while True:
        success,img = cap.read()
        imgResult = img.copy()
        xA, yA, xB, yB = findPerson(img)
        #converts 0,0 position from top left corner to center of camera
        area = (xB-xA)*(yB-yA)
        x_rel = xA-160
        y_rel = 120-yA
        angle = x_rel/4
        if abs(angle) < 4:
            angle = 0
        condition_yaw(angle,True)
        if area<8000:
            if area<50:
                speed = 0
            else:
                speed = fwTopSpeed = 0.3 #m/s
        up_velocity = y_rel/(120/vertSpeed)
        send_ned_velocity(speed,0,up_velocity,1/refreshRate)



#####################################################################################################
########################################Start of Code################################################
#####################################################################################################


vehicle = connectMyCopter()
vehicle.airspeed = 5

#points to travel to
point1 = LocationGlobalRelative(28.6105938,-81.2099399,7)

time.sleep(1)
condition_servo(9,1000)
print("servo activated")
arm()
takeoff(10)
move_to_pos(point1)
trackLand()

print("end of script")
