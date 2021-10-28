from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse


vehicle.airspeed = 5
point1 = LocationGlobalRelative(28.6107822,-81.2098002,20)


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
    print  "Mode: %s" % vehicle.mode.name
    print  "Armable?: %s" % vehicle.is_armable
    
    return vehicle

def arm():
    while vehicle.mode.name != "AUTO":
    	print("Waiting for vehicle to become armable...")
    	time.sleep(1)
##    	print("Vehicle is armable")
    vehicle.mode.name="STABALIZE"
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for drone to arm")
        time.sleep(1)

    print("Vehicle is now armed.")
    print("Props are now spinning")

    return None

def takeoff(targetAltitude):
    vehicle.mode.name = "GUIDED"
    print "Taking off!"
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

def arm_and_takeoff(targetAltitude):
    arm()
    takeoff(targetAltitude)


def move_to_pos(point):
    vehicle.simple_goto(point)
    time.sleep(30)
    print "moving to next location"

def land():
    print "returning to launch"
    vehicle.mode.name = "RTL"



#####################################################################################################
########################################Start of Code################################################
#####################################################################################################
vehicle = connectMyCopter()
arm_and_takeoff(20)
move_to_pos(point1)
land()



print("end of script")


