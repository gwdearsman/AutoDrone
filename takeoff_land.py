from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse




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
    while vehicle.channels['6'] < 1500:
    	print("Waiting for vehicle to become armable...")
    	time.sleep(1)
##    	print("Vehicle is armable")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    print  "Mode: %s" % vehicle.mode.name
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for drone to arm")
        time.sleep(1)

    print("Vehicle is now armed.")
    print("Props are now spinning")

    return None

def takeoff(targetAltitude):
    print "Taking off!"
    print  "Mode: %s" % vehicle.mode.name
    thrust = 0.7
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAltitude*0.95:
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
    vehicle.mode = VehicleMode("RTL")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
# Set up velocity mappings
# velocity_x > 0 => fly North
# velocity_x < 0 => fly South
# velocity_y > 0 => fly East
# velocity_y < 0 => fly West
# velocity_z < 0 => ascend
# velocity_z > 0 => descend

#####################################################################################################
########################################Start of Code################################################
#####################################################################################################



vehicle = connectMyCopter()
point1 = LocationGlobalRelative(28.6107822,-81.2098002,20)
vehicle.airspeed = 5


arm()
takeoff(20)
move_to_pos(point1)
send_ned_velocity(3,0,0,3)
send_ned_velocity(0,3,0,3)
send_ned_velocity(0,0,-3,3)
send_ned_velocity(-3,-3,3,3)

land()



print("end of script")


