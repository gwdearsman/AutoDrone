from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse



def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 921600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    print  "Mode: %s" % vehicle.mode.name
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    print("Vehicle is armable")

    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for drone to arm")
        time.sleep(1)

    print("Vehicle is now armed.")
    print("Props are now spinning")

    return None


vehicle = connectMyCopter()
arm()
print("end of script")

