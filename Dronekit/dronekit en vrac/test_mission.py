import numpy as np
import argparse
import imutils
import time, sys, math
import cv2
import os
from multiprocessing import Process

#dronekit imports
from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil

def connect_to_drone():

    # Connect to the drone
    print("[INFO] Connecting to drone")
    connection_string = '127.0.0.1:14551'
    vehicle = connect(connection_string, wait_ready=True)

    # Display basic drone state
    print(" Type: %s" %(vehicle._vehicle_type))
    print(" Armed: %s" %(vehicle.armed))
    print(" System status: %s" %(vehicle.system_status.state))
    print(" GPS: %s" %(vehicle.gps_0))
    print(" Alt: %s" %(vehicle.location.global_relative_frame.alt))
    
    return vehicle

def readmission(missionFile):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s"%(missionFile))
    missionlist=[]
    with open(missionFile) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
        
def upload_mission(vehicle, missionFile):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(missionFile)
    
    print("\nUpload mission from a file: %s" % missionFile)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print('Upload mission')
    vehicle.commands.upload()

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        
        time.sleep(1)

def main():

    drone = connect_to_drone()

    upload_mission(drone, "mission1.waypoints")

    arm_and_takeoff(drone, 10)

    drone.mode = VehicleMode("AUTO")

    time.sleep(10)
    
    drone.mode = VehicleMode("LAND")

    arm_and_takeoff(drone, 10)

    time.sleep(10)

    drone.mode = VehicleMode("AUTO")

if __name__=='__main__':
    main()