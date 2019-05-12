from dronekit import connect, Command, LocationGlobal, VehicleMode

import time

def connect_to_drone():
    # Connect to the drone
    print("[INFO] Connecting to drone")
    connection_string = '127.0.0.1:14551'
    vehicle = connect(connection_string, wait_ready=True)

    # Display basic drone state
    print(" Type: %s" % (vehicle._vehicle_type))
    print(" Armed: %s" % (vehicle.armed))
    print(" System status: %s" % (vehicle.system_status.state))
    print(" GPS: %s" % (vehicle.gps_0))
    print(" Alt: %s" % (vehicle.location.global_relative_frame.alt))

    return vehicle

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

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break

        time.sleep(1)
