from duav_utils import connect_to_drone, arm_and_takeoff
from dronekit import VehicleMode

def main():
    drone = connect_to_drone()

    arm_and_takeoff(drone, 10)

    print('Return to launch')
    drone.mode = VehicleMode("RTL")

if __name__=="__main__":
    main()