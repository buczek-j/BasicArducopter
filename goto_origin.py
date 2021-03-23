from BasicArdu import BasicArdu
from CommonStructs import Frames
from gpiozero import LED, Button
from time import sleep

'''
Mission for drone to go to the specified waypoint and land, used to get the location on the ground 
'''
ALT = 5	# altitude in meters realative to home position
WAYPOINTS = [[100.0, 0.0, -5.0, 0.0]] # [[meters north, meters east, meters down, delay in sec], [] ...]

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(frame=Frames.NED, connection_string='/dev/ttyACM0')    # connect to drone

    # takeoff drone
    drone.handle_takeoff(ALT)  # takeoff alititude: 5 meters
    drone.wait_for_target()   # wait to reach desired location

    for waypoint in WAYPOINTS:
        # goto waypoint
        drone.handle_waypoint(Frames.NED, waypoint[0], waypoint[1], -1.0*abs(waypoint[2]), 0)    
        drone.wait_for_target()
        # ... Code to run at waypoint ...
        print("- - Reached Waypoint - -", waypoint)
        sleep(waypoint[3])  # delay
    
    # land
    drone.handle_landing()
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly 
