#!/usr/bin/env python3

'''
Method to takeoff drone and ppilot it to the home location
'''

from BasicArdu import BasicArdu
from CommonStructs import Frames, Waypoint
from time import sleep

if __name__ == "__main__":
    # Vehicles in gazebo and real life set their home location (lat/lon) and ekf origin (dNorth/dEast coordinate system) when and where they are turned on
    # For multiple drones:
    # If ekf_origin is not specified, then specify the global_home as the same for all drones and do not move them from their turn-on location

    v1 = BasicArdu(frame=Frames.NED, connection_string='tcp:127.0.0.1:5760', global_home=[42.47777625687639,-71.19357940183706,174.0])       

    # takeoff v1
    v1.handle_takeoff(5)   
    v1.wait_for_target()   # wait to reach desired location

    # goto Home wayoint
    v1.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    v1.wait_for_target()

    # land
    v1.handle_landing()

