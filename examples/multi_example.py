from BasicArdu import BasicArdu
from CommonStructs import Frames, Waypoint
from time import sleep

if __name__ == "__main__":
    # Vehicles in gazebo and real life set their home location (lat/lon) and ekf origin (dNorth/dEast coordinate system) when and where they are turned on
    # For multiple drones:
    # If ekf_origin is not specified, then specify the global_home as the same for all drones and do not move them from their turn-on location
    # For specifying the ekf: measure the distance in meters from your desired "origin" where each drone was turned on (dNorth and dEast). That will be your ekf offset

    # example to be run using the simulator command: ./launch_gazebo.sh 2 "Red,Purple" "0.0,15.0" "10.0,0.0"
    v1 = BasicArdu(frame=Frames.NED, connection_string='tcp:10.91.238.66:5762', global_home=[42.47777625687639,-71.19357940183706,174.0], ekf_offset=[0,10])       
    v2 = BasicArdu(frame=Frames.NED, connection_string='tcp:10.91.238.66:5772', global_home=[42.47777625687639,-71.19357940183706,174.0], ekf_offset=[15,0])


    # takeoff v1
    v1.handle_takeoff(5)   
    v1.wait_for_target()   # wait to reach desired location
    sleep(3)
    # goto first waypoint
    v1.handle_waypoint(Frames.NED, 6, 0, -5, 0)
    v1.wait_for_target()
    sleep(3)
    # goto second wayoint
    v1.handle_waypoint(Frames.NED, 0, 5, -5, 0)
    v1.wait_for_target()
    sleep(3)

    v1.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    sleep(1)

    v1.handle_hold()
    v1.wait_for_target()
    sleep(3)

    # goto Home wayoint
    v1.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    v1.wait_for_target()
    sleep(3)
    # land
    v1.handle_landing()


    # takeoff v2
    v2.handle_takeoff(5)   
    v2.wait_for_target()   # wait to reach desired location
    sleep(3)
    # goto first waypoint
    v2.handle_waypoint(Frames.NED, 6, 0, -5, 0)
    v2.wait_for_target()
    sleep(3)
    # goto second wayoint
    v2.handle_waypoint(Frames.NED, 0, 5, -5, 0)
    v2.wait_for_target()
    sleep(3)
    # goto Home wayoint
    v2.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    v2.wait_for_target()
    sleep(3)

    # goto Landing wayoint
    v2.handle_waypoint(Frames.NED, 0, -2, -5, 0)
    v2.wait_for_target()
    sleep(3)

    # land
    v2.handle_landing()