from BasicArdu import BasicArdu
from CommonStructs import Frames
from gpiozero import LED, Button
from time import sleep

ALT = 5	# altitude in meters realative to home position
WAYPOINTS = [[10.0, 0.0], [0.0, 5.0]] # [[meters north, meters east], [] ...]
GPIO_OUT=17	
GPIO_IN=2

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(frame=Frames.NED, connection_string='/dev/ttyACM0')    # connect to drone
    trigger_out = LED(GPIO_OUT)
    trigger_in = Button(GPIO_IN)


    # takeoff drone
    drone.handle_takeoff(ALT)  # takeoff alititude: 5 meters
    drone.wait_for_target()   # wait to reach desired location

    for waypoint in WAYPOINTS:
	    # goto waypoint
	    drone.handle_waypoint(Frames.NED, waypoint[0], waypoint[1], -1.0*ALT, 0)    
	    drone.wait_for_target()
	    # ... Code to run at waypoint ...
            print("- - Reached Waypoint - -")
            trigger_out.on()	# trigger the WuR
            sleep(2)
            trigger_out.off()
            while not trigger_in.is_pressed():	# Wait for WuR response
                sleep(1)
            print("- - WuR Trigger Recieved - -")
	    


    
    # goto Home wayoint (starting position)
    drone.handle_waypoint(Frames.NED, 0, 0, -5.0, 0)
    drone.wait_for_target()
    

    # land
    drone.handle_landing()
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly 
