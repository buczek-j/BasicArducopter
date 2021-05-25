from BasicArdu.BasicArdu import BasicArdu, Frames
from argparse import ArgumentParser
from time import sleep, time

def main():
    parser = ArgumentParser()
    parser.add_argument('--connection_string', type=str, default='/dev/ttyACM0', help='Ardupilot connection string')
    options = parser.parse_args()

    # simple use example
    drone = BasicArdu(connection_string=options.connection_string, tolerance_location=1)    # connect to ArduPilot

    # takeoff drone
    drone.handle_takeoff(5)   
    sleep(3)
    start_time = time()
    while drone.vehicle.battery.voltage > 28.8:
    	# goto first waypoint (5m north, 0 meters east, 5 meters up, facing North)
    	drone.handle_waypoint(Frames.NED, 5, 0, -5, 0)
    	sleep(3)
	
    	# goto second wayoint(5m north, 5 meters east, 5 meters up, facing North)
    	drone.handle_waypoint(Frames.NED, 5, 5, -5, 0)
    	sleep(3)

        drone.handle_waypoint(Frames.NED, 0, 5, -5, 3.14)
    	sleep(3)
	
    	# goto Home wayoint (0m north, 0 meters east, 5 meters up, facing North)
    	drone.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    	sleep(3)
    	print("Flight Time:", time()-start_time, "Battery:", drone.vehicle.battery.voltage)

    # land
    drone.handle_landing()


if __name__ == '__main__':
    main()