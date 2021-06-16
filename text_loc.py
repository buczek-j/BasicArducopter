from BasicArdu.BasicArdu import BasicArdu, Frames
from argparse import ArgumentParser
from time import sleep, time

def main():
    try:
        parser = ArgumentParser()
        parser.add_argument('--connection_string', type=str, default='/dev/ttyACM0', help='Ardupilot connection string')
        parser.add_argument('--file_name', type=str, default='data', help='log file name')
        options = parser.parse_args()

        # simple use example
        drone = BasicArdu(connection_string=options.connection_string, tolerance_location=1)    # connect to ArduPilot

        # takeoff drone
        drone.handle_takeoff(5)   
        sleep(2)

        start_time = time()
        while True:
            waypoint = input("input waypoint:\n").split(",")

            if waypoint[0] == 'exit' or waypoint[0] == 'break':
                break
            drone.handle_waypoint(Frames.NED, float(waypoint[0]) , float(waypoint[1]), -1.0*abs(float(waypoint[2])), 0)
        
        # land
        drone.handle_landing()
        print("LANDED")

    except Exception as e:
        print(e)
        # land
        drone.handle_landing()


        


if __name__ == '__main__':
    main()
