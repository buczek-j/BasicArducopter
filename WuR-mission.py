from BasicArdu import BasicArdu, Frames

import subprocess
from threading import Thread
from time import sleep, time
import csv

ALT = 5	# altitude in meters realative to home position
WAYPOINTS = [[0.0, 0.0, -5.0], [10.0, 0.0, -5.0]] # [[meters north, meters east, meters down], [] ...]


class Serial_Logger():
    def __init__(self, get_loc):
        '''
        method to initialize the Serial logger object
        :param get_loc: Method to get the location of the drone
        '''
        self.file = open( 'data.txt' , 'a' )

        self.get_loc = get_loc
        self.test_complete = False                              # Bool to goto next waypoint


    def run(self, stop):
        '''
        method to run in thread
        :param stop: Method returning true/false to stop the thread
        '''
        print("RUNNING THREAD",)
        self.loop = True
        self.process = subprocess.Popen( 'java ~/Documents/tinyos-main/tools/tinyos/java/net.tinyos.tools.PrintfClient -comm serial@/dev/ttyUSB0:telosb' , shell = True , stdout = subprocess.PIPE )
        
        while stop() == False and self.loop==True:   # TODO setup path
            output = self.process.stdout.readline().decode()
            if self.process.poll() is not None and output == '':
                self.loop = False

            if output:
                self.file.write( output.strip() + '\n' )
                
                if 'Ping #6'in output:
                    self.test_complete = True
                    self.file.write( self.get_loc() + '\n' )
            

        print('Thread Stopped')


# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(frame=Frames.NED, connection_string='tcp:192.168.10.2:5762', global_home=None)    # connect to drone  '/dev/ttyACM0'
    stop_threads = False

    def get_location():
        '''
        Method to return the location of the drone (LLA)
        :return:string of Lat,Lon,Alt,time,meters_north,meters_east,meters_down
        '''
        return str(drone.vehicle.location.global_frame.lat) +', '+ str(drone.vehicle.location.global_frame.lon)+', '+ str(drone.vehicle.location.global_frame.alt) +', '+ + str(time()) + +', '+ str(drone.vehicle.location.local_frame.north) + ', '+str(drone.vehicle.location.local_frame.east)+ ', ' +str(drone.vehicle.location.local_frame.down)


    serial_logger = Serial_Logger(get_location)

    # takeoff drone
    drone.handle_takeoff(ALT)  # takeoff alititude: 5 meters
    drone.wait_for_target()   # wait to reach desired location

    # Start Logger Thread
    serial_thread = Thread(target=serial_logger.run, args=(lambda : stop_threads,))
    serial_thread.start()
    sleep(0.1)

    for waypoint in WAYPOINTS:
        # goto waypoint
        drone.handle_waypoint(Frames.NED, waypoint[0], waypoint[1], -1.0*abs(waypoint[2]), 0)    
        drone.wait_for_target()

        print("- - Reached Waypoint - -")
        serial_logger.test_complete = False
        while serial_logger.test_complete == False:
            sleep(0.5)
            print('Waiting . . . ')

    # goto Home wayoint (starting position)
    drone.handle_waypoint(Frames.NED, 0, 0, -5.0, 0)
    drone.wait_for_target()
    
    # land
    drone.handle_landing()

    # Stop Threads
    stop_threads = True
    
    serial_logger.file.close()
    serial_thread.join()
    
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly 
