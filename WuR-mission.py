from BasicArdu import BasicArdu, Frames

import subprocess
from threading import Thread
from time import sleep, time
import csv, io, serial

ALT = 5	# altitude in meters realative to home position
WAYPOINTS = [[0.0, 0.0, -5.0], [10.0, 0.0, -5.0]] # [[meters north, meters east, meters down], [] ...]


class Serial_Logger():
    def __init__(self, get_loc,filename='data.txt', mote_is_sniffer = False , serial_port_name = '/dev/ttyUSB0' , stream_type = 'text'):
        '''
        method to initialize the Serial logger object
        :param get_loc: Method to get the location of the drone
        '''
        print( 'SerialSaver started.' , flush = True )
        print( '\tFilename: ' , filename , flush = True )
        print( '\tMote is sniffer?: ' , mote_is_sniffer , flush = True )
        print( '\tSerial port name: ' , serial_port_name , flush = True )
        print( '\tStream type: ' , stream_type , flush = True )
        
        self.file = open( filename , 'a' )
        self.mote_is_sniffer= mote_is_sniffer
        self.serial_port_name = serial_port_name
        self.stream_type = stream_type

        self.get_loc = get_loc
        self.test_complete = False                              # Bool to goto next waypoint


    def run(self, stop):
        '''
        method to run in thread
        :param stop: Method returning true/false to stop the thread
        '''
        print("RUNNING THREAD",)
        loop = True

        text_buffer = str()
        
        if self.stream_type == 'bytes':
            ser = serial.Serial( self.serial_port_name , baudrate = 115200 if not self.mote_is_sniffer else 57600 , timeout = 0 )

            while stop() == False and loop==True:
                bytes_in = ser.read( ser.inWaiting() )

                if len( bytes_in ) > 0:
                    text_in = str( bytes_in )[ 2:-1 ]

                    while r'\x00d' in text_in:
                        text_in = text_in[ ( text_in.index( r'\x00d' ) + len( r'\x00d' ) ): ]

                        if r'\x' in text_in:
                            text_buffer += text_in[ :text_in.index( r'\x' ) ]

                        else:
                            text_buffer += text_in
                        
                        if "Ping #6" in text_buffer:
                            self.test_complete = True

                if r'\n' in text_buffer:
                    #print( text_buffer[ :text_buffer.index( r'\n' ) ] , flush = True )
                    self.file.write( text_buffer[ :text_buffer.index( r'\n' ) ] + '\n' )
                    text_buffer = text_buffer[ ( text_buffer.index( r'\n' ) + len( r'\n' ) ): ]

        elif self.stream_type == 'text':
            ser = serial.Serial( self.serial_port_name , baudrate = 115200 if not self.mote_is_sniffer else 57600 , timeout = 0 )
            sio = io.TextIOWrapper( io.BufferedReader( ser ) , encoding = 'latin-1' )

            while stop() == False and loop==True:
                text_in = sio.readline()

                if len( text_in ) > 0:
                    text_buffer += text_in

                if '\x00d' in text_buffer and '\n' in text_buffer:
                    text_out = text_buffer[ ( text_buffer.index( '\x00d' ) + len( '\x00d' ) ):text_buffer.index( '\n' ) ]

                    if len( text_out ) > 28:

                        #print( text_out[ :28 ] + text_out[ 41: ] , flush = True )
                        self.file.write( text_out[ :28 ] + text_out[ 41: ] + '\n' )

                    else:
                        #print( text_out , flush = True )
                        self.file.write( text_out + '\n' )

                    text_buffer = text_buffer[ ( text_buffer.index( '\n' ) + len( '\n' ) ): ]
                    if "Ping #6" in text_buffer:
                        self.test_complete = True

        elif self.stream_type == 'java':
            process = subprocess.Popen( 'java net.tinyos.tools.PrintfClient -comm serial@' + self.serial_port_name + ( ':telosb' if not self.mote_is_sniffer else ':57600' ) , shell = True , stdout = subprocess.PIPE )

            while stop() == False and loop==True:
                text_buffer = process.stdout.readline().decode()
                
                if process.poll() is not None and text_buffer == '':
                    loop = False
                
                if "Ping #6" in text_buffer:
                    self.test_complete = True
                    
                if text_buffer:
                    #print( text_buffer.strip() , flush = True )
                    self.file.write( text_buffer.strip() + '\n' )

        else:

            print( 'Error: the stream type has to be one of the following:' , flush = True )
            print( '\tbytes' , flush = True )
            print( '\ttext' , flush = True )
            print( '\tjava' , flush = True )


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
    try:
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

    except Exception as e:
        print(e)

    finally:
        # goto Home wayoint (starting position)
        drone.handle_waypoint(Frames.NED, 0, 1, -5.0, 0)
        drone.wait_for_target()

        # land
        drone.handle_landing()

        # Stop Threads
        stop_threads = True
        
        serial_logger.file.close()
        serial_thread.join()
    
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly 
