from BasicArdu import BasicArdu, Frames

'''
Program to print out the location and other telemetry from the drone for debugging
'''

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(frame=Frames.NED, connection_string='/dev/ttyACM0')    # connect to drone

    while True:
        a = input("Press Enter to Continue")
        print('\n',drone.vehicle.location.global_frame.lat, drone.vehicle.location.global_frame.lon, drone.vehicle.location.global_frame.alt, drone.vehicle.battery.voltage, end='\n\n')

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly 
