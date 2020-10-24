'''
Basic Dronekit Wrapper for ArduPilot controller
'''

from time import sleep
from math import pi
from MavLowLevel import *
from CommonStructs import Frames, Waypoint

class BasicArdu():
    def __init__(self, frame=Frames.LLA, verbose=False, connection_string='tcp:127.0.0.1:5760', tolerance_location=2.0):
        '''
        Dronekit wrapper class for Ardupilot
        :param frame: vehicle coordinate frame
        :param verbose: boolean for extra text outputs
        :param connection_string: string of ip address and port to connect 
        :param tolerance_location: float for tolerance for reaching waypoint (m)
        '''
        # Vehicle Connection 
        while True:
            try:
                self.vehicle = connect(connection_string, wait_ready=False)
                break
            except:
                pass    
        
        if self.vehicle.armed:                # If the vehicle is already in the air, set to standby mode
            self.vehicle.mode = "GUIDED"

        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            sleep(2)

        # Set Vehicle Parameters
        # self.vehicle.parameters['COM_OF_LOSS_T'] = 3600		            # set offboard failsafe time (s) min: 0 max: 60? set to 3600s = 1hr
        # self.vehicle.parameters['MPC_LAND_SPEED'] = vLandingMax 		# Landing descend rate (m/s)		[default: 0.7]
        # self.vehicle.parameters['MPC_XY_VEL_MAX'] = vXYMax		        # maximum x-y plane velocity (m/s)	[default: 12]
        # self.vehicle.parameters['MPC_Z_VEL_MAX_UP'] = vZUpMax	        # maximum z up velocity (m/s)		[deffault: 3]
        # self.vehicle.parameters['MPC_Z_VEL_MAX_DN'] = vZDnMax	        # maximum z down velocity (m/s) 	[default: 1]
        # self.vehicle.parameters['MC_YAWRATE_MAX '] = vYawMax	        # maximum yaw rate (deg/s)	        [default: 200]

        # Initialize flight variables
        self.frame = frame
        self.tolerance_location = tolerance_location # minimum distance variance to waypoint (meters)
        self.target_waypoint = Waypoint(self.frame)                   # current target waypoint
        self.target_waypoint.update(self.vehicle)

        self.verbose = verbose

        # Download the vehicle waypoints (commands). Wait until download is complete. Necessary for self.vehicle.home_location to be set
        cmds = self.vehicle.commands # https://dronekit.netlify.com/automodule.html#dronekit.Vehicle.home_location
        cmds.download()
        cmds.wait_ready()
        print(' - - - Initialization Successful - - -')

    def handle_arm(self):
        '''
        Method to arm the vehicle
        '''
        if self.verbose:
            print('> Arming')
        self.handle_guided()
        self.vehicle.armed=True
        
    def handle_guided(self):
        '''
        Method to set the flight mode to Offboard
        '''
        if self.verbose:
            print('> Set Offboard')

        self.vehicle.mode = "GUIDED"

    def handle_kill(self):
        '''
        Method to emergency stop motors
        '''
        kill_vehicle(self.vehicle)
        if self.verbose:
            print('> Emergency Stop')
    
    def handle_takeoff(self, alt, phi=0):
        '''
        Method to takeoff the vehicle
        '''
        self.target_waypoint = Waypoint(self.frame)     # Clear the target waypoint
        self.target_waypoint.update(self.vehicle)       # Update the target location to the current location of the vehicle
        self.target_waypoint.phi = phi
        print('~~ Take Off ~~')

        self.vehicle.mode = "GUIDED"
        self.vehicle.armed=True

        # We want for the motors to arm before we takeoff
        while not self.vehicle.armed:
            print("Waiting for arming...")
            sleep(0.5)

        self.vehicle.simple_takeoff(alt)
        
        # Set the vehicle to go to the target waypoint with adjusted altitude
        if self.target_waypoint.frame == Frames.LLA:
            self.target_waypoint.alt = self.target_waypoint.alt + alt
            
        elif self.target_waypoint.frame == Frames.NED:
            self.target_waypoint.dDown = self.target_waypoint.dDown - alt
            
    def handle_landing(self):
        '''
        Method to land the vehicle
        '''
        print("~~ Landing ~~")
        land_vehicle(self.vehicle)

    def handle_waypoint(self, frame, x, y, z, phi=0):
        '''
        Method to send the vehicle to a new waypoint
        :param frame: Frame Enum frame of reference for the waypoint command
        x, y, z depend on the frams
        '''
        if self.verbose:
            print('> Waypoint CMD')

        if frame.value == Frames.VEL.value:   # velocity command
            print('VELOCITY')
            velocity_cmd_NED(self.vehicle, x, y, z, phi)
            self.target_waypoint=None 
        else:
            if frame.value == Frames.LLA.value:
                print('LLA')
                self.target_waypoint = Waypoint(frame=Frames.LLA, x=x, y=y, z=z, compass_angle=phi)
                waypoint_cmd_LLA(self.vehicle, x, y, z, phi)
            elif frame.value == Frames.NED.value:
                print('NED')
                self.target_waypoint = Waypoint(frame=Frames.NED, x=x, y=y, z=z, compass_angle=phi)
                waypoint_cmd_NED(self.vehicle, x, y, z, phi)
            self.target_waypoint.print()

    def handle_hold(self):
        '''
        Method to stop the vehicle from continuing to its current target location
        '''
        if self.verbose:
            print('> Hold')

        self.target_waypoint = Waypoint(self.frame)     # Clear the target waypoint
        self.target_waypoint.update(self.vehicle)       # Update the target location to the current location of the vehicle
        # Set the vehicle to go to the target waypoint
        if self.target_waypoint.frame == Frames.LLA:
            waypoint_cmd_LLA(self.vehicle, self.target_waypoint.lat, self.target_waypoint.lon, self.target_waypoint.alt, self.target_waypoint.phi)
        
        elif self.target_waypoint.frame == Frames.NED:
            waypoint_cmd_NED(self.vehicle, self.target_waypoint.dNorth, self.target_waypoint.dEast, self.target_waypoint.dDown, self.target_waypoint.phi)

    def reached_target(self):
        '''
        Method to check if the vehicle has reached its target location
        :return: Boolean for if the target has been reached
        '''
        if self.target_waypoint:
            # print(self.target_waypoint.current_distance(self.vehicle))
            if self.target_waypoint.current_distance(self.vehicle) <= self.tolerance_location:
                self.target_waypoint = None 
            else:
                return False
        
        else:               # if the drone has reached its target, then the target waypoint is set to None
            return True

    def wait_for_target(self):
        ''' 
        Method to delay code progression until the target location has been reached
        '''
        while not self.reached_target():
            sleep(0.5)

        if self.verbose:
            print('Reached Target')
    
    def get_LLA(self):
        '''
        Method to return the current Lattitude, Longitude, and msl Altitude of the vehicle
        :return (float) latitude, (float) longitude, (float) altitude msl
        '''
        return self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt

def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone = BasicArdu(frame=Frames.NED)    # connect to ArduPilot

    # takeoff drone
    drone.handle_takeoff(5)   
    drone.wait_for_target()   # wait to reach desired location
    sleep(3)
    # goto first waypoint
    drone.handle_waypoint(Frames.NED, 10, 0, -5, 0)
    drone.wait_for_target()
    sleep(3)
    # goto second wayoint
    drone.handle_waypoint(Frames.NED, 0, 5, -5, 0)
    drone.wait_for_target()
    sleep(3)
    # goto Home wayoint
    drone.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    drone.wait_for_target()
    sleep(3)
    # land
    drone.handle_landing()


if __name__ == '__main__':
    main()













