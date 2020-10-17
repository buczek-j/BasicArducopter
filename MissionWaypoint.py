'''
Waypoint Mission Class
'''

import time, argparse, math,random
from threading import Thread
import subprocess
import shlex

from BasicArdu import BasicPX4
from CommonStructs import Waypoint, Frames
from flightcontrolPrimitives import calculateAngle
		
# calculate the POI at every given time along the path

class MissionWaypoint():
	def __init__(self, waypoints, takeoff_alt, connection_string='127.0.0.1:14540', land_at_home=False, measure_throughput=False, mission_message='junk', throughput_script='./runN.sh', nRuns=1, POI=None, POI_offset=0, verbose=True, vXY=12, vZU=3, vZD=1, vYaw=200):
		'''
		Mission Object for waypoint and throughput mission
		:param waypoints: list of Waypoint class objects for waypoint the drone will travel to
		:param takeoff_alt: float for the altitude (meters realative to starting alt) that the drone will takeoff to and preform the mission at
		:param connection_string: string for what device dronekit connects to
		:param land_at_home: bool for if the drone should return to home position to land (True), or land at final waypoint (False)
		:param measure_throughput: bool for if throughput tests should be taken
		:param mission_message: string of the message to be sent during the throughput test
		:param throughput_script: string of the throughput script to execute (.sh file)
		:param nRuns: integer of the number of times to run the throughput tests per waypoint
		:param POI: Waypoint class object of the point of interest to point the drone at for all waypoints
		:param POI_offset: float for the offset (degrees) of the POI if antenna is not directly mounted foward
		:param verbose: bool for extra command line outputs / debugging
		:param vXY: float for the max velocity for the drone to move along the x-y plane (m/s)
		:param vZU: float for the max velocity for the drone to move in the up Z direction (m/s)
		:param vZD: float for the max velocity for the drone to move in the down Z direction (m/s)
		:param vYaw: float for the max yaw rate for the drone (degrees/s)
		'''
		self.pix = BasicPX4(connection_string=connection_string, vXYMax=vXY, vZDnMax=vZD, vZUpMax=vZU, vYawMax=vYaw, verbose=verbose)	# Start PX4 connection

		# Waypoints
		self.waypoints = waypoints
		self.home_ground = Waypoint()				# Initialize home waypoint
		self.home_ground.update(self.pix.vehicle)			# Update with current vehicle position
		self.home_takeoff = Waypoint()

		# Mission Parameters
		self.takeoff_alt = takeoff_alt			# mission altitude in meters (relative)
		self.land_at_home = land_at_home		# boolean for if the vehicle should land at the home position after finishing all waypoints, or just over final waypoint
		self.mission_message = mission_message	
		self.throughput_script = throughput_script
		self.nRuns = nRuns
		self.measure_throughput = measure_throughput
		self.measurement_complete = False
		self.POI = POI
		self.POI_offset = POI_offset
		self.verbose = verbose

		if self.verbose:
			print("~~~ home before take off"),	self.home_ground.print()	
		
	def get_throughput(self, message):
		print("~~~ STARTING SCRIPT ")
		subprocess.call(shlex.split('bash '+ self.throughput_script +' '+str(self.nRuns)+' '+message))
		print("~~~ SCRIPT DONE ")
		self.measurement_complete = True

	def run(self):
		# Takeoff
		self.pix.handle_takeoff(self.takeoff_alt, self.pix.vehicle.heading)	# takeoff with mission alt and current heading
		self.pix.wait_for_target()
		print('- - - Takeoff Successful - - -')

		self.home_takeoff.update(self.pix.vehicle)	# update with current vehicle position

		if self.verbose:
			print("~~~ home after take off"), self.home_takeoff.print()

		# Goto Waypoints
		for waypoint in self.waypoints:
			if self.POI:		# if there is a point of interest, set angle towards it
				#TODO fix the POI calculator
				#angle = waypoint.current_bearing(None, self.POI, self.POI_offset)
				angle = calculateAngle(waypoint, self.POI)
			else:
				angle = waypoint.phi

			self.pix.handle_waypoint(Frames.LLA, waypoint.lat, waypoint.lon, self.home_takeoff.alt, angle)	# goto next waypoint
			self.pix.wait_for_target()																# wait until waypoint is reached
			
			waypoint_start_time = time.time()

			if self.measure_throughput:			# measure the throughput if True
				self.measurement_complete = False
				print("Logging message is {}".format(self.mission_message))
				measureThroughputThread = Thread(target = self.get_throughput, args = (self.mission_message,))
				measureThroughputThread.start()

				while not self.measurement_complete:
					time.sleep(0.5)

			else:	# if no measurement, wait for time specified in the waypoint (default is zero sec)
				while time.time()-waypoint_start_time < waypoint.duration:
					time.sleep(0.5)

		print("~~~ finished waypoints ")

		if self.land_at_home:	# if True, land back at home waypoint, otherwise land at final waypoint
			self.pix.handle_waypoint(Frames.LLA, self.home_takeoff.lat, self.home_takeoff.lon, self.home_takeoff.alt, angle)	# goto home position
			self.pix.wait_for_target()

		# Land Vehicle
		self.pix.handle_landing()
		while self.pix.vehicle.armed:
			time.sleep(2)
			print('. . .')

		print('~~~ Landed ~~~')
		
def main():
	'''
	Sample Mission for simulator tests
	'''
	# position [lat, long, alt, angle, duration]	[47.397751, 8.545607, 498.118, 0, 5]
	positions = [
		[47.3978, 8.545607, 498.118, 0, 10], 
		[47.3978, 8.5457, 498.118, math.pi*90/180, 10],
		[47.3976, 8.545607, 498.118, math.pi*270/180, 10],
	]
	waypoints = []
	for position in positions:
		waypoints.append(Waypoint(x=position[0], y=position[1], z=position[2], compass_angle=position[3], duration=position[4]))

	POI = Waypoint(y=47.397751, x=8.545607, z=498.118)
	#POI=None
	mission = MissionWaypoint(waypoints, 10, True, POI=POI)
	mission.run()

if __name__ == '__main__':
	main()

	

