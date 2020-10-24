# BasicArducopter

## Intel Aero Setup

### 1. Preliminary Setup
If not done already, then follow the [Official Setup Instructions from Intel](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-setup) that ask you to:
* Downloand the latest OS image ("intel-aero-image-intel-aero.iso") from the [Intel Download Center](https://downloadcenter.intel.com/download/26500/UAV-installation-files-for-Intel-Aero-Platform)
* [Flash the BIOS to the Aero compute board](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-setup#flashing-the-bios)
* [Flash the OS to the Aero compute board with a USB drive](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-setup#flash-intel-aero-linux-distribution)
* [Upgrade the FPGA](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-setup#flashing-the-fpga) with the .jam file included with the OS image

### 2. Repository Setup
* Make sure that the following are installed 
```
sudo apt-get install python3 python3-pip git
```
 
 * Clone this repo (Typically in the ~/Documents folder)
 ```
 cd ~/Documents
 git clone https://github.com/buczek-j/BasicArducopter.git
 ```

 * Install the required Python3 Libraries:
 ```
 pip3 install -r BasicArducopter/SetupFiles/requirements.txt
 ```

### 3. Install Arducopter
Either use the _arducopter.px4_ file from this repo (BasicArducopter/SetupFiles/arducopter.px4), or download the latest aero-fc ArduPilot firmware from firmware.ardupilot.org. Documentation for this can be found on the official [flashing page](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-setup#flashing-the-flight-controller-rtf-only). 
* Copy the firmware to the Aero compute board
```
sudo cp arducopter.px4 /etc/aerofc/ardupilot/
cd /etc/aerofc/ardupilot/
```

* Force the bootloader pin into the high state:
```
sudo aerofc-force-bootloader-pin.py 1
```

* Install the Ardupilot version:
```
sudo aerofc-update.sh arducopter.px4
```

* Reset the bootloader pin to low:
```
sudo aerofc-force-bootloader-pin.py 0
```
* Reboot the Intel Aero

### 4. Mission Planner / QGroundcontrol Setup
Next the parameters need to be configured, this is normally done with Mission Planner (Currently only supported by Windows), but should also be possible using QGroundcontrol as they both support Mavlink message communication.
* Login to the Intel Aero compute board and connect it to the same wifi network as your computer that is running Mission Planner. 
* Open a terminal onthe Aero and get the IP address of the Aero compute board
```
ifconfig
```
* Use Mission Planner to connect to the Aero over the Network. Select the 'TCP' connection option on the top right, click connect, input the IP address of the Aero, and ensure that the port number is 5760.
* Select the 'Config' Top-bar tab, and the 'Full Parameter Tree' Side-bar tab
* From this Repo, download the parameter file from the /SetupFiles/ folder (ex: 'aero_param_list(RC-Mode1).param'). On the right side of the Mission Planner screen, press the 'Load from file' button and select the downloaded .param file. Note: that the (RC-Mode1) files are for Mode-1 transmitter layouts (throttle on the right hand). If you fly with the Mode-2 layout (throttle on the left hand), you may need to reverse some of the RC channel inputs (channels 1-4) by changing the 'RC1_REVERSED' parameter in Mission Planner.
* All the parameters have been loaded, now just complete all of the Calibrations under the 'Initial Setup' -> 'Mandatory hardware' tabs.
    * Accel Calibration
    * Level Horizon 
    * Radio Control Calibration
    * Compass Calibration
* Reboot the Intel Aero

### 5. Creating and Running Mission Files
Missions are very easy to make with the BasicArdu wrapper. Here is an example mission:
```
from BasicArdu import BasicArdu

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(frame=Frames.NED, connection_string='tcp:127.0.0.1:5760')    # connect to Intel Aero using 'North, East, Down' Reference Basis


    # takeoff drone
    drone.handle_takeoff(5)  # takeoff alititude: 5 meters
    drone.wait_for_target()   # wait to reach desired location

    
    # goto first waypoint
    drone.handle_waypoint(Frames.NED, 10.0, 0, -5.0, 0)    # 10 meters North, 0 meters East, -5 meters Down, Yaw angle 0rad (North) 
    drone.wait_for_target()
    # ... Code to run at first waypoint ...


    # goto second wayoint
    drone.handle_waypoint(Frames.NED, 0, 5.0, -5.0, 3.14/2) # 0 meters North, 5 meters East, -5 meters Down, Yaw angle pi/2 rad (East)
    drone.wait_for_target()
    # ... Code to run at third waypoint ...


    # goto Home wayoint (starting position)
    drone.handle_waypoint(Frames.NED, 0, 0, -5.0, 0)
    drone.wait_for_target()
    

    # land
    drone.handle_landing()
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly (python3 filename.py)
```