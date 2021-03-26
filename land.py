import dronekit
d = dronekit.connect('/dev/ttyACM0')
d.mode="LAND"
