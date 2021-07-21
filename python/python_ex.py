from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, Coordinates)
from time import sleep

serial_port = get_first_pozyx_serial_port()

if serial_port is not None:
	pozyx = PozyxSerial(serial_port)
	print("Connection success!")
else:
	print("No Pozyx port was found")
	exit(1)

pozyx.setHeight(390) # For 2D and 2.5D positioning

while True:
	coordinates = Coordinates()
	pozyx.getCoordinates(coordinates)
	print(coordinates)
	sleep(0.1)
