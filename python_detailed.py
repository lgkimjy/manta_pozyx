from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, Coordinates, Acceleration, AngularVelocity, EulerAngles, LinearAcceleration, Magnetic, MaxLinearAcceleration, Pressure, Quaternion, Temperature)
from time import sleep

serial_port = get_first_pozyx_serial_port()

if serial_port is not None:
	pozyx = PozyxSerial(serial_port)
	print("Connection success!")
else:
	print("No Pozyx port was found")
	exit(1)

# pozyx.setHeight(1000) # For 2D and 2.5D positioning

def printCoordinates():
	coordinates = Coordinates()
	pozyx.getCoordinates(coordinates)
	print("Coordinates: ", coordinates.x, ", ", coordinates.y, ", ", coordinates.z)


def printAcceleration():
	acceleration_mg = Acceleration()
	pozyx.getAcceleration_mg(acceleration_mg)
	print("Acceleration: ", acceleration_mg.x, ", ", acceleration_mg.y, ", ", acceleration_mg.z)

def printAngularVelocity():
	angular_velocity_dps = AngularVelocity()
	pozyx.getAngularVelocity_dps(angular_velocity_dps)
	print("Angular velocity: ", angular_velocity_dps.x, ", ", angular_velocity_dps.y, ", ", angular_velocity_dps.z)

def printEulerAngles():
	euler_angles_deg = EulerAngles()
	pozyx.getEulerAngles_deg(euler_angles_deg)
	print("Euler angles: ", euler_angles_deg.heading, ", ", euler_angles_deg.roll, ", ", euler_angles_deg.pitch)

def printGravityVector():
	gravity_vector_mg = Acceleration()
	pozyx.getGravityVector_mg(gravity_vector_mg)
	print("Gravity vector: ", gravity_vector_mg.x, ", ", gravity_vector_mg.y, ", ", gravity_vector_mg.z)

def printLinearAcceleration():
	linear_acceleration_mg = LinearAcceleration()
	pozyx.getLinearAcceleration_mg(linear_acceleration_mg)
	print("Linear acceleration: ", linear_acceleration_mg.x, ", ", linear_acceleration_mg.y, ", ", linear_acceleration_mg.z)

def printMagnetic():
	magnetic_uT = Magnetic()
	pozyx.getMagnetic_uT(magnetic_uT)
	print("Magnetic: ", magnetic_uT.x, ", ", magnetic_uT.y, ", ", magnetic_uT.z)

def printMaxLinearAcceleration():
	max_linear_acceleration_mg = MaxLinearAcceleration()
	pozyx.getMaxLinearAcceleration_mg(max_linear_acceleration_mg)
	print("Max linear acceleration: ", max_linear_acceleration_mg.value)

def printNormalizedQuaternion():
	normalized_quaternion = Quaternion()
	pozyx.getNormalizedQuaternion(normalized_quaternion)
	print("Normalized quaternion: ", normalized_quaternion.x, ", ", normalized_quaternion.y, ", ", normalized_quaternion.z, ", ", normalized_quaternion.w)

def printPressure():
	pressure_pa = Pressure()
	pozyx.getPressure_Pa(pressure_pa)
	print("Pressure: ", pressure_pa.value)

def printQuaternion():
	quaternion = Quaternion()
	pozyx.getQuaternion(quaternion)
	print("Quaternion: ", quaternion.x, ", ", quaternion.y, ", ", quaternion.z, ", ", quaternion.w)

def printTemperature():
	temperature_c = Temperature()
	pozyx.getTemperature_c(temperature_c)
	print("Temperature: ", temperature_c.value)


while True:
	printCoordinates()
	printAcceleration()
	printAngularVelocity()
	printEulerAngles()
	printGravityVector()
	printLinearAcceleration()
	printMagnetic()
	printMaxLinearAcceleration()
	printNormalizedQuaternion()
	printPressure()
	printQuaternion()
	printTemperature()
	print()
	sleep(0.1)
