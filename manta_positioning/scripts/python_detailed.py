#!/usr/bin/env python

import rospy
from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, Coordinates, Acceleration, AngularVelocity, EulerAngles, LinearAcceleration, Magnetic, MaxLinearAcceleration, Pressure, Quaternion, Temperature)
from time import sleep
from mqtt2ros.msg import mqtt_msg
from geometry_msgs.msg import PoseStamped

# pozyx.setHeight(1000) # For 2D and 2.5D positioning

def printCoordinates():
	coordinates = Coordinates()
	pozyx.getCoordinates(coordinates)
	#print("Coordinates: ", coordinates.x, ", ", coordinates.y, ", ", coordinates.z)
	return coordinates.x, coordinates.y, coordinates.z 

def mqtt_2_ros():
	global pozyx
	#initialize serail port
	serial_port = get_first_pozyx_serial_port()

	if serial_port is not None:
		pozyx = PozyxSerial(serial_port)
		print("Connection success!")
	else:
		print("No Pozyx port was found")
		exit(1)
	
	#ros
	pub = rospy.Publisher('/coord', mqtt_msg, queue_size=10)
	rospy.init_node('python_detailed', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		msg = mqtt_msg()
		pose = PoseStamped()

		pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = printCoordinates()
		msg.data.append(pose)
		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		mqtt_2_ros()
	except rospy.ROSInterruptException:
		pass