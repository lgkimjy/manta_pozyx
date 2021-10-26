#!/usr/bin/env python3
import rospy
from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, Acceleration, AngularVelocity, EulerAngles, LinearAcceleration, MaxLinearAcceleration, Quaternion)
from geometry_msgs.msg import Accel
from geometry_msgs.msg import QuaternionStamped

pub_acc = rospy.Publisher('/accel', Accel, queue_size=10)
pub_quat = rospy.Publisher('/quaternion', QuaternionStamped, queue_size=10)
pub_euler = rospy.Publisher('/euler_ang', QuaternionStamped, queue_size=10)

def returnAngularVelocity():
	angular_velocity_dps = AngularVelocity()
	pozyx.getAngularVelocity_dps(angular_velocity_dps)
	# print("Angular velocity: ", angular_velocity_dps.x, ", ", angular_velocity_dps.y, ", ", angular_velocity_dps.z)
	return angular_velocity_dps.x, angular_velocity_dps.y, angular_velocity_dps.z

def returnEulerAngles():
	euler_angles_deg = EulerAngles()
	pozyx.getEulerAngles_deg(euler_angles_deg)
	# print("Euler angles: ", euler_angles_deg.heading, ", ", euler_angles_deg.roll, ", ", euler_angles_deg.pitch)
	return euler_angles_deg.roll, euler_angles_deg.pitch, euler_angles_deg.heading

def returnLinearAcceleration():
	linear_acceleration_mg = LinearAcceleration()
	pozyx.getLinearAcceleration_mg(linear_acceleration_mg)
	# print("Linear acceleration: ", linear_acceleration_mg.x, ", ", linear_acceleration_mg.y, ", ", linear_acceleration_mg.z)
	return linear_acceleration_mg.x, linear_acceleration_mg.y, linear_acceleration_mg.z

def returnQuaternion():
	quaternion = Quaternion()
	pozyx.getQuaternion(quaternion)
	#print("Quaternion: ", quaternion.x, ", ", quaternion.y, ", ", quaternion.z, ", ", quaternion.w)
	return quaternion.x, quaternion.y, quaternion.z, quaternion.w

def timer_callback(event):
	# acceleration
	acc_msg = Accel()
	acc_msg.linear.x, acc_msg.linear.y, acc_msg.linear.z = returnLinearAcceleration()
	acc_msg.angular.x, acc_msg.angular.y, acc_msg.angular.z = returnAngularVelocity()
	# quaternion
	quat_msg = QuaternionStamped()
	quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z, quat_msg.quaternion.w = returnQuaternion()
	# euler
	euler_msg = QuaternionStamped()
	euler_msg.quaternion.x, euler_msg.quaternion.y, euler_msg.quaternion.z = returnEulerAngles()  ## roll, pitch, yaw
	# publisher
	pub_acc.publish(acc_msg)
	pub_quat.publish(quat_msg)
	pub_euler.publish(euler_msg)

if __name__ == '__main__':
	global pozyx
	serial_port = get_first_pozyx_serial_port()

	if serial_port is not None:
		pozyx = PozyxSerial(serial_port)
		rospy.loginfo("Connection success!")
	else:
		rospy.logerr("No Pozyx port was found")
		exit(1)

	rospy.init_node('sensor_output', anonymous=True)
	rospy.Timer(rospy.Duration(0.01), timer_callback)

	while not rospy.is_shutdown():
		rospy.sleep(0.1)