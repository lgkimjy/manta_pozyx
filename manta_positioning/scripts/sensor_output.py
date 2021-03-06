#!/usr/bin/env python3
import rospy
from pypozyx import (get_first_pozyx_serial_port, get_serial_ports, PozyxSerial, Acceleration, AngularVelocity, EulerAngles, LinearAcceleration, MaxLinearAcceleration, Quaternion)
from pypozyx.structures.device_information import DeviceDetails
from geometry_msgs.msg import AccelStamped, QuaternionStamped

pub_acc = rospy.Publisher('/accel', AccelStamped, queue_size=10)
pub_quat = rospy.Publisher('/quaternion', QuaternionStamped, queue_size=10)
pub_euler = rospy.Publisher('/euler_ang', QuaternionStamped, queue_size=10)

def myhook():
  print("shutdown Occured! Change Yaw state")

def returnAngularVelocity():
	angular_velocity_dps = AngularVelocity()
	pozyx.getAngularVelocity_dps(angular_velocity_dps)
	# print("Angular velocity: ", angular_velocity_dps.x, ", ", angular_velocity_dps.y, ", ", angular_velocity_dps.z)
	return angular_velocity_dps.x, angular_velocity_dps.y, angular_velocity_dps.z

def returnEulerAngles():
	euler_angles_deg = EulerAngles()
	pozyx.getEulerAngles_deg(euler_angles_deg)
	# print("Euler angles: ", euler_angles_deg.heading, ", ", euler_angles_deg.roll, ", ", euler_angles_deg.pitch)
	return -1*euler_angles_deg.pitch, -1*euler_angles_deg.roll, 360-euler_angles_deg.heading

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

def LowPassFilter(raw_data, prev_data, alpha):
	output = raw_data * alpha + (1 -alpha) * prev_data
	return output

def HighPassFilter(raw_data, prev_data, prev_filtered_data, alpha):
	output = alpha * (prev_filtered_data + raw_data - prev_data)
	return output, raw_data

def timer_callback(event):
	global pre_linear_x, pre_linear_y, pre_linear_z
	
	ros_time = rospy.get_rostime()
	# acceleration
	acc_msg = AccelStamped()
	acc_msg.header.stamp = ros_time
	acc_msg.header.frame_id = str(system_details.id)
	acc_msg.accel.linear.x, acc_msg.accel.linear.y, acc_msg.accel.linear.z = returnLinearAcceleration()
	pre_linear_x = LowPassFilter(acc_msg.accel.linear.x, pre_linear_x, 0.8)
	pre_linear_y = LowPassFilter(acc_msg.accel.linear.y, pre_linear_y, 0.8)
	pre_linear_z = LowPassFilter(acc_msg.accel.linear.z, pre_linear_z, 0.8)
	acc_msg.accel.linear.x = pre_linear_x
	acc_msg.accel.linear.y = pre_linear_y
	acc_msg.accel.linear.z = pre_linear_z
	acc_msg.accel.angular.x, acc_msg.accel.angular.y, acc_msg.accel.angular.z = returnAngularVelocity()
	# quaternion
	quat_msg = QuaternionStamped()
	quat_msg.header.stamp = ros_time
	quat_msg.header.frame_id = str(system_details.id)
	quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z, quat_msg.quaternion.w = returnQuaternion()
	# euler
	euler_msg = QuaternionStamped()
	euler_msg.header.stamp = ros_time
	euler_msg.header.frame_id = str(system_details.id)
	euler_msg.quaternion.x, euler_msg.quaternion.y, euler_msg.quaternion.z = returnEulerAngles()  ## roll, pitch, yaw
	# publisher
	pub_acc.publish(acc_msg)
	pub_quat.publish(quat_msg)
	pub_euler.publish(euler_msg)

	serial_port = get_first_pozyx_serial_port()
	if serial_port is None:
		print("a")
		rospy.signal_shutdown(myhook)


if __name__ == '__main__':
	global pozyx, serial_port, flag
	serial_port = get_first_pozyx_serial_port()
	print(serial_port)
	if serial_port is not None:
		pozyx = PozyxSerial(serial_port)
		rospy.loginfo("Connection success!")
	else:
		rospy.logerr("No Pozyx port was found")
		exit(1)

	remote_id =None
	system_details = DeviceDetails()
	pozyx.getDeviceDetails(system_details, remote_id=None)

	if remote_id is None:
		print("Local %s with id 0x%d" % (system_details.device_name, system_details.id))
	else:
		print("%s with id 0x%0.4x" % (system_details.device_name.capitalize(), system_details.id))

	rospy.init_node('sensor_output', anonymous=True)
	rospy.Timer(rospy.Duration(0.01), timer_callback)

	pre_linear_x, pre_linear_y, pre_linear_z = returnLinearAcceleration()

	while not rospy.is_shutdown():
		rospy.sleep(0.1)