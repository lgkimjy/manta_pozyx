#!/usr/bin/env python3
import rospy
from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, Acceleration, AngularVelocity, EulerAngles, LinearAcceleration, MaxLinearAcceleration, Quaternion)
from pypozyx.structures.device_information import DeviceDetails
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry

pub_acc = rospy.Publisher('/accel', AccelStamped, queue_size=10)
pub_euler = rospy.Publisher('/euler_ang', QuaternionStamped, queue_size=10)
pub_lpf_acc = rospy.Publisher('/lpf/accel', AccelStamped, queue_size=10)
pub_pose = rospy.Publisher('/imu_odom', Odometry, queue_size=10)

pre_linear_x = pre_linear_y = pre_linear_z = 0
old_vx = old_vy = old_vz = 0
odom_msg = Odometry()
dt = 0.01

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

def printAcceleration():
	acceleration_mg = Acceleration()
	pozyx.getAcceleration_mg(acceleration_mg)
	# print("Acceleration: ", acceleration_mg.x, ", ", acceleration_mg.y, ", ", acceleration_mg.z)
	return acceleration_mg.x, acceleration_mg.y, acceleration_mg.z

def returnLinearAcceleration():
	linear_acceleration_mg = LinearAcceleration()
	pozyx.getLinearAcceleration_mg(linear_acceleration_mg)
	# print("Linear acceleration: ", linear_acceleration_mg.x, ", ", linear_acceleration_mg.y, ", ", linear_acceleration_mg.z)
	return linear_acceleration_mg.x, linear_acceleration_mg.y, linear_acceleration_mg.z

def LowPassFilter(raw_data, prev_data, alpha):
	output = raw_data * alpha + (1 -alpha) * prev_data
	return output

def timer_callback(event):
	global pre_linear_x, pre_linear_y, pre_linear_z, old_vx, old_vy, old_vz
	ros_time = rospy.get_rostime()
	# acceleration
	acc_msg = AccelStamped()
	acc_msg.header.stamp = ros_time
	acc_msg.header.frame_id = str(system_details.id)
	acc_msg.accel.linear.x, acc_msg.accel.linear.y, acc_msg.accel.linear.z = returnLinearAcceleration()
	acc_msg.accel.angular.x, acc_msg.accel.angular.y, acc_msg.accel.angular.z = returnAngularVelocity()

	# euler
	euler_msg = QuaternionStamped()
	euler_msg.header.stamp = ros_time
	euler_msg.header.frame_id = str(system_details.id)
	euler_msg.quaternion.x, euler_msg.quaternion.y, euler_msg.quaternion.z = returnEulerAngles()  ## roll, pitch, yaw

	# lpf acc
	acc_lpf_msg = AccelStamped()
	pre_linear_x = LowPassFilter(acc_msg.accel.linear.x, pre_linear_x, 0.01)
	pre_linear_y = LowPassFilter(acc_msg.accel.linear.y, pre_linear_y, 0.01)
	pre_linear_z = LowPassFilter(acc_msg.accel.linear.z, pre_linear_z, 0.01)
	acc_lpf_msg.accel.linear.x = old_vx + pre_linear_x * dt
	acc_lpf_msg.accel.linear.y = old_vy + pre_linear_y * dt
	acc_lpf_msg.accel.linear.z = old_vz + pre_linear_z * dt
	old_vx = pre_linear_x * dt
	old_vy = pre_linear_y * dt
	old_vz = pre_linear_z * dt

	# odom_msg.header.frame_id = str(system_details.id)
	# odom_msg.pose.pose.position.x = odom_msg.pose.pose.position.x + (pre_linear_x * 0.001 * 0.003)
	# odom_msg.pose.pose.position.y = odom_msg.pose.pose.position.y + (pre_linear_y * 0.003 * 0.003)
	# odom_msg.pose.pose.position.z = odom_msg.pose.pose.position.z + (pre_linear_z * 0.003 * 0.003)

	# publisher
	pub_acc.publish(acc_msg)
	pub_euler.publish(euler_msg)
	pub_lpf_acc.publish(acc_lpf_msg)
	# pub_pose.publish(odom_msg)

if __name__ == '__main__':
	global pozyx
	serial_port = get_first_pozyx_serial_port()

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
	rospy.Timer(rospy.Duration(dt), timer_callback)

	# LPF data initalization
	pre_linear_x, pre_linear_y, pre_linear_z = returnLinearAcceleration()

	while not rospy.is_shutdown():
		rospy.sleep(0.1)