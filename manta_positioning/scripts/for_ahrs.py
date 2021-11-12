#!/usr/bin/env python3
import rospy
from pypozyx import (get_first_pozyx_serial_port, PozyxSerial, AngularVelocity, EulerAngles, LinearAcceleration, Quaternion, Magnetic)
from pypozyx.structures.device_information import DeviceDetails
from geometry_msgs.msg import AccelStamped, QuaternionStamped
from sensor_msgs.msg import Imu, MagneticField

pub_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
pub_mag = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

## w IMU data, including angular velocities and linear accelerations.

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

def returnMagnetic():
	magnetic_uT = Magnetic()
	pozyx.getMagnetic_uT(magnetic_uT)
	# print("Magnetic: ", magnetic_uT.x, ", ", magnetic_uT.y, ", ", magnetic_uT.z)
	return magnetic_uT.x, magnetic_uT.y, magnetic_uT.z

def LowPassFilter(raw_data, prev_data, alpha):
	output = raw_data * alpha + (1 -alpha) * prev_data
	return output

def timer_callback(event):
	global pre_linear_x, pre_linear_y, pre_linear_z
	
	ros_time = rospy.get_rostime()
	# acceleration
	imu_msg = Imu()
	imu_msg.header.stamp = ros_time
	imu_msg.header.frame_id = str(system_details.id)
	imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = returnLinearAcceleration()
	# pre_linear_x = LowPassFilter(imu_msg.linear_acceleration.x, pre_linear_x, 0.9)
	# pre_linear_y = LowPassFilter(imu_msg.linear_acceleration.y, pre_linear_y, 0.9)
	# pre_linear_z = LowPassFilter(imu_msg.linear_acceleration.z, pre_linear_z, 0.9)
	# imu_msg.linear_acceleration.x = pre_linear_x
	# imu_msg.linear_acceleration.y = pre_linear_y
	# imu_msg.linear_acceleration.z = pre_linear_z
	imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = returnAngularVelocity()
	# quaternion
	mag_msg = MagneticField()
	mag_msg.header.stamp = ros_time
	mag_msg.header.frame_id = str(system_details.id)
	mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = returnMagnetic()
	# publisher
	pub_imu.publish(imu_msg)
	pub_mag.publish(mag_msg)
	
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
	rospy.Timer(rospy.Duration(0.01), timer_callback)

	pre_linear_x, pre_linear_y, pre_linear_z = returnLinearAcceleration()

	while not rospy.is_shutdown():
		rospy.sleep(0.1)