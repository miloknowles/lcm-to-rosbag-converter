#!/usr/bin/env python
# -*- coding: utf-8 -*-

#convert LCM logs to Rosbags

import lcm
import rosbag

#the ROS message types for /cam0/image_raw and /fcu/imu
from sensor_msgs.msg import Image, Imu

#get the LCM types for ins and image
from sensors import ins_t, image_t


class BagfileMaker(object):
  def __init__(self, full_bag_path):
    self.bag = rosbag.Bag(full_bag_path, 'w')

  def imuHandler(self, event, imu_topic_name):
  	"""
  	event: the LCM event object
  	msg: the decoded data of the LCM event
  	Convert LCM imu message to a ROS sensor_msgs/Imu message and write to bag.
  	"""
  	#decode the event to get formatted imu data
  	lcm_msg = ins_t.decode(event.data)

  	#fill in the available information of our sensor_msgs/Imu message
  	imu_msg = Imu()
  	imu_msg.header.seq = event.eventnum
  	imu_msg.header.stamp.secs = lcm_msg.utime / 1000000
  	imu_msg.header.stamp.nsecs = #TODO get nanosecs
  	imu_msg.header.frame_id = 'imu'
  	imu_msg.orientation.w = lcm_msg.quat[0]
  	imu_msg.orientation.x = lcm_msg.quat[1]
  	imu_msg.orientation.y = lcm_msg.quat[2]
  	imu_msg.orientation.z = lcm_msg.quat[3]
  	imu_msg.angular_velocity.x = lcm_msg.gyro[0]
  	imu_msg.angular_velocity.y = lcm_msg.gyro[1]
  	imu_msg.angular_velocity.z = lcm_msg.gyro[2]
  	imu_msg.linear_acceleration.x = lcm_msg.accel[0]
  	imu_msg.linear_acceleration.y = lcm_msg.accel[1]
  	imu_msg.linear_acceleration.z = lcm_msg.accel[2]


  	#write the msg to bag
  	self.bag.write(imu_topic_name, imu_msg)

  def cameraHandler(self, event, cam_topic_name):
  	"""
  	Convert LCM camera message to a ROS sensor_msgs/Image message and write to bag.
  	"""
  	#decode the event to get formatted camera data
  	lcm_msg = image_t.decode(event.data)

  	img_msg = Image()


  	#write the msg to bag
  	#self.bag.write(cam_topic_name, img_msg)


  def closeBag(self):
  	self.bag.close()

"""
Image message:

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
"""

"""
Imu message:

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance

"""





def main():

	#define the path to the lcm log
	lcm_file_dir = '/home/mknowles/Datasets/STAR/run/'
	lcm_file_name = 'lcmlog-2015-08-18.00'
	bagfile_dir = '/home/mknowles/bagfiles/star'
	bagfile_name = 'star0.bag'

	#define rosbag output topic names
	cam_topic_name = '/cam0/image_raw'
	imu_topic_name = '/fcu/imu'


	#define LCM input channel names
	ins_channel = 'MICROSTRAIN_INS'
	img_channel = 'CAMLCM_IMAGE'
	vicon_channel = 'VICON_star_pose_2'

	#make the bagfile maker
	bfm = BagfileMaker(bagfile_dir+bagfile_name)

	#make the LCM Event log to retrieve events from
	LCMLog = lcm.EventLog(lcm_file_dir+lcm_file_name, 'r', overwrite=False)



	#translate LCM events into ros msgs using the appropriate handler functions
	for event in LCMLog:

		#check which type of LCM message we have and decode accordingly
		if event.channel == img_channel:

			bfm.cameraHandler(event, cam_topic_name)

		elif event.channel == ins_channel:
			bfm.imuHandler(event, imu_topic_name)

		elif event.channel == vicon_channel:
			pass
		else:
			print "Event from unknown channel."

	#close the LCMLog
	LCMLog.close()

	#close the bagfile
	bfm.closeBag()


if __name__ == '__main__':
	main()