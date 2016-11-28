#!/usr/bin/env python
# -*- coding: utf-8 -*-

#convert LCM logs to Rosbags
import lcm
import rosbag

#stuff for image conversion (mjpeg to numpy array)
import cv2
import numpy as np

#the ROS message types for /cam0/image_raw and /fcu/imu
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge

#get the LCM types for ins and image
from sensors import ins_t, image_t

#get the LCM type for vicon
from body import rigid_transform_t


class BagfileMaker(object):
  def __init__(self, full_bag_path):
    self.bag = rosbag.Bag(full_bag_path, 'w')
    self.minTime = 0
    self.maxTime = 0
    self.seq = 0
    self.bridge = CvBridge()

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
  	imu_msg.header.seq = self.seq
  	imu_msg.header.stamp.secs, imu_msg.header.stamp.nsecs = getSecsNSecs(lcm_msg.utime)

  	if self.minTime == 0:
  		self.minTime = imu_msg.header.stamp.secs
  	elif imu_msg.header.stamp.secs > self.maxTime:
  		self.maxTime = imu_msg.header.stamp.secs

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

  	# increment the seq for the next bag msg
  	self.seq += 1

  	#write the msg to bag
  	self.bag.write(imu_topic_name, imu_msg, imu_msg.header.stamp)

  def cameraHandler(self, event, cam_topic_name):
  	"""
  	Convert LCM camera message to a ROS sensor_msgs/Image message and write to bag.
  	"""
  	#decode the event to get formatted camera data
  	lcm_msg = image_t.decode(event.data)

  	# decode the jpeg source images into opencv arrays
  	src_img = np.fromstring(lcm_msg.data, dtype='uint8')
  	raw_image = cv2.imdecode(src_img, cv2.CV_LOAD_IMAGE_GRAYSCALE)
  	img_msg = self.bridge.cv2_to_imgmsg(raw_image, encoding="mono8")

  	#make a new rosbag image msg
  	img_msg.header.seq = self.seq
  	img_msg.header.stamp.secs, img_msg.header.stamp.nsecs = getSecsNSecs(lcm_msg.utime)
  	img_msg.header.frame_id = 'cam'

  	# increment the seq for the next message
  	self.seq += 1

  	#write the msg to bag
  	self.bag.write(cam_topic_name, img_msg, img_msg.header.stamp)

  def viconHandler(self, event, vicon_topic_name):
  	"""
  	Convert LCM vicon message into a ROS geometry_msgs/TransformStamped msg
  	"""
  	lcm_msg = rigid_transform_t.decode(event.data)
  	vicon_msg = TransformStamped()

  	#build the header
  	vicon_msg.header.seq = self.seq
  	vicon_msg.header.stamp.secs, vicon_msg.header.stamp.nsecs = getSecsNSecs(lcm_msg.utime)
  	vicon_msg.header.frame_id = 'vicon'

  	vicon_msg.transform.translation.x = lcm_msg.trans[0]
  	vicon_msg.transform.translation.y = lcm_msg.trans[1]
  	vicon_msg.transform.translation.z = lcm_msg.trans[2]

  	vicon_msg.transform.rotation.w = lcm_msg.quat[0]
  	vicon_msg.transform.rotation.x = lcm_msg.quat[1]
  	vicon_msg.transform.rotation.y = lcm_msg.quat[2]
  	vicon_msg.transform.rotation.z = lcm_msg.quat[3]

  	# increment the seq for the next message
  	self.seq += 1
  	self.bag.write(vicon_topic_name, vicon_msg, vicon_msg.header.stamp)

  def closeBag(self):
  	print("Finished writing", self.seq, "messages to rosbag file.")
  	print("Rosbag duration:", self.maxTime-self.minTime, "secs.")
  	self.bag.close()


def getSecsNSecs(utime):
	"""
	Takes a utime value (microsecs since epoch) given by LCM
	Returns (secs, nsecs)
	secs: seconds since epoch
	nsecs: nanosecs since last sec
	"""
	secs = int(utime // 1e6)
	nsecs = int((float(utime) / 1e6 - secs) *1e9)
	#print(secs, nsecs)
	return (secs, nsecs)


def main():

	#define the path to the lcm log
	lcm_file_dir = '/home/mknowles/Datasets/STAR/run/'
	bagfile_dir = '/home/mknowles/bagfiles/star/'

	#NOTE: change the names here!!!!!
	lcm_file_name = 'lcmlog-2015-08-18.02' #18.00 through 18.18
	bagfile_name = 'star2.bag'

	#define rosbag output topic names
	cam_topic_name = '/cam0/image_raw'
	imu_topic_name = '/imu0'
	vicon_topic_name = 'vicon/tf'


	#define LCM input channel names
	ins_channel = 'MICROSTRAIN_INS'
	img_channel = 'CAMLCM_IMAGE'
	vicon_channel = 'VICON_star_pose_2'

	#make the bagfile maker
	bfm = BagfileMaker(bagfile_dir + bagfile_name)

	#make the LCM Event log to retrieve events from
	LCMLog = lcm.EventLog(lcm_file_dir+lcm_file_name, 'r', overwrite=False)



	#translate LCM events into ros msgs using the appropriate handler functions
	for event in LCMLog:
		print(bfm.seq)
		#check which type of LCM message we have and decode accordingly
		if event.channel == img_channel:
			#print("Image channel")
			bfm.cameraHandler(event, cam_topic_name)

		elif event.channel == ins_channel:
			#print("Imu channel")
			bfm.imuHandler(event, imu_topic_name)

		elif event.channel == vicon_channel:
			#print("Vicon channel")
			bfm.viconHandler(event, vicon_topic_name)
		else:
			#print("Event from unknown channel.")
			pass

	#print("Min:", bfm.minTime, "Max:", bfm.maxTime)
	#close the LCMLog
	LCMLog.close()

	#close the bagfile
	bfm.closeBag()


if __name__ == '__main__':
	main()


"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w
"""


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