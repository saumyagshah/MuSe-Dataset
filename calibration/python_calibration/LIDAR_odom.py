#! /usr/bin/env python

import rospy

import json

import math
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


data = {}
data['nrays'] = 360
data['min_theta'] = -1 * math.pi + math.pi/180
data['max_theta'] = math.pi
# data['estimate'] = ['null', 'null', 'null']
# data['true_pose'] = ['null', 'null', 'null']

data['theta'] = []
theta = -1 * math.pi + math.pi/180
for i in range(360):
	data['theta'].append(theta)
	theta += math.pi / 180
	
	
def callback_odom(incoming):
	# data['temp_odom'] = {incoming.header.seq, incoming.header.stamp.secs, incoming.header.stamp.nsecs}
	q_w = incoming.pose.pose.orientation.w
	q_x = incoming.pose.pose.orientation.x
	q_y = incoming.pose.pose.orientation.y
	q_z = incoming.pose.pose.orientation.z
	yaw = math.atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)
	data['odometry'] = []
	data['odometry'].append(incoming.pose.pose.position.x)
	data['odometry'].append(incoming.pose.pose.position.y)
	data['odometry'].append(yaw)


def callback_scan(incoming):
	# data['temp_scan'] = {incoming.header.seq, incoming.header.stamp.secs, incoming.header.stamp.nsecs}
	# if(incoming.header.seq == 0):
	# 	base_case_fixer = 1
	# else:
	# 	base_case_fixer = 2
	
	# data['timestamp'] = [incoming.header.stamp.secs, incoming.header.stamp.nsecs]
	data['readings'] = list(incoming.ranges)
	for index, x in enumerate(data['readings']):
		if(str(x) == 'Infinity' or str(x) == 'inf'):
			data['readings'][index] = 20
	data['valid'] = []
	for x in data['readings']:
		if (str(x) == 'inf'):
			data['valid'].append(0)
		else:
			data['valid'].append(1)

	with open('formatted_data.txt', 'a') as output:
		json.dump(data, output)
	# print(data)


	
def listener():
	rospy.init_node('listener', anonymous=True)
	# rospy.Subscriber("/mobile_base/sensors/core", SensorState, callback_kobuki)
	rospy.Subscriber("/odom", Odometry, callback_odom)
	rospy.Subscriber("/scan", LaserScan, callback_scan)
	rospy.spin()

if __name__ == '__main__':
	listener()
