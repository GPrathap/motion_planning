#!/usr/bin/env python
'''
This is the application of PID on steering angle for path tracking
Authors: Adarsh Patnaik
'''

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# node name: PID_steering
# Publish Topic: cmd_delta, cross_err
# Subscribe Topic: base_pose_ground_truth, astroid_path

global n
global ep_max
global ep_sum
global ep_avg
global path_data
wheelbase = 1.983 
kp = 0.1 		#proportional constant
ki = 0.001		#integral constant
kd = 0.001		#derivative constant
n=0
ep_avg = 0
ep_sum = 0
ep_max = 0


def callback_feedback(data):
	"""
	Gets the value of bot position and yaw,
	calculates crosstrack error and gives the pid controlled steering angle.
	:param cross_track_err: crosstrack error
	:param delta : steering angle 
	:param steering correction : difference of path tangent and bot yaw
	:param x_bot : current x coordinate of bot 
	:param y_bot : current y coordinate of bot
	:param: bot_yaw : current yaw of bot
	"""
	global x_bot
	global y_bot
	global bot_yaw
	global path_data

	x_bot = data.pose.pose.position.x
	y_bot = data.pose.pose.position.y

	siny = 2.0 * (data.pose.pose.orientation.w *
				  data.pose.pose.orientation.z +
				  data.pose.pose.orientation.x *
				  data.pose.pose.orientation.y)
	cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
						data.pose.pose.orientation.y +
						data.pose.pose.orientation.z *
						data.pose.pose.orientation.z)
	bot_yaw = math.atan2(siny, cosy)

	global cross_track_err
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length

	prev_cte = 0
	diff_cte = 0
	sum_cte = 0
	delta = Twist()
	cross_err = Twist()

	# path_data = data
	calc_path_length(path_data)
	data1=path_data

	distances = []
	for i in range(len(path_data.poses)):
		a = path_data.poses[i]
		distances += [dist(a, x_bot, y_bot)]
	distance_min = min(distances)
	index = distances.index(distance_min)

	if (distance_min > ep_max):
		ep_max = distance_min
	n = n + 1
	ep_sum = ep_sum + distance_min
	ep_avg = ep_sum / n

	# deciding the sign of crosstrack error
	orientation_vec = [(x_bot - data1.poses[index].pose.position.x), (y_bot - data1.poses[index].pose.position.y)]
	heading_vec = [math.cos(bot_yaw), math.sin(bot_yaw)]
	cross_prod = heading_vec[0] * orientation_vec[1] - heading_vec[1] * orientation_vec[0]

	if (cross_prod > 0):
		distance_min = -distance_min 

	cross_track_err = distance_min   
	cross_err.linear.x = cross_track_err
	cross_err.angular.x = ep_max
	cross_err.angular.y = ep_avg

	
	siny = +2.0 * (path_data.poses[index].pose.orientation.w *
				   path_data.poses[index].pose.orientation.z +
				   path_data.poses[index].pose.orientation.x *
				   path_data.poses[index].pose.orientation.y)
	cosy = +1.0 - 2.0 * (path_data.poses[index].pose.orientation.y *
						 path_data.poses[index].pose.orientation.y +
						 path_data.poses[index].pose.orientation.z *
						 path_data.poses[index].pose.orientation.z)

	path_yaw = math.atan2(path_data.poses[index].pose.position.y - path_data.poses[index-1].pose.position.y, path_data.poses[index].pose.position.x - path_data.poses[index-1].pose.position.x )

	steering_correction = (path_yaw - bot_yaw) * (180 / math.pi)
	# print ("steering_correction = ", steering_correction)
	
	# PID controlled steering angle calculation
	diff_cte = cross_track_err - prev_cte
	sum_cte += cross_track_err
	steering_angle = (kp * cross_track_err + kd * diff_cte + ki * sum_cte)
	steering_angle = steering_angle * (180 / math.pi)
	prev_cte = cross_track_err
	cross_err.linear.y = steering_correction
	cross_err.linear.z = path_length[index]
	delta.angular.z = min(30, max(-30, steering_correction + steering_angle))
	pub.publish(delta)
	pub2.publish(cross_err)

def dist(a,x,y):
	"""
	function to calculate distance
	"""
	return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def path_length_distance(a,b):
	return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5

def calc_path_length(data):
	global path_length
	global path_data
	path_length = []

	for i in range(len(data.poses)):
		if i == 0:
			path_length.append(0)
		else:
			path_length.append(path_length[i-1] + path_length_distance(data.poses[i], data.poses[i-1]))


def callback_path(data):
	
	global x_bot
	global y_bot
	global bot_yaw
	global cross_track_err
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global path_data

	path_data = data


def start():

	global pub
	global pub2
	rospy.init_node("PID_steering", anonymous = True)
	pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
	pub = rospy.Publisher("cmd_delta", Twist, queue_size = 100)
	rospy.Subscriber("astroid_path", Path, callback_path)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
	rospy.spin()

if __name__ == '__main__':
	start()