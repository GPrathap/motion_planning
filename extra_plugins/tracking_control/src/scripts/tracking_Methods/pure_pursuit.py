#!/usr/bin/env python
'''
This is the implementation of pure pursuit controller for path tracking
Link to reference paper: https://www.ri.cmu.edu/pub_files/2009/2
/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

Authors: Adarsh Patnaik, Anand Jhunjhunwala

'''

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from prius_msgs.msg import Control

# node name: path_tracking
# Publish Topic: cmd_delta
# Subscribe Topic: base_pose_ground_truth, astroid_path

max_vel = 6.0 # maximum linear velocity
global steer
k = 1.0 # constant for relating look ahead distance and velocity
wheelbase = 1.983 # wheel base for the vehicle
d_lookahead = 0.1 # look ahead distance to calculate target point on path
global n
global ep_max
global ep_sum
global ep_avg
global q
global x_p

q=0
n=0
ep_avg = 0
ep_sum = 0
ep_max = 0

def callback_feedback(data):
	'''
	Assigns the position of the robot to global variables from odometry and
	calculates target path point and the steering angle
	:param x_bot [float]
	:param y_bot [float]
	:param yaw [float]
	:param vel [float]
	:param data [Path]
	:param ep [float]
	:param cp [int]
	'''
	global x_bot
	global y_bot
	global yaw
	global vel
	global ep # min distance
	global cp # index of closest point
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global x_p

	x_bot = data.pose.pose.position.x
	y_bot = data.pose.pose.position.y
	# quarternion to euler conversion
	siny = +2.0 * (data.pose.pose.orientation.w *
				   data.pose.pose.orientation.z +
				   data.pose.pose.orientation.x *
				   data.pose.pose.orientation.y)
	cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
						 data.pose.pose.orientation.y +
						 data.pose.pose.orientation.z *
						 data.pose.pose.orientation.z)
	yaw = math.atan2(siny, cosy) # yaw in radians

	# printing the odometry readings

	cross_err = Twist()
	calc_path_length(x_p)
	data1 = x_p

	distances = []
	for i in range(len(x_p.poses)):
		a = x_p.poses[i]
		distances += [dist(a, x_bot, y_bot)]
	ep = min(distances)
	ep1 = ep

	if (ep > ep_max):
		ep_max = ep

	n = n + 1
	ep_sum = ep_sum + ep
	ep_avg = ep_sum / n


	cp = distances.index(ep)
	cp1 = cp
	cross2 = [(x_bot - data1.poses[cp1].pose.position.x),
			  (y_bot - data1.poses[cp1].pose.position.y)]
	cross = [math.cos(yaw), math.sin(yaw)]
	cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
	if (cross_prod > 0):
		ep1 = -ep1

	cross_err.linear.x = ep1
	cross_err.angular.x = ep_max
	cross_err.angular.y = ep_avg

	# calculate index of target point on path
	cmd = Twist()
	cmd1 = Twist()
	prius_vel = Control()
	L = 0
	Lf = k * max_vel + d_lookahead

	while Lf > L and (cp + 1) < len(x_p.poses):
		dx = data1.poses[cp + 1].pose.position.x - \
			data1.poses[cp].pose.position.x
		dy = data1.poses[cp + 1].pose.position.y - \
			data1.poses[cp].pose.position.y
		L += math.sqrt(dx ** 2 + dy ** 2)
		cp = cp + 1

	goal_point = [x_p.poses[cp].pose.position.x,
				  x_p.poses[cp].pose.position.y]
	error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
	steer_angle = pure_pursuit(goal_point)

	siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
				   x_p.poses[cp].pose.orientation.z +
				   x_p.poses[cp].pose.orientation.x *
				   x_p.poses[cp].pose.orientation.y)

	cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
						 x_p.poses[cp].pose.orientation.y +
						 x_p.poses[cp].pose.orientation.z *
						 x_p.poses[cp].pose.orientation.z)

	steer_path = math.atan2(siny, cosy)
	steer_err = (yaw - steer_path)
	cross_err.linear.y =  (-1)*(yaw - steer_path)


	cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
	cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2) / 10
	cross_err.linear.z = path_length[cp]

	pub1.publish(cmd)
	pub2.publish(cross_err)


def dist(a, x, y):
	'''
	Calculates distance between two points.
	:param a [float]
	:param x [float]
	:param y [float]
	'''
	# calculate distance
	return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def path_length_distance(a,b):
	return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5

def calc_path_length(data):
	global path_length
	path_length = []

	for i in range(len(data.poses)):
		if i == 0:
			path_length.append(0)

		else:
			path_length.append(path_length[i-1] + path_length_distance(data.poses[i], data.poses[i-1]))


def callback_path(data):

	global ep # min distance
	global cp # index of closest point
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global x_p

	x_p = data


def pure_pursuit(goal_point):
	'''
	Calculates the steering angle required for path tracking
	:params goal_point [float,float] goal point coordinates
	:params Delta [float] steering angle in radians 
	'''
	tx = goal_point[0]
	ty = goal_point[1]
	# measuring heading angle 
	alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
	Lf = k * max_vel + d_lookahead
	# measuring the steering angle using pure pursuit controller
	Delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)
	return Delta


def start():
	global pub1
	global pub2
	rospy.init_node('path_tracking', anonymous=True)
	pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
	pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=100)
	rospy.Subscriber("frenet_path", Path, callback_path)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
	rospy.spin()


if __name__ == '__main__':
	start()
