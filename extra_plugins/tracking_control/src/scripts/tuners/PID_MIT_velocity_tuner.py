#!/usr/bin/env python
'''
PID_MIT_velocity_tuner
This code implements the MIT rule in adaptive PID control.
Applies adaptive PID with MIT rule on velocity.
Reference paper:
Author : Het Shah
'''
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread
from prius_msgs.msg import Control

# Node name       - controls
# Published topic  - pid_output, plot
# Subscribed topic - base_pose_ground_truth, cmd_vel 

gear_stat = "F"
tar_vel = 0 # target velocity
tar_omega = 0 # target omega
active_vel = 0
error_sum = 0
prev_error = 0
error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0 #radius of path
steering_angle = 0 # steering angle

kp = 8.0 # proportional gain
ki = 2.0 # integral gain
kd = 0.2 # derivative gain

yp = 20.0 # kp gain
yi = 0.5 # ki gain
yd = 0.1 # kd gain

acc_thershold = 0 # threshold for acceleration
brake_threshold = 20 # threshold for braking
global pub
global tar_vel
global tar_omega
tar_vel = 0
tar_omega = 0

def convert(v, omega, wheelbase):
	'''
	convert omega to angle in degrees
	: params v [float]
	: params omega [float]
	: params wheelbase [float]
	returns angle in degrees
	'''
	if omega == 0 or v == 0:
		return 0

	radius = v / omega
	# checking sign of the radius of path
	if omega > 0 and v > 0:
		radius = -abs(radius)
	if omega > 0 and v < 0:
		radius = abs(radius)
	if omega < 0 and v > 0:
		radius = abs(radius)
	if omega < 0 and v < 0:
		radius = -abs(radius)

	return math.atan(wheelbase / radius) * 180 / 3.14


def prius_pub(data):
	'''
	publishes the velocity and steering angle
	published on topic : ackermann_cmd_topic
	'''
	global prius_vel
	prius_vel = Control()

	if(data.linear.x > 0):
		prius_vel.throttle = data.linear.x / 100
		prius_vel.brake = 0
		print ("acc")
		print (prius_vel.throttle)

	if(data.linear.x < 0):
		prius_vel.brake = -data.linear.x / 100
		prius_vel.throttle = 0
		print ("brake")
		print (prius_vel.brake)

	prius_vel.steer = data.angular.z / 30

	pub.publish(prius_vel)   

def callback_feedback(data):
	'''
	applies adaptive PID control wiht MIT rule to the velocity
	:params data [Twist]
	:params output [Twist]
	:params plot [Twist]  
	'''
	global active_vel
	global tar_vel
	global tar_omega
	global wheelbase
	global error_sum
	global error
	global error_diff
	global output
	global i
	global flag
	global kp
	global ki
	global kd
	global pub
	global prev_error
	global gear_stat
	global acc_thershold
	global brake_threshold
	global act_velocity
	global yp
	global yi
	global yd
	# convert quarternion to euler
	siny = +2.0 * (data.pose.pose.orientation.w *
				   data.pose.pose.orientation.z +
				   data.pose.pose.orientation.x *
				   data.pose.pose.orientation.y)
	cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
						 data.pose.pose.orientation.y +
						 data.pose.pose.orientation.z *
						 data.pose.pose.orientation.z)
	yaw = math.atan2(siny, cosy)

	last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
						 data.twist.twist.linear.y * math.sin(yaw))

	active_vel = last_recorded_vel
  
	plot = Twist()
	output = Twist()
	# apply PID to velocity
	error = tar_vel - active_vel
	error_sum += error
	error_diff = error - prev_error
	prev_error = error
	if error == 0:
		if tar_vel == 0:
			output.linear.x = 0
		else:
			output.linear.x = output.linear.x - 5
	# Updating kp, ki, kd using MIT rule
	kp = kp + yp * error * error
	ki = ki + yi * error * error_sum
	kd = kd + yd * error * error_diff
	print kp
	print ki
	print kd

	if error > 0.01:
		output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
	if error < -0.01:
		output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
						   brake_threshold)

	plot.linear.x = tar_vel
	plot.linear.y = active_vel
	plot.linear.z = tar_vel - active_vel  # error term
	# thresholding velocity between -100 and 100
	if output.linear.x > 100:
		output.linear.x = 100
	if output.linear.x < -100:
		output.linear.x = -100
	# thresholding angle 
	output.angular.z = min(
		80, max(-80, convert(tar_vel, tar_omega, wheelbase)))

	rospy.loginfo("linear velocity : %f",output.linear.y)
	rospy.loginfo("target linear velocity : %f",output.linear.x)
	rospy.loginfo("delta : %f",output.angular.z)
	prius_pub(output)
	pub1.publish(plot)


def callback_cmd_vel(data):
	"""
	Assigns the value of velocity and omega from topic cmd_vel to tar_vel 
	and tar_omega

	:param tar_vel: (float) target velocity
	:param tar_omega: (float) target omega
	:param data: (twist) 
	"""
	global tar_vel
	global tar_omega
	tar_vel = data.linear.x
	tar_omega = data.angular.z

def start():
	global pub
	global pub1
	ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
	rospy.init_node('controls', anonymous=True)
	pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
	pub1 = rospy.Publisher('plot', Twist, queue_size=10)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)

	rospy.spin()


if __name__ == '__main__':
	start()
