#!/usr/bin/env python
'''
AdpPID_velocity_tuner
This code is using adaptive PID for tuning the velocity
Link to reference paper- http://oa.upm.es/30015/1/INVE_MEM_2013_165545.pdf 
Authors - Manthan Patel, Sombit Dey
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
# Publish topic  - pid_output (Twist) , plot(Twist)
# Subscribe topic - base_pose_ground_truth , cmd_vel (Twist)

global pub
global tar_vel
global tar_omega
gear_stat = "F"
active_vel = 0
error_sum = 0
prev_error = 0
error_diff = 0
filtered_error = 0
prev_filtered_error = 0
filtered_error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0
steering_angle = 0
kp = 5  # proportionality gain
ki = 0.02  # integral gain
kd = 0.05  # derivative gain
yp = 0.8  # kp gain
yi = 0.02  #Ki gain
yd = 0.5  #Kd gain 
y = 0.8  #low pass filter constant
acc_thershold = 0
brake_threshold = 20
tar_vel = 0
tar_omega = 0


def convert(v, omega, wheelbase):
	"""
	converts the input angular velocity into steering angle 

	:param v: (float) target velocity
	:param omega: (float) target omega
	:param wheelbase: (float) wheelbase of car
	:return: (float) steering angle
	"""
	if omega == 0 or v == 0:
		return 0
	radius = v / omega

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
	"""
	Uses the adaptive PID formula and publish the velocity and delta
	on topic pid_output

	:param yaw: (float)
	"""
	global active_vel
	global tar_vel
	global tar_omega
	global wheelbase
	global error_sum
	global error
	global error_diff
	global filtered_error_diff
	global filtered_error
	global prev_filtered_error
	global y
	global output
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
	plot = Twist()
	output = Twist()

	siny = 2.0 * (data.pose.pose.orientation.w *
				  data.pose.pose.orientation.z +
				  data.pose.pose.orientation.x *
				  data.pose.pose.orientation.y)

	cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
						data.pose.pose.orientation.y +
						data.pose.pose.orientation.z *
						data.pose.pose.orientation.z)

	yaw = math.atan2(siny, cosy)

	last_recorded_vel = data.twist.twist.linear.x * math.cos(yaw) + data.twist.twist.linear.y * math.sin(yaw)

	active_vel = last_recorded_vel

	error = tar_vel - active_vel
	error_sum += error
	error_diff = error - prev_error

	filtered_error = error * (1 - y) + prev_error * y  #Low Pass Filter
	filtered_error_diff = filtered_error - prev_filtered_error
	prev_filtered_error = filtered_error

	if error == 0:

		if tar_vel == 0:
			output.linear.x = 0
		else:
			output.linear.x = output.linear.x - 5

	#Tuning the values of PID constants        
	kp = kp + yp * (error - filtered_error)
	ki = ki + yi * filtered_error
	kd = kd + yd * (error_diff - filtered_error_diff)

	#Limiting the values of kp, ki, kd to a certain range
	if(kp > 25):
		kp = 25

	if(kp < 3):
		kp = 3

	if(ki > 10):
		ki = 10

	if(ki < 0):
		ki = 0

	if(kd > 10):
		kd = 10

	if(kd < 0):
		kd = 0

	if error > 0.01:
		output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
	if error < -0.01:
		output.linear.x = ((kp * error + ki * error_sum + kd * error_diff)
													   - brake_threshold)

	plot.linear.x = tar_vel  #target velocity
	plot.linear.y = active_vel  #current velocity
	plot.linear.z = tar_vel - active_vel  # error term
	plot.angular.x = filtered_error

	#Limiting the values of target velocity
	if output.linear.x > 100:
		output.linear.x = 100
	if output.linear.x < -100:
		output.linear.x = -100

	#Limiting the steering angle between [-30,30]
	output.angular.z = min(
		30, max(-30, convert(tar_vel, tar_omega, wheelbase)))

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
