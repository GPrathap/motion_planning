#!/usr/bin/env python
'''
PID_velocity_controller
This code implements a simple PID controller for path tracking.
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
# Published topic  - pid_output (Twist)
# Subscribed topic - base_pose_ground_truth , cmd_vel, cmd_delta 

gear_stat = "F"
tar_vel = 0 # target velocity
tar_omega = 0 # target omega
tar_delta = 0
active_vel = 0 # current velocity of robot
error_sum = 0
prev_error = 0
error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0 # radius of path
steering_angle = 0 # steering angle in degrees
kp = 1000.0 # proportional gain
ki = 1.5 # integral gain
kd = 1.8 # differential gain
acc_thershold = 0 # threshold for acceleration
brake_threshold = 20 # threshold for brake
global pub
global tar_vel
global tar_omega
global tar_delta
tar_vel = 0
tar_omega = 0

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
	#print "steering:", prius_vel.steer

	pub.publish(prius_vel)


def callback_feedback(data):
	'''
	Applies PID to velcity input from odom readings and publishes.
	:params data [Odometry]
	:params output [Twist]
	:params plot [Twist] 
	'''
	global active_vel
	global tar_vel
	global tar_delta
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
	# conversion of odometry readings from quarternion to euler
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
	# applying PID on the Velocity
	error = tar_vel - active_vel
	error_sum += error
	error_diff = error - prev_error
	prev_error = error
	if error == 0:
		print ("e")
		if tar_vel == 0:
			output.linear.x = 0
		else:
			output.linear.x = output.linear.x - 5

	if error > 0.01:
		print ("e1")
		output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
	if error < -0.01:
		print ("e2")
		output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
						   brake_threshold)

	plot.linear.x = tar_vel
	plot.linear.y = active_vel
	plot.linear.z = tar_vel - active_vel  # error term


	# thresholding the forward velocity within -100 to 100
	if output.linear.x > 100:
		output.linear.x = 100
	if output.linear.x < -100:
		output.linear.x = -100


	# Thresholding the steering angle between 30 degrees and -30 degrees
	output.angular.z = min(30.0, max(-30.0, tar_delta))
	rospy.loginfo("linear velocity : %f",plot.linear.y)
	rospy.loginfo("target linear velocity : %f",plot.linear.x)
	rospy.loginfo("delta : %f",output.angular.z)
	# publish the msg
	#prius_pub(output) for simulator
	pub.publish(output)
	pub1.publish(plot)


def callback_cmd_vel(data):
	'''
	Subscribes from cmd_vel and gives target velocity
	:params data [Twist]
	:params tar_vel [float]
	'''
	global tar_vel
	tar_vel = data.linear.x
	# smoothing the velocity and reducing it at greater turning angles.
	 


def callback_delta(data):
	'''
	Subscribes from cmd_delta and returns target steering angle.
	:params data [Twist]
	:params tar_delta [Float] 
	'''
	global tar_delta
	tar_delta = data.angular.z



def start():
	global pub
	global pub1
rospy.init_node('controls', anonymous=True)
	#ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
	#pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
	pub = rospy.Publisher('pid_output', Twist, queue_size=10)
	pub1 = rospy.Publisher('plot', Twist, queue_size=10)
	rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
	rospy.Subscriber("cmd_delta", Twist, callback_delta)
	rospy.Subscriber("odometry/filtered1", Odometry, callback_feedback)
	#rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback) (for simulator)
	rospy.spin()


if __name__ == '__main__':	
	start()
