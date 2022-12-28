#!/usr/bin/env python
'''
prius_pub
This code sends the value of throttle and brake and steering to gazebo.

'''
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from prius_msgs.msg import Control

'''
Node name: PRIUS_CMD
Published topic: ackerman_cmd_topic
Subscribed topic: pod_output, base_pose_ground_truth
'''
print ("Start")

act_vel_can = 0

def vel_callback(data):
	'''
	for subscribing to topic "base_pose_ground_truth
	sets steering angle according to pure pursuit controller
	'''
    global act_vel_can
    siny = +2.0 * (data.pose.pose.orientation.w * 
                    data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x * 
                    data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y * 
                        data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z * 
                        data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)
    last_recorded_vel = data.twist.twist.linear.x * \
        math.cos(yaw) + data.twist.twist.linear.y * math.sin(yaw)
    act_vel_can = last_recorded_vel
    #print("Real Vel %f" %act_vel_can)


def pid_callback(data):
	'''
	for subscribing to topic "pid_output"
	sets velocity using geometry_msgs:Twist
	'''
    global prius_vel
    global act_vel_can
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
    print "steering:", prius_vel.steer

    pub.publish(prius_vel)


if __name__ == '__main__':
    rospy.init_node('PRIUS_CMD', anonymous=True)
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
    rospy.Subscriber("pid_output", Twist, pid_callback)
    rospy.Subscriber("base_pose_ground_truth", Odometry, vel_callback)
    pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
    rospy.spin()
