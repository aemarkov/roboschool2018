#!/usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from car_msgs.msg import MotorsControl

motors_topic_name = '/motors_commands'
twist_topic_name = '/cmd_vel'
MAX_PWM = 255

def truncate(pwm):
	if pwm < -MAX_PWM:
		return -MAX_PWM
	if pwm > MAX_PWM:
		return MAX_PWM
	return pwm

# Convert Twit msg to MotorsControl msg
def twist_callback(msg):
	forward = msg.linear.x
	rot = msg.angular.z

	cmd = MotorsControl()	
	left  =  forward * MAX_PWM
	right =  forward * MAX_PWM
	left  -= rot * MAX_PWM
	right += rot * MAX_PWM
	cmd.left = truncate(left)
	cmd.right = truncate(right)
	pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('twist_to_motors')

	# set publishers and subscribers
	pub = rospy.Publisher(motors_topic_name, MotorsControl, queue_size=10)
	rospy.Subscriber(twist_topic_name, Twist, twist_callback)
	
	rospy.spin()