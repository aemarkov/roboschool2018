#!/usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from car_msgs.msg import MotorsControl

motors_topic_name = '/motors_commands'
twist_topic_name = '/cmd_vel'
PWM_RANGE = 255

def truncate(pwm):
	if pwm < -PWM_RANGE:
		return -PWM_RANGE
	if pwm > PWM_RANGE:
		return PWM_RANGE
	return pwm

# Convert Twit msg to MotorsControl msg
def twist_callback(msg):
	forward = msg.linear.x
	rot = msg.angular.z

	cmd = MotorsControl()	
	cmd.left  =  forward * MAX_PWM
	cmd.right =  forward * MAX_PWM
	cmd.left  += rot * MAX_PWM
	cmd.right -= rot * MAX_PWM
	cmd.left = truncate(cmd.left)
	cmd.right = truncate(cmd.right)
	pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('twist_to_motors')

	# Get parameters
	MAX_PWM = rospy.get_param('~max_pwm', 25)	
	rospy.loginfo("MAX_PWM:   %d", MAX_PWM)

	# set publishers and subscribers
	pub = rospy.Publisher(motors_topic_name, MotorsControl, queue_size=10)
	rospy.Subscriber(twist_topic_name, Twist, twist_callback)
	
	rospy.spin()