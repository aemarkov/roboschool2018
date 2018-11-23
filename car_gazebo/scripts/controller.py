#!/usr/bin/env python
#coding=utf-8

import rospy
from car_msgs.msg import MotorsControl
from std_msgs.msg import Bool
import sys, select, termios, tty

def stop():
    msg = MotorsControl()
    msg.left = 0
    msg.right = 0
    stop_pub.publish(msg)

def color(red, green):
    msg_green = Bool()
    msg_red = Bool()
    msg_green.data = green
    msg_red.data = red
    green_pub.publish(msg_green)
    red_pub.publish(msg_red)

def green():
    color(False, True)

def red():
    color(True, False)

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('twist_to_motors')
    settings = termios.tcgetattr(sys.stdin)

    print('Easy contoller for gazebo car simulation')
    print('Controls:')
    print('\ts - stop the car')
    print('\tr - turn on red signal')
    print('\tg - turn on green signal')

    stop_pub = rospy.Publisher('/motors_commands', MotorsControl, queue_size=1)
    red_pub = rospy.Publisher('/light/red', Bool, queue_size=1)
    green_pub = rospy.Publisher('/light/green', Bool, queue_size=1)

    rate = rospy.Rate(5)
    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key=='g':
                green()
                rospy.loginfo('Green')
            elif key == 'r':
                red()
                rospy.loginfo('Red')
            elif key == 's':
                stop()
                rospy.loginfo('Stop')

            rate.sleep()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)