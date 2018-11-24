#coding=utf-8
##########################
#           ROS          #
##########################

import rospy
from car_msgs.msg import MotorsControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MAX_PWM = 255

class CarHelper:
    def __init__(self, callback):
        rospy.init_node('line_mover')
        self.callback = callback
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/motors_commands', MotorsControl, queue_size = 10)
        self.sub = rospy.Subscriber('/car_gazebo/camera1/image_raw', Image, self.image_callback)   

    def run(self):
        rate = rospy.Rate(30)
        try:
            while True:
                rate.sleep()
        except:
            pass
        finally:
            # В общем, это не работает :(
            rospy.loginfo('Stopping')
            self.motors(0,0)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')    
        self.callback(image)

    def truncate(self, pwm):
        if pwm < -MAX_PWM:
            return -MAX_PWM
        if pwm > MAX_PWM:
            return MAX_PWM
        return pwm

    def motors(self, l, r):
        msg = MotorsControl()
        msg.left = self.truncate(l)
        msg.right = self.truncate(r)
        self.pub.publish(msg)