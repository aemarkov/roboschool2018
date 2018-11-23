#!/usr/bin/env python
#coding=utf-8

import rospy
from car_msgs.msg import MotorsControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

MAX_PWM = 255

# Чтобы значения не выходили за допустимиые
def truncate(pwm):
    if pwm < -MAX_PWM:
        return -MAX_PWM
    if pwm > MAX_PWM:
        return MAX_PWM
    return pwm

# Управление моторами
def motors(l, r):
    msg = MotorsControl()
    msg.left = truncate(l)
    msg.right = truncate(r)
    pub.publish(msg)

# Коллбек получения изображений
def image_callback(msg):
    # Преобразование сообщения в изображение OpenCV
    image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    # TODO: Вставьте ваш код сюда
    motors(50, 50) # Пример

    # Мы можем показать изображения используя стандартные средства OpenCV
    cv2.imshow('img', image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('line_mover')

    # CvBridge служит для пребразования sensor_msgs/Image в cv2 (numpy-array)
    bridge = CvBridge()
    
    # Подписка и публикация тоиков
    pub = rospy.Publisher('/motors_commands', MotorsControl, queue_size = 10)
    sub = rospy.Subscriber('/car_gazebo/camera1/image_raw', Image, image_callback)

    rospy.spin()