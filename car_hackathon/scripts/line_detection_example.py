#!/usr/bin/env python
#coding=utf-8

# ВНИМАНИЕ 
# Это - пример детектирования линии
# Скорее всего это не те дроиды, что ты ищешь

import rospy
from car_msgs.msg import MotorsControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

MAX_PWM = 255
is_trackbars = False

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

def create_trackbars(image):
    global is_trackbars
    if not is_trackbars:
        cv2.imshow('img', image)
        cv2.createTrackbar('H min', 'img', 0,   180, nothing)
        cv2.createTrackbar('H max', 'img', 180, 180, nothing)
        cv2.createTrackbar('S min', 'img', 0,   255, nothing)
        cv2.createTrackbar('S max', 'img', 190, 255, nothing)
        cv2.createTrackbar('V min', 'img', 0,   255, nothing)
        cv2.createTrackbar('V max', 'img', 255, 255, nothing)
        cv2.createTrackbar('Threshold', 'img', 45, 300, nothing)
        cv2.createTrackbar('Min length', 'img', 100, 300, nothing)
        cv2.createTrackbar('Max gap', 'img', 10, 300, nothing)
        cv2.createTrackbar('dilate kernel', 'img', 5, 15, nothing)

        is_trackbars = True

# Коллбек получения изображений
def image_callback(msg):
    # Преобразование сообщения в изображение OpenCV
    image = bridge.imgmsg_to_cv2(msg, 'bgr8')    
    create_trackbars(image)

    # Полученеи значений ползунков
    h_min = cv2.getTrackbarPos('H min', 'img')
    h_max = cv2.getTrackbarPos('H max', 'img')
    s_min = cv2.getTrackbarPos('S min', 'img')
    s_max = cv2.getTrackbarPos('S max', 'img')
    v_min = cv2.getTrackbarPos('V min', 'img')
    v_max = cv2.getTrackbarPos('V max', 'img')
    threshold = cv2.getTrackbarPos('Threshold', 'img')
    min_length = cv2.getTrackbarPos('Min length', 'img')
    max_gap = cv2.getTrackbarPos('Max gap', 'img')
    kernel_size = cv2.getTrackbarPos('dilate kernel', 'img')

    # Обрезка (только нижняя часть изображения)
    image_cropped = image[150:]

    # Преобразование в HSV и пороговая бинаризация
    hsv = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2HSV)
    binary = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
    binary = 255 - binary

    # Нахождение границ
    edges = cv2.Canny(binary, 100, 200)
    
    # Делаем границыы толще (иначе Хафф плохо работает)
    kernel = np.ones((kernel_size, kernel_size),np.uint8) 
    dilation = cv2.dilate(edges, kernel, iterations = 1)

    # Находим линии с помощью преобразований Хаффа
    lines = cv2.HoughLinesP(dilation, 0.5, np.pi/360.0, threshold, min_length, max_gap)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image_cropped, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
    cv2.imshow('img', image)
    cv2.imshow('binary', binary)
    cv2.imshow('edges', dilation)
    cv2.waitKey(1)

def nothing(x):
    pass

if __name__ == '__main__':
    rospy.init_node('line_mover')
    rospy.loginfo('1')

    # CvBridge служит для пребразования sensor_msgs/Image в cv2 (numpy-array)
    bridge = CvBridge()
    
    # Подписка и публикация тоиков
    pub = rospy.Publisher('/motors_commands', MotorsControl, queue_size = 10)
    sub = rospy.Subscriber('/car_gazebo/camera1/image_raw', Image, image_callback)   

    rospy.spin()