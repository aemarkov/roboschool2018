#!/usr/bin/env python
#coding=utf-8
from car_helper import CarHelper
import cv2
import numpy as np

def image_callback(image):
    # ----------------------------------
    helper.motors(40, 40)
            
    cv2.imshow('img', image)
    cv2.waitKey(1)
    # ----------------------------------

if __name__ == '__main__':
    helper = CarHelper(image_callback)
    helper.run()
