#coding=utf-8
##########################
#      RASPBERRY PI      #
##########################

import serial
from picamera.array import PiRGBArray
from picamera import PiCamera

MAX_PWM = 255

class CarHelper:
    def __init__(self, callback):
        self.uart = serial.Serial('/dev/ttyAMA0', 9600)
        self.callback = callback
        size = (320, 240)
        self.camera = PiCamera()
        self.camera.vflip = True
        self.camera.hflip = True
        self.camera.resolution = size
        self.capture = PiRGBArray(self.camera, size=size)   

    def run(self):
        try:
            for frame in self.camera.capture_continuous(self.capture, format="bgr", use_video_port=True):
                image = frame.array
                self.callback(image)
                self.capture.truncate(0)
        except:
            pass
        finally:
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
        self.uart.write(('L{};R{};'.format(self.truncate(l), self.truncate(r))).encode('ASCII'))
