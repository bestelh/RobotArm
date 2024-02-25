#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

class Perception:
    def __init__(self, target_color=('red', 'blue', 'green')):
        my_camera = Camera.Camera()
        my_camera.camera_open()


    def camera_open(self):
        cv2.imshow('Frame', Frame)

if __name__ == '__main__':
    try:
        perception = Perception()
        perception.camera_open()
    except KeyboardInterrupt:
        print('Program stop')
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
        sys.exit(0)