#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import math
import Camera
import numpy as np
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

Gripper_servo = 1
Gripper_twist=2
Wrist=3
Elbow=4
Shoulder=5
Shoulder_twist=6


servo_angle=500

AK = ArmIK()
class Movement: 

    def _init_(self):
        self.coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        self.angle = 0

    def initMove(self):
        Board.setBusServoPulse(1, servo_angle - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def openGripper(self):
        Board.setBusServoPulse(1, servo_angle - 280, 500)
        time.sleep(1)

    def closeGripper(self):
        Board.setBusServoPulse(1, servo_angle, 500)



if __name__ == "__main__":
    move=Movement()
    move.openGripper()
    time.sleep(2)
    move.closeGripper()