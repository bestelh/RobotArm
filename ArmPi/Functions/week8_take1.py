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
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def openGripper(self):
        Board.setBusServoPulse(1, servo_angle - 280, 500)
        time.sleep(1)

    def closeGripper(self):
        Board.setBusServoPulse(1, servo_angle, 500)

    def move_to_block(self, x, y, z):
        self.openGripper()
        result = AK.setPitchRangeMoving((-2, 18, 1.5), -90, -90, 1000)
        if result == False:
            print("Unreachable 0")
        else:
            servo2_angle = getAngle(-2, 18, self.angle)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(3)
            self.closeGripper()

if __name__ == "__main__":
    move=Movement()
    move.initMove()
    move.move_to_block(1,20,1.5)
    time.sleep(5)
    move.initMove()