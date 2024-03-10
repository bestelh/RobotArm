#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import math
import Camera
import threading
import queue
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
    
    def __init__(self):
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

    def initBack(self):
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def openGripper(self):
        Board.setBusServoPulse(1, servo_angle - 280, 500)
        time.sleep(1)

    def closeGripper(self):
        Board.setBusServoPulse(1, servo_angle +75 , 500)

    def move_to_block(self, block_data,color):
        x = block_data[color]['location'][0]
        y = block_data[color]['location'][1]
        z=1.5
        angle_twist = block_data[color]['rotation_angle']
        AK.setPitchRangeMoving((x, y, z), -90, -90, 1000)
        self.openGripper()
        servo2_angle = getAngle(x, y, angle_twist)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(2)
        self.closeGripper()
        time.sleep(2)
        move.initBack()
        time.sleep(2)
        AK.setPitchRangeMoving(self.coordinate[color], -90, -90, 1000)
        time.sleep(2)
        self.openGripper()
        time.sleep(1)

if __name__ == "__main__":
    move = Movement()
    move.initMove()
    time.sleep(2)
