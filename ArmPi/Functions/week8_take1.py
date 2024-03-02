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

def initMove():
    Board.setBusServoPulse(1, servo_angle - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

Board.setBusServoPulse(Gripper_servo, servo_angle - 280, 500)