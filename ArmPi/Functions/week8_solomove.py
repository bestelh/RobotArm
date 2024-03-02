#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
 
 
class Moving:
    def __init__(self):
        self.AK = ArmIK()
        self.coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        self.rotation_angle = 0
 
    def initMove(self):
        Board.setBusServoPulse(1, 500 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
 
    def open_hand(self):
        Board.setBusServoPulse(1, 500 - 280, 500)
        time.sleep(0.8)
 
    def close_hand(self):
        Board.setBusServoPulse(1, 500, 500)
        time.sleep(1)
 
    def move_to_block(self, x, y, z):
        self.open_hand()
        result = self.AK.setPitchRangeMoving((-2, 18, 1.5), -90, -90, 1000)
        if result == False:
            print("Unreachable 0")
        else:
            servo2_angle = getAngle(-2, 18, self.rotation_angle)
            Board.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(2.8)
            self.close_hand()
 
    def move_to_final_location(self, x, y, z):
        result = self.AK.setPitchRangeMoving((x, y, z + 10), -90, -90, 1000)
        if result == False:
            print("Unreachable 1")
        else:
            time.sleep(result[2]/1000)
 
    def lower_block(self, x, y, z):
        result = self.AK.setPitchRangeMoving((x, y, z), -90, -90, 1000)
        if result == False:
            print("Unreachable 2")
        else:
            time.sleep(result[2]/1000)
            self.open_hand()
 
    def pick_up_block(self, x, y, z, color):
        self.move_to_block(x, y, z)
        self.move_to_final_location(x, y, z)
        self.lower_block(x, y, z)
        self.AK.setPitchRangeMoving((x, y, z + 12), -90, -90, 0, 1000)
        time.sleep(1)
        self.initMove()
        time.sleep(1.5)
        self.close_hand()
 
if __name__ == '__main__':
    moving = Moving()
    moving.initMove()
    moving.pick_up_block(-15 + 0.5, 12 - 0.5, 1.5, 'red')