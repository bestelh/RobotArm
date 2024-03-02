#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import math
import Camera
import threading
import numpy as np
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


servo1 = 500

class Move:
    def __init__(self):
        self.rect = None
        self.track = None
        self._stop = None
        self.get_roi = None
        self.unreachable = None
        self.__isRunning = None
        self.detect_color = None
        self.action_finish = None
        self.rotation_angle = None
        self.world_X = None
        self.world_Y = None
        self.world_x = None
        self.world_y = None
        self.center_list = None
        self.count = None
        self.start_pick_up = None
        self.first_move = None

    def start(self):
        self.__isRunning = True

    def move(self, block_location, block_color):
        # 不同颜色木块放置坐标(x, y, z)
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

        while True:
            if self.__isRunning:
                if self.first_move and self.start_pick_up:  # 当首次检测到物体时
                    self.action_finish = False
                    result = AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0)
                    if result == False:
                        self.unreachable = True
                    else:
                        self.unreachable = False
                    time.sleep(result[2] / 1000)  # 返回参数的第三项为时间
                    self.start_pick_up = False
                    self.first_move = False
                    self.action_finish = True
                elif not self.first_move and not self.unreachable:  # 不是第一次检测到物体
                    if self.track:  # 如果是跟踪阶段
                        if not self.__isRunning:  # 停止以及退出标志位检测
                            continue
                        AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        self.track = False
                    if self.start_pick_up:  # 如果物体没有移动一段时间，开始夹取
                        self.action_finish = False
                        if not self.__isRunning:  # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1, 500)  # 夹持器闭合
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        result = AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.__isRunning:
                            continue
                        servo2_angle = getAngle(coordinate[self.detect_color][0], coordinate[self.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], coordinate[self.detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[self.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开，放下物体
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue                    
                        AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        initMove()  # 回到初始位置
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        self.action_finish = True
                        self.start_pick_up = False
                        set_rgb(self.detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

if __name__ == '__main__':
    move_instance = Move()
    move_instance.start()  # Start the movement
    move_location=(2, 20, 1.5)
    move_instance.move(move_location, 'red')