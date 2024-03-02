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


class BaseClass:
    def __init__(self):
        self.__target_color = None
        self.__isRunning = False

    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return (True, ())

    def start(self):
        self.__isRunning = True
        print("Start")

    def stop(self):
        self.__isRunning = False
        print("Stop")

    def exit(self):
        self.__isRunning = False
        print("Exit")

class Perception(BaseClass): 

    def __init__(self): 
        self.__target_color = ('red', 'blue', 'green') 
        self.__isRunning = False 
        self.rect = None 
        self.size = (640, 480) 
        self.roi = () 
        self.get_roi = False 
        self.my_camera = Camera.Camera() 
        self.my_camera.camera_open()
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),}

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Iterate through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only contours with an area greater than 300 are considered valid to filter out noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour

    def draw_lines(self, img):
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        return img

    def preprocess_image(self, img):
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        return frame_lab, frame_gb

    def process_color(self, frame_gb, frame_lab, detect_color):
        if detect_color in self.__target_color:
            frame_gb = getMaskROI(frame_gb, self.rois[detect_color], self.size) if self.get_rois[detect_color] else frame_gb
            frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = self.getAreaMaxContour(contours)
            return areaMaxContour, area_max
        return None, 0  

    def process_contour(self, img, areaMaxContour, area_max, detect_color):
        if area_max > 2500:
            self.rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(self.rect))
            self.rois[detect_color] = getROI(box)
            self.get_rois[detect_color] = True
            img_centerx, img_centery = getCenter(self.rect, self.rois[detect_color], self.size, square_length)
            world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)
            self.positions[detect_color] = (img_centerx, img_centery)
            self.locations[detect_color] = (world_x, world_y)
            cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)
        return img

    def run(self, img):
        self.positions = {'red': None, 'blue': None, 'green': None}
        self.locations = {'red': None, 'blue': None, 'green': None}
        self.rois = {'red': None, 'blue': None, 'green': None}
        self.get_rois = {'red': False, 'blue': False, 'green': False}

        img_copy = img.copy()

        img = self.draw_lines(img)

        if not self.__isRunning:
            return img

        frame_lab, frame_gb = self.preprocess_image(img_copy)

        for detect_color in color_range:
            areaMaxContour, area_max = self.process_color(frame_gb, frame_lab, detect_color)
            img = self.process_contour(img, areaMaxContour, area_max, detect_color)
        
        print('Positions:', self.positions)
        print('Locations:', self.locations)

        return img

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
class Move(BaseClass):
    def __init__(self):
        # Initialize your class attributes here
        self.rect = None
        self.track = None
        self._stop = None
        self.get_roi = None  # Add get_roi attribute
        self.unreachable = None  # Add unreachable attribute
        self.__isRunning = None  # Add __isRunning attribute
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

    def process_perception_data(self, perception):
        self.detect_color = perception.__target_color
        self.locations = perception.locations

    def move_block(self, location, color):
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
            }
        
        while True:
            if __isRunning:
                if first_move and start_pick_up:  
                    action_finish = False
                    setBuzzer(0.1)
                    result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                    time.sleep(result[2]/1000)  # 返回参数的第三项为时间
                    start_pick_up = False
                    first_move = False
                    action_finish = True
                elif not first_move and not unreachable:  # 不是第一次检测到物体
                    if track:  # 如果是跟踪阶段
                        if not __isRunning:  # 停止以及退出标志位检测
                            continue
                        AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        track = False
                    if start_pick_up:  # 如果物体没有移动一段时间，开始夹取
                        action_finish = False
                        if not __isRunning:  # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)

                        # ... (continue with the rest of your code)

                        initMove()  # 回到初始位置
                        time.sleep(1.5)

                        detect_color = 'None'
                        first_move = True
                        get_roi = False
                        action_finish = True
                        start_pick_up = False
                        set_rgb(detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

def main_loop(perception, move):
    while True:
        img = perception.my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = perception.run(frame)
            move.process_perception_data(perception)
            # Add code here to use the color and location data in the Move object
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    perception.my_camera.camera_close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    move = Move()

    perception.setTargetColor('red')
    perception.start()

    move.setTargetColor('red')
    move.start()

    main_loop(perception, move)

    perception.stop()
    perception.exit()

    move.stop()
    move.exit()