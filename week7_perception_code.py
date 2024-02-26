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
import numpy as np
 
class ObjectTracker:
    def __init__(self):
        self.target_color = ('red','green','blue')
        self.color_range = {
            'red': [(0, 151, 100), (255, 255, 255)],
            'green': [(0, 0, 0), (255, 115, 255)],
            'blue': [(0, 0, 0), (255, 255, 110)],
            'black': [(0, 0, 0), (56, 255, 255)],
            'white': [(193, 0, 0), (255, 250, 255)],
            }
       
        self.size = (640, 480)
        self.square_length = 60
        self.roi = ()
        self.last_x, self.last_y = 0, 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.t1 = 0
        self.get_roi = False
        self.start_count_t1 = False
        self.center_list = []
        self.count = 0
        self.track = False
        self.rotation_angle = 0
 
    def start(self):
        print("ColorTracking Start")
 
    def run(self, img):
        frame = img.copy()
        img_h, img_w = frame.shape[:2]
        cv2.line(frame, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(frame, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
 
        frame_resize = cv2.resize(frame, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
 
        if self.get_roi:
            self.get_roi = False
            frame_gb = self.get_mask_roi(frame_gb, self.roi, self.size)
 
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
 
        area_max = 0
        areaMaxContour = 0
        for color in self.color_range:
            if color in self.target_color:
                detect_color = color
                frame_mask = cv2.inRange(frame_lab, self.color_range[detect_color][0], self.color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.get_area_max_contour(contours)
 
            if area_max > 2500:
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))
 
                self.roi = self.get_roi_from_box(box)
                self.get_roi = True
 
                img_centerx, img_centery = self.get_center(rect, self.roi, self.size, self.square_length)
                self.world_x, self.world_y = self.convert_coordinate(img_centerx, img_centery, self.size)
 
                cv2.drawContours(frame, [box], -1, (0, 0, 255), 2)
                cv2.putText(frame, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 1)
 
                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
                self.last_x, self.last_y = self.world_x, self.world_y
                self.track = True
 
                if distance < 0.3:
                    self.center_list.extend((self.world_x, self.world_y))
                    self.count += 1
                    if self.start_count_t1:
                        self.start_count_t1 = False
                        self.t1 = time.time()
                    if time.time() - self.t1 > 1.5:
                        self.rotation_angle = rect[2]
                        self.start_count_t1 = True
                        self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                        self.count = 0
                        self.center_list = []
                        self.start_pick_up = True
                else:
                    self.t1 = time.time()
                    self.start_count_t1 = True
                    self.count = 0
                    self.center_list = []
 
        return frame
 
    def get_area_max_contour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
 
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c
 
        return area_max_contour, contour_area_max
 
if __name__ == '__main__':
    tracker = ObjectTracker()
    tracker.start()
 
    my_camera = cv2.VideoCapture(0)
    while True:
        ret, frame = my_camera.read()
        if ret:
            processed_frame = tracker.run(frame)
            cv2.imshow('Frame', processed_frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
 
    my_camera.release()
    cv2.destroyAllWindows()