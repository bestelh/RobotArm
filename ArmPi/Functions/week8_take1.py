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
class Perception: 

    def __init__(self): 
        self.__target_color = ('red', 'blue', 'green') 
        self.__isRunning = False 
        self.rotation_angle=0
        self.positions = {'red': None, 'blue': None, 'green': None}
        self.locations = {'red': None, 'blue': None, 'green': None}
        self.rois = {'red': None, 'blue': None, 'green': None}
        self.get_rois = {'red': False, 'blue': False, 'green': False}
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


    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return (True, ())

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

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self.__isRunning = False
        print("ColorTracking Exit")

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
            self.rotation_angle = self.rect[2]
            self.positions[detect_color] = (img_centerx, img_centery)
            self.locations[detect_color] = (world_x, world_y)
            cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)
        return img
    
    def get_block_data(self):
        block_data = {}
        for color in ['red', 'blue', 'green']:
            block_data[color] = {
                'position': self.positions[color],
                'location': self.locations[color],
                'rotation_angle': self.rotation_angle
            }
        return block_data
    
    def run(self, img):
        
        img_copy = img.copy()

        img = self.draw_lines(img)

        block_data = self.get_block_data()
        print(block_data)

        if not self.__isRunning:
            return img

        frame_lab, frame_gb = self.preprocess_image(img_copy)

        for detect_color in color_range:
            areaMaxContour, area_max = self.process_color(frame_gb, frame_lab, detect_color)
            img = self.process_contour(img, areaMaxContour, area_max, detect_color)

        return img

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
        Board.setBusServoPulse(1, servo_angle +100, 500)

    def move_to_block(self, block_data,color):
        x = block_data[color]['location'][0]
        y = block_data[color]['location'][1]
        z=1.5
        angle_twist = block_data[color]['rotation_angle']
        AK.setPitchRangeMoving((x, y, z), -90, -90, 1000)
        self.openGripper()
        servo2_angle = getAngle(x, y, angle_twist)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(3)
        self.closeGripper()
        time.sleep(2)
        move.initBack()
        time.sleep(2)
        AK.setPitchRangeMoving(self.coordinate[color], -90, -90, 1000)
        time.sleep(3)
        self.openGripper()
        time.sleep(1)

block_data_queue = queue.Queue()

def main_loop(perception):
    while True:
        img = perception.my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = perception.run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            block_data = perception.get_block_data()
            block_data_queue.put(block_data)
            if key == 27:
                break
    perception.my_camera.camera_close()
    cv2.destroyAllWindows()

def move_blocks(move):
    block_data = None
    while True:
        if not block_data_queue.empty():  # Check if there is new block_data in the queue
            block_data = block_data_queue.get()  # Get the latest block_data
        if block_data is not None:
            move.move_to_block(block_data, 'red')  # Use the latest block_data to move to block
        time.sleep(2)
        move.initMove()
        time.sleep(2)
        
if __name__ == "__main__":
    perception = Perception()
    perception.start()
    move = Movement()
    move.initMove()
    time.sleep(2)

    main_loop_thread = threading.Thread(target=main_loop, args=(perception,))
    move_blocks_thread = threading.Thread(target=move_blocks, args=(move,))

    main_loop_thread.start()
    move_blocks_thread.start()

    main_loop_thread.join()
    move_blocks_thread.join()