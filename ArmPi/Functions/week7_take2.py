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
 
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
 
__target_color = ('red', 'blue', 'green') 
# Set the detection color
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())
 
# Find the contour with the maximum area
# Parameter is a list of contours to compare
def getAreaMaxContour(contours):
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
 
 
__isRunning = False
 
def start():
    global __isRunning
    __isRunning = True
    print("ColorTracking Start")
 
def stop():
    global __isRunning
    __isRunning = False
    print("ColorTracking Stop")
 
def exit():
    global __isRunning
    __isRunning = False
    print("ColorTracking Exit")
 
rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
 
 
t1 = 0
roi = ()
get_roi = False
last_x, last_y = 0, 0
def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1

    # Initialize dictionaries to store positions and locations for each color
    positions = {'red': None, 'blue': None, 'green': None}
    locations = {'red': None, 'blue': None, 'green': None}

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    if get_roi:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)

    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    area_max = 0
    areaMaxContour = 0
    for i in color_range:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
        if area_max > 2500:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box)
            get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)

            # Store positions and locations for each color
            positions[detect_color] = (img_centerx, img_centery)
            locations[detect_color] = (world_x, world_y)

            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

    # Print positions and locations for each color
    print('Positions:', positions)
    print('Locations:', locations)

    return img
 
if __name__ == '__main__':
    start()
    __target_color = ('red', 'blue', 'green')  # Add 'blue' and 'green' to the target colors
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()