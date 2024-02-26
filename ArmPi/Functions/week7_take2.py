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
 
 
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
 
    if not __isRunning:
        return img
 
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    # If a certain area is recognized, keep detecting that area until there is none
    if get_roi:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)
 
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert the image to LAB space
 
    area_max = 0
    areaMaxContour = 0
    for i in color_range:
        if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Bitwise operation on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Opening operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closing operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find contours
                areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the largest contour
        if area_max > 2500:  # If the largest area is found
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
 
            roi = getROI(box)  # Get the ROI area
            get_roi = True
 
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # Get the center coordinates of the block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)  # Convert to real-world coordinates
 
            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)  # Draw the center point
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))  # Check if it has moved by comparing with the previous coordinates
            last_x, last_y = world_x, world_y
            track = True
            # Accumulate judgment
            if distance < 0.3:
                    # Adjust the indentation level of the code block inside the if statement
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
            else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
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