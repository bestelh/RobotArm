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
import cv2
import numpy as np
import math
import time

class ColorTracking:
    def __init__(self):
        self.color_range = {}
        self.range_rgb = {}
        self.frame_lab = None
        self.size = (640, 480)
        self.square_length = 0

        self.roi = None
        self.rect = None
        self.count = 0
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.__isRunning = False
        self.unreachable = False
        self.detect_color = 'None'
        self.action_finish = True
        self.rotation_angle = 0
        self.last_x, self.last_y = 0, 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.start_count_t1 = True
        self.t1 = 0
        self.start_pick_up = False
        self.first_move = True

    def getAreaMaxContour(self, contours):
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

    def getMaskROI(self, frame, roi, size):
        # Implement as needed
        pass

    def initMove(self):
        # Implement as needed
        pass

    def setBuzzer(self, timer):
        # Implement as needed
        pass

    def set_rgb(self, color):
        # Implement as needed
        pass

    def run(self, img):
        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img.copy(), self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = self.getMaskROI(frame_gb, self.roi, self.size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
            for i in self.color_range:
                if i in __target_color:
                    self.detect_color = i
                    frame_mask = cv2.inRange(frame_lab, self.color_range[self.detect_color][0], self.color_range[self.detect_color][1])
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)

            if area_max > 2500:
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = self.getROI(box)
                self.get_roi = True

                img_centerx, img_centery = self.getCenter(self.rect, self.roi, self.size, self.square_length)
                self.world_x, self.world_y = self.convertCoordinate(img_centerx, img_centery, self.size)

                cv2.drawContours(img, [box], -1, self.range_rgb[self.detect_color], 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detect_color], 1)
                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
                self.last_x, self.last_y = self.world_x, self.world_y
                self.track = True

                if self.action_finish:
                    if distance < 0.3:
                        self.center_list.extend((self.world_x, self.world_y))
                        self.count += 1
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1.5:
                            self.rotation_angle = self.rect[2]
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

        return img

    def main(self):
        self.init()
        self.start()
        self.__target_color = ('red', )
        my_camera = Camera.Camera()
        my_camera.camera_open()
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                Frame = self.run(frame)
                cv2.imshow('Frame', Frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        my_camera.camera_close()
        cv2.destroyAllWindows()

    def init(self):
        print("ColorTracking Init")
        self.initMove()

    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self.__isRunning = False
        print("ColorTracking Exit")

    def reset(self):
        self.count = 0
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

if __name__ == '__main__':
    tracking = ColorTracking()
    tracking.main()
