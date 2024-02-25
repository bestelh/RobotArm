import cv2
import numpy as np
import math

class PerceptionModule:
    def __init__(self):
        self.roi = ()
        self.rect = None
        self.count = 0
        self.track = False
        self.get_roi = False
        self.center_list = []
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

    def get_area_max_contour(self, contours):
        # Implementation of getAreaMaxContour function
        pass

    def initialize_servo_positions(self):
        # Implementation of initializing servo positions
        pass

    def set_buzzer(self, timer):
        # Implementation of setting the buzzer
        pass

    def set_rgb_lights(self, color):
        # Implementation of setting RGB lights
        pass

    def reset_variables(self):
        # Implementation of resetting variables
        pass

    def process_frame(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, (640, 480), interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = self.get_mask_roi(frame_gb, self.roi, (640, 480))

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
            for i in color_range:
                if i in self.__target_color:
                    self.detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[self.detect_color][0], color_range[self.detect_color][1])
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    areaMaxContour, area_max = self.get_area_max_contour(contours)

            if area_max > 2500:
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = self.get_roi(box)
                self.get_roi = True

                img_centerx, img_centery = self.get_center(self.rect, self.roi, (640, 480), 10)
                self.world_x, self.world_y = self.convert_coordinate(img_centerx, img_centery, (640, 480))

                cv2.drawContours(img, [box], -1, (0, 0, 255), 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

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

    # Additional helper methods can be added here

if __name__ == '__main__':
    perception_module = PerceptionModule()

    # Example: Simulating image capture loop
    while True:
        img = cv2.imread("sample_image.jpg")  # Replace with actual image capture
        if img is not None:
            frame = img.copy()
            perception_module.process_frame(frame)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

    cv2.destroyAllWindows()
